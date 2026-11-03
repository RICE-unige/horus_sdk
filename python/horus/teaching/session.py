"""Teach control and capture state machines, independent of ROS and Unity."""

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import threading
import math
import time
import uuid

from .adapter import ObserveAdapter, validate_pose
from .config import TeachConfig, canonical
from .recording import EpisodeRecorder
from .input import TrackingUnavailable, negotiate_input, validate_input_v2
from .alignment import validate_alignment, map_input


def _client_diagnostics(request):
    """Optional recorder evidence; malformed diagnostics never block a stop."""
    value = request.get("client_diagnostics")
    if not isinstance(value, dict):
        return {}
    return {key: value[key].encode("utf-8", errors="replace")[:256].decode("utf-8", errors="ignore")
            for key in ("build_guid", "app_version", "tracking_origin", "tracking_origin_id", "invalidation_reason")
            if isinstance(value.get(key), str)}


class TeachSession:
    MAX_PENDING_INPUT_EDGES = 64
    INPUT_BUTTONS = ("left_primary", "right_primary", "left_secondary", "right_secondary",
                     "left_stick_click", "right_stick_click")

    def __init__(self, config: TeachConfig, robot_name: str, adapter=None, recorder_factory=EpisodeRecorder,
                 *, action_handlers=None, after_save=None, catalog=None):
        self.config, self.robot_name = config, robot_name
        self.adapter = adapter or ObserveAdapter()
        if self.adapter.capabilities.adapter_id != config.adapter_id:
            raise ValueError("Installed adapter does not match declared profile")
        self.recorder_factory = recorder_factory
        self.lock = threading.RLock()
        self.io = ThreadPoolExecutor(max_workers=1, thread_name_prefix="horus-teach-io")
        self.pending = None
        self.control = "viewing"
        self.capture = "idle"
        self.quality = "pending"
        self.outcome = "unknown"
        self.error = ""
        self.revision = 0
        self.episode_id = ""
        self.owner = ""
        self.session_id = ""
        self.host_id = str(uuid.uuid4())
        self.generation = ""
        self.input_epoch = ""
        self.lease_owner = ""
        self.lease_generation = ""
        self.lease_deadline = 0.0
        self.last_input = 0.0
        self.last_seq = -1
        self.last_processed_seq = -1
        self.anchored = False
        self.latest = None
        self._input_buttons = None
        self._input_edges = []
        self.challenges = OrderedDict()
        self.requests = OrderedDict()
        self.recorder = None
        self.health = {}
        self.closed = False
        self.owner_seen = 0.0
        self.motion_authority = None
        self.hold_reason = ""
        self.received_seq = -1
        self.input_lock_wait_ms = 0.0
        self.hold_input = None
        self.next_hold_check = 0.0
        self.input_features = []
        self.pause_reason = ""
        self.next_pause_check = 0.0
        self.alignment = None
        self.client_diagnostics = {}
        self.calibration_samples = {}
        self.current_task = config.task
        self.saved_notification = None
        self.extensions = None
        if config.task_management or config.episode_counts or config.actions or after_save is not None or catalog is not None:
            from .extensions import TeachExtensions
            self.extensions = TeachExtensions(config, robot_name, action_handlers=action_handlers,
                                             after_save=after_save, catalog=catalog)
        elif action_handlers:
            raise ValueError("Declare TeachAction entries before installing action handlers")

    def append(self, sample, cache_keys=None):
        """Capture raw observations and retain bounded calibration for later episodes."""
        with self.lock:
            stream = next((s for s in self.config.streams if s.stream_id == sample.stream_id), None)
            if stream is None:
                raise ValueError("Undeclared recording stream")
            self.note_stream(sample.stream_id)
            if stream.retained_calibration:
                keys = tuple(cache_keys) if cache_keys is not None else ("latest",)
                updated = dict(self.calibration_samples)
                for key in keys:
                    if not isinstance(key, str) or not key or len(key) > 512:
                        raise ValueError("Invalid calibration cache key")
                    updated[(sample.stream_id, key)] = sample
                unique = {id(s): s for s in updated.values()}.values()
                if len(updated) > 2048 or sum(len(s.payload) + len(s.schema) + 512 for s in unique) > 16 * 1024 * 1024:
                    self.error = "Calibration snapshot cache limit exceeded"
                    self.quality = "unacceptable"
                    self._hold(self.error)
                    return False
                self.calibration_samples = updated
            return self.recorder.append(sample) if self.recorder is not None and self.capture == "recording" else False

    def _active(self):
        return self.control in {"armed", "engaged", "pause_requested", "paused"}

    def update_lease(self, owner: str, generation: str, ttl: float):
        with self.lock:
            changed = (owner, generation) != (self.lease_owner, self.lease_generation)
            self.lease_owner, self.lease_generation = owner, generation
            self.lease_deadline = time.monotonic() + min(max(ttl, 0), 3)
            if changed:
                self._event("authority_changed", {"owner": owner, "authority_generation": generation,
                                                   "remaining_seconds": min(max(ttl, 0), 3)})
            if self._active() and not self._authority():
                self._invalidate_alignment("authority changed")

    def _authority(self):
        return bool(self.owner and self.generation and self.owner == self.lease_owner and
                    self.generation == self.lease_generation and time.monotonic() < self.lease_deadline)

    def note_stream(self, stream_id: str):
        with self.lock:
            self.health[stream_id] = time.monotonic()

    def missing_streams(self, control=False):
        now = time.monotonic()
        camera_ids = {c.stream_id for c in self.config.cameras if c.required_for_control}
        return [s.stream_id for s in self.config.streams
                if ((s.stream_id in camera_ids) if control else s.required)
                and (s.stream_id not in self.health or (not s.retained_calibration
                     and now - self.health[s.stream_id] > s.max_gap_seconds))]

    def readiness(self):
        cap = self.adapter.capabilities
        try:
            ready, reason = self.adapter.ready()
        except Exception as exc:
            return False, "Controller readiness failed: " + str(exc)
        qualified = (cap.qualification == "hardware" and self.config.provenance == "real" or
                     cap.qualification == "simulation" and self.config.provenance in {"simulation", "fixture"})
        matches = (tuple(cap.joint_names) == tuple(self.config.joint_names) and
                   cap.command_frame == self.config.command_frame and
                   cap.calibration_id == self.config.calibration_id)
        controller_ready = bool(ready and qualified and matches and cap.controller_watchdog != "unavailable"
                                and cap.authority_contract != "unavailable")
        if not qualified: reason = "Adapter is not qualified for this data provenance"
        elif not matches: reason = "Adapter joint order, command frame or calibration does not match the profile"
        elif cap.controller_watchdog == "unavailable": reason = "Controller-local watchdog is unavailable"
        elif cap.authority_contract == "unavailable": reason = "Controller-scoped authority and hold are unavailable"
        if self.config.tracking_recovery == "auto_resume" and cap.supports_tracking_pause is not True:
            return False, "Adapter does not support confirmed tracking pause"
        if (self.config.tracking_recovery == "auto_resume" and self.config.provenance == "real"
                and getattr(cap, "supports_physical_tracking_recovery", False) is not True):
            return False, "Hardware adapter has not opted into physical tracking recovery"
        return controller_ready, reason if not controller_ready else ""

    def state(self, request_id="", accepted=True, reason="") -> dict:
        with self.lock:
            now = time.monotonic()
            self.challenges = OrderedDict((key, deadline) for key, deadline in self.challenges.items() if deadline > now)
            challenge = str(uuid.uuid4())
            self.challenges[challenge] = now + self.config.input_timeout_ms / 1000
            while len(self.challenges) > 32:
                self.challenges.popitem(last=False)
            ready, blocked = self.readiness()
            if self.config.workflow == "observe_record":
                ready, blocked = False, "Observation-only workflow"
            elif self.config.workflow == "collection" and (self.capture != "recording" or self.quality == "unacceptable"):
                ready, blocked = False, "Start a healthy recording before engagement"
            if self.alignment is None and self.config.workflow != "observe_record":
                ready, blocked = False, "Complete alignment before engagement"
            result = dict(schema_version=1, robot_name=self.robot_name, config_hash=self.config.digest,
                request_id=request_id, accepted=accepted, reason=reason, revision=self.revision,
                host_id=self.host_id, session_id=self.session_id, owner=self.owner, input_epoch=self.input_epoch,
                challenge=challenge, control=self.control, capture=self.capture, quality=self.quality,
                outcome=self.outcome, episode_id=self.episode_id, export="not_requested",
                control_ready=ready and not self.missing_streams(True),
                control_reason=blocked or ("Stale control streams: " + ", ".join(self.missing_streams(True)) if self.missing_streams(True) else ""),
                recording_ready=not self.missing_streams(), missing_streams=self.missing_streams(),
                stale_streams=[s.stream_id for s in self.config.streams if s.stream_id not in self.health or
                    (not s.retained_calibration and now - self.health[s.stream_id] > s.max_gap_seconds)],
                error=self.error, last_hold_reason=self.hold_reason,
                input_health=self._input_health(now), last_hold_input=self.hold_input)
            result.update(alignment_schema_version=1, alignment_required=True,
                          alignment_id=self.alignment["alignment_id"] if self.alignment else "",
                          tracking_origin_id=self.alignment["tracking_origin_id"] if self.alignment else "")
            if self.config.input_schema_version == 2:
                result.update(input_schema_version=2, input_features=list(self.input_features),
                              required_input_features=list(self.config.required_input_features))
                result["engagement_mode"] = self.config.engagement_mode
            if self.config.tracking_recovery != "manual":
                result.update(tracking_recovery=self.config.tracking_recovery, pause_reason=self.pause_reason)
            if self.extensions is not None:
                ext = self.extensions
                extension_busy = ext.busy or self.saved_notification is not None
                if ext.blocks_control or self.saved_notification is not None:
                    result["control_ready"] = False
                    result["control_reason"] = "Waiting for task or action operation"
                blocked_action = ("Waiting for extension operation" if extension_busy else
                    "Finish this episode first" if self.capture in {"preparing", "recording", "finalizing"} else
                    "Disengage and confirm hold first" if self.motion_authority is not None or self.control not in {"viewing", "holding"} else "")
                result.update(extensions_version=1, current_task=self.current_task,
                    current_task_counts=ext.counts, task_page=ext.page, task_total=ext.task_total,
                    tasks=ext.tasks if self.config.task_management or self.config.episode_counts else [],
                    extension_busy=extension_busy, extension_error=ext.error or ext.catalog_warning,
                    extension_operation=ext.kind if ext.busy else "save" if self.saved_notification is not None else "",
                    task_editable=self.config.task_management and not blocked_action,
                    task_reason=blocked_action, last_action=ext.last_action, save_hook=ext.save_hook,
                    actions=[dict(action_id=a.action_id,
                        enabled=not blocked_action and a.action_id in ext.handlers,
                        reason=blocked_action or ("No handler installed" if a.action_id not in ext.handlers else ""))
                        for a in self.config.actions])
                from .extensions import task_identity
                result["current_task_id"] = task_identity(self.current_task) if self.current_task.strip() else ""
            return result

    def request(self, request: dict) -> dict:
        with self.lock:
            if not isinstance(request, dict):
                return self.state(accepted=False, reason="Request must be an object")
            request_id = request.get("request_id", "")
            fingerprint = None
            try:
                uuid.UUID(request_id)
                fingerprint = canonical(request)
                if request_id in self.requests:
                    previous, accepted, reason = self.requests[request_id]
                    if previous != fingerprint:
                        return self.state(request_id, False, "Request ID reused with different content")
                    return self.state(request_id, accepted, reason)
                if self.closed:
                    raise ValueError("Session is closed")
                if request.get("config_hash") != self.config.digest:
                    raise ValueError("Registration/profile changed; reopen Teach")
                action = request.get("action")
                if action == "open":
                    features = negotiate_input(self.config, request)
                    if not isinstance(request.get("owner"), str) or not request["owner"] or len(request["owner"]) > 128:
                        raise ValueError("Operator identity is required")
                    if self.owner and self.owner != request["owner"]:
                        raise ValueError("Another operator owns this teaching session")
                    if not self.owner:
                        if self.capture in {"preparing", "recording", "finalizing"}:
                            raise ValueError("Previous session is still finalizing")
                        self.owner = request["owner"]
                        self.session_id = str(uuid.uuid4())
                        self.control = "viewing"
                        self.input_features = features
                        self.alignment = None
                        self.client_diagnostics = _client_diagnostics(request)
                    elif features != self.input_features:
                        raise ValueError("Input capabilities changed; close and reopen Teach")
                else:
                    if request.get("session_id") != self.session_id or request.get("owner") != self.owner or not self.owner:
                        raise ValueError("Wrong teaching session or operator")
                    # Hold is always admissible for this session, even with stale UI state.
                    if action not in {"hold", "heartbeat", "invalidate_alignment"} and request.get("expected_revision") != self.revision:
                        raise ValueError("State changed; refresh before retrying")
                    self._transition(action, request)
                self.owner_seen = time.monotonic()
                if action != "heartbeat": self.revision += 1
                result = (fingerprint, True, "")
            except (ValueError, TypeError, AttributeError, RuntimeError, OSError) as exc:
                result = (fingerprint, False, str(exc))
            if isinstance(request_id, str) and request_id and fingerprint is not None:
                self.requests[request_id] = result
                while len(self.requests) > 256:
                    self.requests.popitem(last=False)
            return self.state(request_id, result[1], result[2])

    def _transition(self, action, request):
        catalogue_control = (self.extensions is not None and self.extensions.busy and
            not self.extensions.blocks_control and self.saved_notification is None and action in {"align", "engage"})
        if self.extensions is not None and (self.extensions.busy or self.saved_notification is not None) and not catalogue_control and action not in {"hold", "heartbeat", "invalidate_alignment", "close"}:
            raise ValueError("Waiting for extension operation to finish")
        if action == "heartbeat":
            return
        elif action == "hold":
            self._hold("operator")
            if self.motion_authority is None and self.control == "faulted":
                self.control, self.error = "holding", ""
        elif action == "align":
            if self.motion_authority is not None or self.control not in {"viewing", "holding"}:
                raise ValueError("Confirm hold before alignment")
            self.alignment = validate_alignment(request.get("alignment"))
            self.client_diagnostics = _client_diagnostics(request)
            self._event("alignment", dict(self.alignment, owner=self.owner, session_id=self.session_id,
                                          config_hash=self.config.digest, client_diagnostics=dict(self.client_diagnostics)))
        elif action == "invalidate_alignment":
            self._invalidate_alignment("tracking alignment invalidated", _client_diagnostics(request))
        elif action == "engage":
            if self.motion_authority is not None or self.control not in {"viewing", "holding"}:
                raise ValueError("Confirm hold before engaging again")
            self.generation = request.get("authority_generation", "")
            if not self._authority():
                raise ValueError("A current positive robot lease is required")
            if self.alignment is None or request.get("alignment_id") != self.alignment["alignment_id"]:
                raise ValueError("Complete alignment before engagement")
            ready, reason = self.readiness()
            if not ready or self.missing_streams(True):
                raise ValueError(reason or "Required control stream is stale")
            if self.config.workflow == "observe_record":
                raise ValueError("This workflow is observation only")
            if self.config.workflow == "collection" and (self.capture != "recording" or self.quality == "unacceptable"):
                raise ValueError("Healthy recording must be confirmed before engagement")
            self.control = "armed"
            self.motion_authority = dict(robot_name=self.robot_name, owner=self.owner,
                session_id=self.session_id, authority_generation=self.generation)
            self.input_epoch = str(uuid.uuid4())
            self.last_seq, self.latest, self.anchored = -1, None, False
            self.last_processed_seq, self.pause_reason = -1, ""
            self.last_input = time.monotonic()
            self._event("armed", {"authority_generation": self.generation})
        elif action == "record":
            if self.capture not in {"idle", "saved", "interrupted"} or self.pending is not None:
                raise ValueError("Recorder is busy")
            if self.missing_streams():
                raise ValueError("Required recording streams are stale")
            self._hold("new episode")
            episode_id = request.get("episode_id", "")
            uuid.UUID(episode_id)
            task = request.get("task") or self.config.task
            if self.config.task_management:
                from .extensions import task_identity
                task_identity(self.current_task)
                if request.get("task") != self.current_task:
                    raise ValueError("Task changed; refresh before recording")
                task = self.current_task
            if not isinstance(task, str) or len(task) > 4096:
                raise ValueError("Task must be an instruction of at most 4096 characters")
            if self.extensions is not None and not self.config.task_management and task.strip() and task != self.current_task:
                self.extensions.select_task(task)
            self.episode_id = episode_id
            self.current_task = task
            if self.extensions is not None: self.extensions.current_task = task
            self.capture, self.quality, self.outcome, self.error = "preparing", "pending", "unknown", ""
            self.pending = self.io.submit(self.recorder_factory, self.config, self.robot_name,
                                          episode_id, task)
        elif action in {"select_task", "list_tasks", "custom_action"}:
            if self.extensions is None:
                raise ValueError("Teach extensions are not configured")
            if self.capture in {"preparing", "recording", "finalizing"}:
                raise ValueError("Finish this episode before changing tasks or running actions")
            if self.motion_authority is not None or self.control not in {"viewing", "holding"}:
                raise ValueError("Disengage and confirm hold before running this action")
            if action == "list_tasks":
                if not self.config.task_management: raise ValueError("Task selection is not configured")
                self.extensions.list_tasks(request.get("task_page", 0))
            elif action == "select_task":
                if not self.config.task_management: raise ValueError("Task editing is not configured")
                identity = request.get("task_id", "")
                if not isinstance(identity, str) or (identity and (len(identity) != 64 or any(c not in "0123456789abcdef" for c in identity))):
                    raise ValueError("Invalid task ID")
                self.extensions.select_task(request.get("task"), identity)
            else:
                from .extensions import TeachActionContext
                action_id = request.get("action_id")
                if action_id not in {a.action_id for a in self.config.actions}: raise ValueError("Unknown Teach action")
                owner, session_id, episode_id = self.owner, self.session_id, self.episode_id
                current = lambda: self._extension_current(owner, session_id, episode_id)
                self.extensions.action(TeachActionContext(action_id, request["request_id"], self.robot_name,
                    owner, session_id, episode_id, current))
        elif action == "finish":
            if self.capture != "recording" or self.recorder is None:
                raise ValueError("No active recording to finish")
            outcome = request.get("outcome", "unknown")
            if outcome not in {"unknown", "success", "failure", "abort"}:
                raise ValueError("Invalid outcome")
            self._hold("finish")
            self.outcome = outcome
            self._event("finish_requested", {"outcome": outcome, "control": self.control})
            self.capture = "finalizing"
            self.pending = self.io.submit(self.recorder.finish, outcome)
        elif action == "close":
            self._hold("exit")
            if self.capture in {"preparing", "recording", "finalizing"}:
                raise ValueError("Finish the recording before closing the session")
            self.owner, self.session_id = "", ""
            self.alignment = None
            self.input_features = []
            self.control = "viewing"
        else:
            raise ValueError("Unknown teaching action")

    def _event(self, kind, value):
        if self.recorder is not None and self.capture == "recording":
            self.recorder.event(kind, value)

    def _extension_current(self, owner, session_id, episode_id):
        with self.lock:
            return (not self.closed and self.owner == owner and self.session_id == session_id and
                self.episode_id == episode_id and self.motion_authority is None and
                self.control in {"viewing", "holding"} and self.capture in {"idle", "saved", "interrupted"})

    def _invalidate_alignment(self, reason, client_diagnostics=None):
        self.alignment = None
        self._hold(reason)
        self._event("alignment_invalidated", {"reason": reason,
            "client_diagnostics": dict(self.client_diagnostics if client_diagnostics is None else client_diagnostics)})

    def _hold(self, reason):
        self.latest = None
        self._input_buttons = None
        self._input_edges.clear()
        self.input_epoch = ""
        self.anchored = False
        self.pause_reason = ""
        if self.motion_authority is None:
            # Observation and recording must never send unsolicited robot commands.
            return
        self.hold_reason = reason
        self.hold_input = self._input_health(time.monotonic())
        self._confirm_hold()
        self._event("hold", {"reason": reason, "control": self.control})

    def _pause(self, reason):
        if self.config.tracking_recovery != "auto_resume":
            self._hold(reason)
            return
        self.latest = None
        self._input_buttons = None
        self._input_edges.clear()
        if self.motion_authority is None or self.control in {"pause_requested", "paused"}:
            return
        self.pause_reason = reason
        # Only a new challenge issued after the pause can resume this authority.
        self.challenges.clear()
        self.control = "pause_requested"
        self._confirm_pause()
        self._event("tracking_pause", {"reason": reason, "control": self.control})

    def _confirm_pause(self):
        self.next_pause_check = time.monotonic() + .1
        try:
            # No anchor has reached the adapter yet, so there is no motion to pause.
            if not self.anchored or self.adapter.pause(dict(self.motion_authority, reason=self.pause_reason)) is True:
                self.control = "paused"
                self.challenges.clear()
        except Exception as exc:
            self._hold("adapter pause fault")
            self.control, self.error = "faulted", str(exc)

    def _confirm_hold(self):
        self.next_hold_check = time.monotonic() + 0.1
        self.control = "hold_requested"
        try:
            if self.adapter.hold(dict(self.motion_authority, reason=self.hold_reason)) is True:
                self.control = "holding"
                self.motion_authority = None
        except Exception as exc:
            self.control, self.error = "faulted", str(exc)

    def _input_health(self, now):
        return dict(received_sequence=self.received_seq, accepted_sequence=self.last_seq,
                    accepted_age_ms=max(0.0, (now-self.last_input)*1000) if self.last_seq >= 0 else -1.0,
                    callback_lock_wait_ms=self.input_lock_wait_ms)

    def input(self, sample: dict) -> bool:
        received = time.monotonic()
        with self.lock:
            if not isinstance(sample, dict):
                return False
            if (self.owner and sample.get("session_id") == self.session_id and sample.get("owner") == self.owner
                    and type(sample.get("sequence")) is int):
                self.received_seq = sample["sequence"]
                self.input_lock_wait_ms = max(0.0, (time.monotonic()-received)*1000)
            if not self._active():
                return False
            now = time.monotonic()
            if (sample.get("session_id") != self.session_id or sample.get("input_epoch") != self.input_epoch or
                sample.get("owner") != self.owner or not self._authority()):
                return False
            try:
                seq = sample["sequence"]
                if type(seq) is not int or seq <= self.last_processed_seq:
                    return False
                self.last_processed_seq = seq
                try:
                    sample = map_input(sample, self.alignment)
                except (KeyError, TypeError, ValueError):
                    self._invalidate_alignment("invalid alignment or tracking pose")
                    raise
                if self.config.input_schema_version == 2:
                    validate_input_v2(self.config, sample)
                elif sample.get("input_schema_version", 1) != 1:
                    raise ValueError("Expected legacy Teach input")
                if self.challenges.get(sample.get("challenge"), 0) <= now:
                    self._pause("stale transport sample")
                    return False
                if self.config.engagement_mode == "hold_to_run" and self.control == "armed" and sample.get("clutch") is False:
                    return False
                if sample.get("clutch") is not True:
                    self._hold("clutch released")
                    return False
                if sample.get("tracking_valid") is not True or sample.get("ui_focus") is not False:
                    self.owner_seen = now
                    self._pause("tracking or UI focus unavailable")
                    return False
                for hand in self.config.controlled_hands:
                    validate_pose(sample[hand])
                tools = {}
                for name, binding in self.config.grasp_bindings.items():
                    value = sample.get(binding)
                    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1:
                        raise ValueError("Missing or invalid configured grasp axis: " + binding)
                    tools[name] = value
                if self.control == "pause_requested":
                    return False
                if self.control == "paused":
                    ready, blocked = self.readiness()
                    if not ready or self.missing_streams(True):
                        self._hold("controller unavailable: " + blocked if not ready else "stale control streams")
                        return False
                    self._event("tracking_resumed", {"sequence": seq, "reason": self.pause_reason})
                    self.pause_reason = ""
                # Keep discrete transitions, not a backlog of obsolete motion poses.
                sample.pop("input_edges", None)
                if self.config.input_schema_version == 2:
                    buttons = {name: sample[name] for name in self.INPUT_BUTTONS}
                    edges = [] if self._input_buttons is None else [
                        dict(button=name, pressed=value, sequence=seq,
                             source_monotonic_seconds=sample["source_monotonic_seconds"])
                        for name, value in buttons.items() if value != self._input_buttons[name]]
                    if len(self._input_edges) + len(edges) > self.MAX_PENDING_INPUT_EDGES:
                        self._hold("input edge queue overflow")
                        self.error = "Control worker did not consume accepted input edges"
                        return False
                    self._input_edges.extend(edges)
                    self._input_buttons = buttons
                self.last_seq, self.last_input = seq, now
                self.owner_seen = now
                self.control = "engaged"
                self.latest = dict(sample)
                self.latest["tools"] = tools
                self.latest["authority"] = dict(self.motion_authority)
                self._event("operator_intent", sample)
                return True
            except TrackingUnavailable as exc:
                self.owner_seen = now
                if sample.get("clutch") is True:
                    self._pause(str(exc))
                else:
                    self._hold("clutch released")
                return False
            except (KeyError, TypeError, ValueError) as exc:
                self._hold("invalid input")
                self.error = str(exc)
                return False

    def tick(self):
        """Called by the robot-local worker, never by the disk writer."""
        with self.lock:
            if self.extensions is not None and self.extensions.pending is not None and self.extensions.pending.done():
                selected_task = self.extensions.poll()
                if selected_task is not None: self.current_task = selected_task
                self.revision += 1
            if self.recorder is not None and (self.recorder.error or self.recorder.dropped):
                self.error = self.recorder.error or f"Recorder queue dropped {self.recorder.dropped} samples"
                self.quality = "unacceptable"
                if self.config.workflow == "collection" and self._active():
                    self._hold("recorder failure")
            if self.control == "hold_requested" and time.monotonic() >= self.next_hold_check:
                self._confirm_hold()
            if self.owner and time.monotonic() - self.owner_seen > 5:
                self._invalidate_alignment("operator disconnected")
                self.owner, self.session_id = "", ""
                self.input_features = []
                if self.capture == "recording" and self.recorder is not None:
                    self.capture = "finalizing"
                    self.pending = self.io.submit(self.recorder.finish, "abort", interrupted=True)
                self.revision += 1
            if self._active():
                ready, blocked = self.readiness()
                missing = self.missing_streams(True)
                stop_reason = ("authority expired" if not self._authority() else
                               "controller unavailable: " + blocked if not ready else
                               "stale control streams: " + ", ".join(missing) if missing else "")
                if stop_reason:
                    if not self._authority():
                        self._invalidate_alignment(stop_reason)
                    else:
                        self._hold(stop_reason)
                elif self.control == "pause_requested" and time.monotonic() >= self.next_pause_check:
                    self._confirm_pause()
                elif self.control == "engaged" and time.monotonic() - self.last_input > self.config.input_timeout_ms / 1000:
                    self._pause("input watchdog expired")
                elif self.latest is not None:
                    sample, self.latest = self.latest, None
                    if self.config.input_schema_version == 2:
                        sample["input_edges"] = self._input_edges
                        self._input_edges = []
                    try:
                        if not self.anchored:
                            self.adapter.anchor(sample)
                            self.anchored = True
                            self._event("anchored", {"sequence": sample["sequence"]})
                        else:
                            command = self.adapter.apply(sample)
                            self._event("command_result", command)
                    except Exception as exc:
                        self._hold("adapter fault")
                        self.error = str(exc)
                        self.control = "faulted"
            if self.pending is not None and self.pending.done():
                pending, self.pending = self.pending, None
                try:
                    result = pending.result()
                    if self.capture == "preparing":
                        self.recorder, self.capture = result, "recording"
                        from dataclasses import asdict
                        self._event("adapter_contract", asdict(self.adapter.capabilities))
                        self._event("recording_started", {"session_id": self.session_id,
                            "lease_owner": self.lease_owner, "authority_generation": self.lease_generation,
                            "client_diagnostics": dict(self.client_diagnostics)})
                        if self.alignment is not None:
                            self._event("alignment", dict(self.alignment, owner=self.owner,
                                session_id=self.session_id, config_hash=self.config.digest,
                                client_diagnostics=dict(self.client_diagnostics)))
                        snapshots = {id(s): s for s in self.calibration_samples.values()}.values()
                        for sample in sorted(snapshots, key=lambda s: s.received_monotonic_ns):
                            self.recorder.append(sample)
                            self._event("calibration_snapshot", dict(stream_id=sample.stream_id,
                                original_receipt_monotonic_ns=sample.received_monotonic_ns,
                                original_receipt_unix_ns=sample.received_unix_ns))
                        if not self.owner and not self.closed:
                            self.capture = "finalizing"
                            self.pending = self.io.submit(self.recorder.finish, "abort", interrupted=True)
                    else:
                        self.capture, self.quality = result["capture"], result["quality"]
                        if self.extensions is not None:
                            self.saved_notification = (result, self.recorder.path, self.owner, self.session_id, self.episode_id)
                            self.extensions.save_hook = dict(episode_id=self.episode_id, status="waiting_for_hold", message="Waiting for confirmed hold")
                        self.recorder = None
                except Exception as exc:
                    self.capture, self.quality, self.error = "interrupted", "unacceptable", str(exc)
                self.revision += 1
            if self.saved_notification is not None and (self.closed or
                    (self.motion_authority is None and self.control in {"viewing", "holding"})):
                result, path, owner, session_id, episode_id = self.saved_notification
                self.saved_notification = None
                try:
                    self.extensions.saved(result, path,
                        lambda: self._extension_current(owner, session_id, episode_id))
                except Exception as exc:
                    self.extensions.error = f"After-save scheduling failed: {type(exc).__name__}: {exc}"[:512]
                    self.extensions.save_hook = dict(episode_id=episode_id, status="failed", message=self.extensions.error)

    def close(self):
        with self.lock:
            self.closed = True
            self._invalidate_alignment("runtime shutdown")
            self.owner, self.session_id = "", ""
            self.input_features = []
        self.io.shutdown(wait=True)
        self.tick()
        if self.recorder is not None and self.capture == "recording":
            result = self.recorder.finish("abort", interrupted=True)
            self.capture, self.quality = result["capture"], result["quality"]
            self.recorder = None
        if self.extensions is not None:
            self.extensions.close()
