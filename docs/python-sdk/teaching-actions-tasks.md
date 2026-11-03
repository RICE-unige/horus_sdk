---
title: Teach Tasks and Actions
---

# Teach Tasks and Actions

These optional controls require the updated Teach host and MR app. Existing
profiles retain their current controls and configuration hashes.

To try the hardware-free example, use the usual ROS environment and run:

```bash
cd ~/horus_sdk
PYTHONPATH=python:$PYTHONPATH python3 python/examples/humanoid_teach_demo.py --continuous --task-actions
```

The extra fixture action only reports readiness. It does not reset joints or
publish hardware commands. The save hook prints the saved path and task.

```python
from horus.teaching import TeachAction, TeachConfig
from horus.teaching.runtime import TeachRuntime

profile = TeachConfig(
    recording_root="~/datasets/robot-teach",
    task="Put the object in the container",
    task_presets=("Put the object in the container", "Open the drawer"),
    task_management=True,
    episode_counts=True,
    actions=(TeachAction("reset_scene", "Reset scene", "Restore the integration's scene"),),
    # Configure streams, cameras and the qualified adapter as usual.
)
robot.configure_teaching(profile)

def reset_scene(context):
    if not context.is_current():
        return {"accepted": False, "reason": "Teaching session changed"}
    # Integration-owned call, with a bounded timeout and idempotency key.
    result = scene.reset(request_id=context.idempotency_key, timeout=2.0)
    if not result.applied:
        return {"accepted": False, "reason": result.error}
    return {"message": "Scene reset"}

def after_save(episode):
    # The final UUID directory and manifest already exist at this point.
    # Use the same guarded reset helper or index/export the saved episode.
    print(episode.path, episode.task, episode.quality, episode.outcome)

with TeachRuntime(robot, adapter=adapter,
                  action_handlers={"reset_scene": reset_scene},
                  after_save=after_save) as runtime:
    run_application()
```

`scene`, `adapter`, and `run_application` are supplied by your integration.
HORUS does not import handler names received from the headset. A declared button
with no installed handler is disabled with an explicit reason.

## Task selection

Click the **Task instruction** text field to open the standard Quest keyboard
(or type normally in the Unity Editor). **Save task**, or the keyboard's Done
action, asks the host to select that instruction. **Cancel**, including keyboard
cancellation, retains the host-confirmed selection. **Next task** selects an
existing instruction; **More tasks** appears when the catalogue has additional
pages. A large **Saved episodes** counter stays above the controls, outside the
scrolling list. It counts only finalized saves for this robot and the current
host-confirmed task. The task list, detailed quality/outcome counts and custom
actions scroll separately from the always-visible counter, Align, Record,
Finish & Save, Engage, Disengage and Exit controls. An unfinished task edit
disables new Record/Engage requests, not Stop or Exit. Incoming state updates do
not overwrite text while it is being edited.
Text editing and actions require an owned session, confirmed hold and no active
recording. Instructions are trimmed, limited to 4096 characters and identified
by their SHA-256. The full instruction is retained even when UI previews shorten it.

The host confirms the selected task. Record freezes that canonical text into
`manifest.json`; it rejects a stale task selection. Selecting a task does not
mutate the registration hash, change an existing episode, or move UUID folders.

## Counts

The host indexes final manifests in `recording_root/.teach-catalog.sqlite3`.
Counts are scoped to the robot name and canonical task, across sessions:

- **Saved:** a durable episode with `capture=saved`.
- **Acceptable:** saved episodes with `quality=acceptable`.
- **Successful:** saved episodes marked `outcome=success`, independently of quality.
- **Interrupted:** episodes with `capture=interrupted`.

Saved does not imply acceptable quality or successful task completion. Original
UUID folders/manifests remain authoritative. The index is refreshed at startup
on the extension worker and updated after each capture finalizes; it does not
scan the dataset on every headset heartbeat. Task lists use bounded pages of 32
previews. The current task's counts are independent of the visible task page.
An unpublished `.inprogress` directory is never counted as a durable save.
The counter updates from the host catalogue after **Finish & Save**, not from a
button click or the request acknowledgement. A recording/finalizing capture does
not add to the count. Missing or still-loading counts show **waiting**, catalogue
errors show **unavailable**, and a disconnected recorder's previous count is
labeled **Last saved count**. Profiles must set `episode_counts=True`; otherwise
the panel explicitly says **not enabled**. No collection target is hardcoded.
Malformed/unpublished manifests are excluded with a visible catalogue warning;
the catalogue does not substitute for a full MCAP integrity audit.

An alternative provider can be passed as `catalog=`. It implements `refresh()`,
`ensure_task(prompt)`, `task_prompt(task_id)`, `task_count()`,
`snapshot(robot_name, page)`, `counts(robot_name, prompt)`, `record(manifest, path)`,
`begin_callback(key)` and `complete_callback(key, status, message)`. Callback claims
and acknowledgements must be durable. Providers must bound their operations.

## Callback contract

`TeachActionContext` supplies `action_id`, `request_id`, `robot_name`, `owner`,
`session_id`, `episode_id`, `idempotency_key` and callable `is_current()`.
`SavedEpisode` supplies `episode_id`, `path`, `robot_name`, `task`, `quality`,
`outcome`, `idempotency_key` and callable `is_current()`.

Callbacks execute serially on a separate worker outside the session/control and
recorder locks. Give external operations explicit timeouts; HORUS cannot preempt
arbitrary user Python code. Shutdown waits for an executing callback. The host
blocks Record, Engage and other extension operations while a callback runs, and
continues to accept Hold and heartbeats. Check `is_current()` immediately before
an external side effect. Custom actions cannot run during an episode or while
robot motion authority is active.

Finish & Save requests hold and completes native recording first. The hook waits
for confirmed adapter hold; the UI reports `waiting_for_hold` and keeps new
Record/Engage requests blocked during that wait. Only a durable
`capture=saved` result invokes `after_save`. Its exception/decline is reported
separately and does not undo the saved episode, change its quality, or overwrite
its manifest. An interrupted capture is indexed without invoking this hook.

The catalogue durably claims an idempotency key before invoking a callback and
stores its acknowledgement. Retransmitted requests do not repeat callbacks, and
startup never replays hooks for old episodes. A crash between an external effect
and its acknowledgement leaves an uncertain claim; HORUS does not automatically
retry it. Use an idempotent external receiver with the same key. The operator can
request a new manual action after inspecting the scene. No universal exactly-once
guarantee is made for external devices.

`EpisodeCatalog.callback_result(idempotency_key)` reads a persisted acknowledgement.
A `running` claim after a crash is uncertain and is retained; it is not silently
classified as a successful external action or replayed on startup.

State includes `extension_busy`, `extension_error`, `actions`, `last_action`,
`save_hook`, `current_task`, `current_task_id`, `current_task_counts`, `tasks`,
`task_page`, `task_total`, `task_editable`, and `task_reason`. A request ACK means
the operation was admitted; `last_action.status`/`save_hook.status` report its
completion or failure. Button availability follows host state and session ownership.

`current_task_counts` is `null` until the catalogue snapshot for that task is
available, including during a task switch. Zero means a confirmed empty saved
set, not missing data. `extension_operation` identifies pending `catalog`, `task`,
`action` or `save` work. Read-only catalogue initialization/paging does not block
Align or otherwise qualified lease-gated engagement; Record and catalogue/task
mutations stay serialized. Save hooks, task changes and custom actions still
block engagement until they finish. Older hosts without this optional field keep
the app's conservative busy-state behavior.
