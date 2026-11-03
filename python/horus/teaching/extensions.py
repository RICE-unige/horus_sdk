"""Optional task catalogue and serial callbacks, independent of ROS and control."""

from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sqlite3
from typing import Callable
import uuid


def task_identity(prompt: str) -> str:
    if not isinstance(prompt, str) or not prompt.strip() or len(prompt) > 4096:
        raise ValueError("Task must contain 1-4096 characters")
    return hashlib.sha256(prompt.strip().encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class TeachActionContext:
    action_id: str
    request_id: str
    robot_name: str
    owner: str
    session_id: str
    episode_id: str
    is_current: Callable[[], bool]

    @property
    def idempotency_key(self):
        return "action:" + self.request_id


@dataclass(frozen=True)
class SavedEpisode:
    episode_id: str
    path: Path
    robot_name: str
    task: str
    quality: str
    outcome: str
    is_current: Callable[[], bool]

    @property
    def idempotency_key(self):
        return "save:" + self.episode_id


class EpisodeCatalog:
    """Derived task/count index. Original UUID folders and manifests stay authoritative."""

    def __init__(self, root):
        self.root = Path(root).expanduser()
        self.path = self.root / ".teach-catalog.sqlite3"
        self.issues = []

    @contextmanager
    def _connect(self):
        self.root.mkdir(parents=True, exist_ok=True)
        db = sqlite3.connect(self.path, timeout=2)
        db.execute("CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, prompt TEXT NOT NULL)")
        db.execute("CREATE TABLE IF NOT EXISTS episodes (id TEXT PRIMARY KEY, task_id TEXT, robot TEXT, capture TEXT, quality TEXT, outcome TEXT, path TEXT)")
        db.execute("CREATE INDEX IF NOT EXISTS episode_robot_task ON episodes(robot, task_id)")
        db.execute("CREATE TABLE IF NOT EXISTS callbacks (id TEXT PRIMARY KEY, status TEXT NOT NULL, message TEXT NOT NULL)")
        try:
            with db: yield db
        finally:
            db.close()

    def ensure_task(self, prompt):
        prompt = prompt.strip() if isinstance(prompt, str) else prompt
        identity = task_identity(prompt)
        with self._connect() as db:
            db.execute("INSERT OR IGNORE INTO tasks VALUES (?, ?)", (identity, prompt))
        return identity

    def record(self, manifest, path):
        episode_id = manifest["episode_id"]
        uuid.UUID(episode_id)
        if manifest.get("capture") == "saved" and Path(path).name != episode_id:
            raise ValueError("Saved episode is not published in its final UUID directory")
        prompt = manifest.get("task", "")
        identity = self.ensure_task(prompt) if isinstance(prompt, str) and prompt.strip() else ""
        with self._connect() as db:
            db.execute("INSERT OR REPLACE INTO episodes VALUES (?, ?, ?, ?, ?, ?, ?)",
                (episode_id, identity, manifest.get("robot_name", ""), manifest.get("capture", ""),
                 manifest.get("quality", ""), manifest.get("outcome", ""), str(path)))

    def refresh(self):
        # Run only on the extension worker, never per heartbeat or on the control thread.
        self.root.mkdir(parents=True, exist_ok=True)
        self.issues = []
        for folder in self.root.iterdir():
            if not folder.is_dir(): continue
            try: uuid.UUID(folder.name.removesuffix(".inprogress"))
            except ValueError: continue
            path = folder / "manifest.json"
            try:
                if path.stat().st_size > 1024 * 1024: raise ValueError("Manifest exceeds catalogue budget")
                manifest = json.loads(path.read_text(encoding="utf-8"))
                if manifest.get("capture") not in {"saved", "interrupted"}: continue
                self.record(manifest, folder)
            except (OSError, ValueError, TypeError, KeyError):
                if not folder.name.endswith(".inprogress") or path.exists(): self.issues.append(folder.name)
        with self._connect() as db:
            for identity, path in db.execute("SELECT id, path FROM episodes").fetchall():
                if not (Path(path) / "manifest.json").is_file():
                    db.execute("DELETE FROM episodes WHERE id=?", (identity,))

    def snapshot(self, robot_name, page=0):
        with self._connect() as db:
            rows = db.execute("""SELECT t.id, t.prompt,
                SUM(CASE WHEN e.capture='saved' THEN 1 ELSE 0 END),
                SUM(CASE WHEN e.capture='saved' AND e.quality='acceptable' THEN 1 ELSE 0 END),
                SUM(CASE WHEN e.capture='saved' AND e.outcome='success' THEN 1 ELSE 0 END),
                SUM(CASE WHEN e.capture='interrupted' THEN 1 ELSE 0 END)
                FROM tasks t LEFT JOIN episodes e ON e.task_id=t.id AND e.robot=?
                GROUP BY t.id ORDER BY t.prompt LIMIT 32 OFFSET ?""", (robot_name, page * 32)).fetchall()
        return [dict(task_id=r[0], prompt=r[1][:256], saved=r[2], acceptable=r[3], successful=r[4], interrupted=r[5]) for r in rows]

    def task_count(self):
        with self._connect() as db:
            return db.execute("SELECT COUNT(*) FROM tasks").fetchone()[0]

    def counts(self, robot_name, prompt):
        identity = task_identity(prompt) if prompt.strip() else ""
        with self._connect() as db:
            row = db.execute("""SELECT
                COALESCE(SUM(capture='saved'),0),
                COALESCE(SUM(capture='saved' AND quality='acceptable'),0),
                COALESCE(SUM(capture='saved' AND outcome='success'),0),
                COALESCE(SUM(capture='interrupted'),0)
                FROM episodes WHERE robot=? AND task_id=?""", (robot_name, identity)).fetchone()
        return dict(saved=row[0], acceptable=row[1], successful=row[2], interrupted=row[3])

    def task_prompt(self, identity):
        with self._connect() as db:
            row = db.execute("SELECT prompt FROM tasks WHERE id=?", (identity,)).fetchone()
        if row is None: raise ValueError("Unknown task ID")
        return row[0]

    def begin_callback(self, key):
        with self._connect() as db:
            return db.execute("INSERT OR IGNORE INTO callbacks VALUES (?, 'running', '')", (key,)).rowcount == 1

    def complete_callback(self, key, status, message):
        with self._connect() as db:
            db.execute("UPDATE callbacks SET status=?, message=? WHERE id=?", (status, message, key))

    def callback_result(self, key):
        with self._connect() as db:
            row = db.execute("SELECT status, message FROM callbacks WHERE id=?", (key,)).fetchone()
        return dict(status=row[0], message=row[1]) if row else None


class TeachExtensions:
    def __init__(self, config, robot_name, *, action_handlers=None, after_save=None, catalog=None):
        self.config, self.robot_name = config, robot_name
        self.handlers = dict(action_handlers or {})
        ids = {a.action_id for a in config.actions}
        if set(self.handlers) - ids or any(not callable(v) for v in self.handlers.values()):
            raise ValueError("Action handlers must be callables for declared TeachAction IDs")
        if after_save is not None and not callable(after_save): raise ValueError("after_save must be callable")
        self.after_save = after_save
        self.catalog = catalog or EpisodeCatalog(config.recording_root)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="horus-teach-actions")
        self.tasks, self.error, self.last_action, self.save_hook = [], "", None, None
        self.catalog_warning = ""
        self.page, self.task_total, self.current_task = 0, 0, config.task
        self.counts = None
        self.pending = self.executor.submit(self._initialize)
        self.kind = "catalog"

    @property
    def busy(self): return self.pending is not None

    @property
    def blocks_control(self): return self.busy and self.kind != "catalog"

    def _initialize(self):
        if self.config.task_management or self.config.episode_counts:
            self.catalog.refresh()
        for prompt in (self.config.task, *self.config.task_presets):
            if prompt.strip(): self.catalog.ensure_task(prompt)
        return self._snapshot(self.current_task)

    def _snapshot(self, prompt):
        if not self.config.task_management and not self.config.episode_counts:
            return dict(tasks=[], task_total=0, counts=dict(saved=0, acceptable=0, successful=0, interrupted=0))
        return dict(tasks=self.catalog.snapshot(self.robot_name, self.page),
                    task_total=self.catalog.task_count(), counts=self.catalog.counts(self.robot_name, prompt),
                    catalog_warning=(f"Catalogue excludes {len(self.catalog.issues)} invalid or unpublished manifests" if getattr(self.catalog, "issues", ()) else ""))

    def list_tasks(self, page):
        if type(page) is not int or page < 0 or page * 32 >= max(1, self.task_total):
            raise ValueError("Task page is out of range")
        self.page = page
        self.kind = "catalog"
        self.pending = self.executor.submit(self._snapshot, self.current_task)

    def select_task(self, prompt, identity=""):
        if not identity: task_identity(prompt)
        self.counts = None
        def select():
            selected = self.catalog.task_prompt(identity) if identity else prompt.strip()
            self.catalog.ensure_task(selected)
            return dict(current_task=selected, **self._snapshot(selected))
        self.kind = "task"
        self.pending = self.executor.submit(select)

    def _callback(self, callback, context):
        if not context.is_current(): return dict(status="cancelled", message="Session changed before action started")
        if not self.catalog.begin_callback(context.idempotency_key):
            return dict(status="not_replayed", message="This callback was already claimed; inspect its previous acknowledgement")
        try:
            if not context.is_current():
                self.catalog.complete_callback(context.idempotency_key, "cancelled", "Session changed before callback execution")
                return dict(status="cancelled", message="Session changed before callback execution")
            result = callback(context)
            if result is not None and not isinstance(result, (str, dict, bool)):
                raise TypeError("Action must return None, a message or an acknowledgement object")
            if isinstance(result, dict) and "accepted" in result and type(result["accepted"]) is not bool:
                raise TypeError("Action accepted field must be boolean")
            if result is False or (isinstance(result, dict) and result.get("accepted") is False):
                raise RuntimeError(str(result.get("reason", "Action was declined")) if isinstance(result, dict) else "Action was declined")
            message = result.get("message", "Completed") if isinstance(result, dict) else result if isinstance(result, str) else "Completed"
            status, message = "completed", str(message)[:512]
        except Exception as exc:
            status, message = "failed", f"{type(exc).__name__}: {exc}"[:512]
        self.catalog.complete_callback(context.idempotency_key, status, message)
        return dict(status=status, message=message)

    def action(self, context):
        callback = self.handlers.get(context.action_id)
        if callback is None: raise ValueError("No handler is installed for this action")
        self.last_action = dict(action_id=context.action_id, request_id=context.request_id, status="running", message="")
        self.kind = "action"
        self.pending = self.executor.submit(self._callback, callback, context)

    def saved(self, manifest, path, current):
        episode = SavedEpisode(manifest["episode_id"], Path(path), self.robot_name,
            manifest.get("task", ""), manifest.get("quality", ""), manifest.get("outcome", ""), current)
        self.save_hook = dict(episode_id=episode.episode_id, status="running" if self.after_save else "not_configured", message="")
        def complete():
            self.catalog.record(manifest, path)
            result = self._callback(self.after_save, episode) if self.after_save and manifest.get("capture") == "saved" else dict(status="not_configured", message="")
            return dict(hook=result, **self._snapshot(self.current_task))
        self.kind = "save"
        self.pending = self.executor.submit(complete)

    def poll(self):
        if self.pending is None or not self.pending.done(): return None
        future, kind = self.pending, self.kind
        self.pending = None
        try:
            value = future.result()
            self.error = ""
            if "tasks" in value: self.tasks = value["tasks"]
            if "catalog_warning" in value: self.catalog_warning = value["catalog_warning"]
            if "counts" in value: self.counts = value["counts"]
            if "task_total" in value: self.task_total = value["task_total"]
            if "current_task" in value: self.current_task = value["current_task"]
            if kind == "action": self.last_action.update(value)
            if kind == "save": self.save_hook.update(value["hook"])
            return value.get("current_task")
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"[:512]
            if kind == "action": self.last_action.update(status="failed", message=self.error)
            if kind == "save": self.save_hook.update(status="failed", message=self.error)
            return None

    def close(self):
        self.executor.shutdown(wait=True, cancel_futures=True)
        self.poll()
