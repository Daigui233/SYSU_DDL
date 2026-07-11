import os
import threading
import time


def parse_cpu_set(value):
    text = str(value or "").strip()
    if not text:
        return None
    cpus = set()
    for part in text.replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            cpus.update(range(int(start), int(end) + 1))
        else:
            cpus.add(int(part))
    return cpus or None


def apply_current_thread_affinity(cpu_set, log_func=None, label="thread"):
    cpus = parse_cpu_set(cpu_set)
    if not cpus or not hasattr(os, "sched_setaffinity"):
        return False
    try:
        os.sched_setaffinity(0, cpus)
        if log_func is not None:
            log_func(f"{label} CPU affinity: {sorted(cpus)}")
        return True
    except Exception as exc:
        if log_func is not None:
            log_func(f"{label} CPU affinity skipped: {exc}")
        return False


def _read_text(path):
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as handle:
            return handle.read()
    except Exception:
        return ""


def _process_name_matches(pid, names):
    names = {str(name).strip().lower() for name in names if str(name).strip()}
    if not names:
        return False, ""
    proc_dir = f"/proc/{pid}"
    comm = _read_text(os.path.join(proc_dir, "comm")).strip()
    cmdline = _read_text(os.path.join(proc_dir, "cmdline")).replace("\0", " ").strip()
    exe = ""
    try:
        exe = os.path.basename(os.readlink(os.path.join(proc_dir, "exe")))
    except Exception:
        pass
    haystack = [comm.lower(), exe.lower(), cmdline.lower()]
    for name in names:
        if any(name == item for item in haystack[:2]):
            return True, comm or exe or str(pid)
        if any(name in item for item in haystack):
            return True, comm or exe or str(pid)
    return False, ""


def apply_named_process_affinity(names, cpu_set, exclude_pids=None, log_func=None, label="external"):
    cpus = parse_cpu_set(cpu_set)
    if not cpus or not hasattr(os, "sched_setaffinity"):
        return []
    exclude = {int(pid) for pid in (exclude_pids or [])}
    changed = []
    proc_root = "/proc"
    for entry in os.listdir(proc_root):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in exclude:
            continue
        matched, matched_name = _process_name_matches(pid, names)
        if not matched:
            continue
        task_dir = os.path.join(proc_root, entry, "task")
        tids = []
        try:
            tids = [int(tid) for tid in os.listdir(task_dir) if tid.isdigit()]
        except Exception:
            tids = [pid]
        ok = False
        for tid in tids:
            try:
                os.sched_setaffinity(tid, cpus)
                ok = True
            except PermissionError:
                continue
            except ProcessLookupError:
                continue
            except Exception:
                continue
        if ok:
            changed.append((pid, matched_name))
    if changed and log_func is not None:
        preview = ", ".join(f"{name}:{pid}" for pid, name in changed[:8])
        extra = "" if len(changed) <= 8 else f" +{len(changed) - 8}"
        log_func(f"{label} CPU affinity -> {sorted(cpus)}: {preview}{extra}")
    return changed


class ProcessAffinityGuard:
    """Periodically keep non-control processes away from the reserved control cores."""

    def __init__(
        self,
        names,
        cpu_set,
        interval=2.0,
        exclude_pids=None,
        log_func=None,
        label="external",
    ):
        if isinstance(names, str):
            names = [item.strip() for item in names.split(",") if item.strip()]
        self.names = list(names or [])
        self.cpu_set = str(cpu_set or "").strip()
        self.interval = max(0.2, float(interval))
        self.exclude_pids = set(int(pid) for pid in (exclude_pids or []))
        self.log_func = log_func or (lambda _message: None)
        self.label = label
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is not None or not self.names or not self.cpu_set:
            return self
        self._thread = threading.Thread(target=self._run, name=f"{self.label}-affinity", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self):
        reported_once = False
        while not self._stop.is_set():
            changed = apply_named_process_affinity(
                self.names,
                self.cpu_set,
                exclude_pids=self.exclude_pids,
                log_func=self.log_func if not reported_once else None,
                label=self.label,
            )
            reported_once = reported_once or bool(changed)
            self._stop.wait(self.interval)
