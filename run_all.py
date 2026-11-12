#!/usr/bin/env python3
"""Run the thesis grid from scratch on every environment except cologne1, with a
live progress display.

Environments (default) and run types, 10 seeds each = 160 jobs:
  grid4x4, saltlake2_400sX200w, saltlake2_stateXuniversity, cologne3
  -> rl_rb80, rl_phase, stochastic_rb80, actuated_rb
cologne1 is already finished in experiments_2026-11-07 and is left out (it can still
be added with --maps cologne1 ...).

Results go to a fresh folder (--out, default experiments_2026-11-11), so nothing from
earlier runs is reused or skipped.  Uses the same commands, output layout and .done
markers as run_experiments.py: re-running the same command resumes an interrupted run
(finished jobs are skipped; jobs that were mid-run start over).

The screen refreshes every --refresh seconds with:
  - totals (done / running / queued / failed)
  - every running job: stage (train/test), current episode / total, %, elapsed, ETA
  - the most recent START / DONE / FAIL events
All events are also appended to <out>/run_all_events.log.

Examples:
    python run_all.py --jobs 24 --dry-run         # list pending jobs, no running
    python run_all.py --jobs 24
    python run_all.py --jobs 40 --maps grid4x4 cologne3
    python run_all.py --jobs 24 --no-clear        # print updates instead of redrawing
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from run_experiments import (ALL_MAPS, DEFAULT_SEEDS, ROOT, build_command, detect_libsumo,
                             is_done, job_list, map_dirs)

STANDARD = ["rl_rb80", "rl_phase", "stochastic_rb80", "actuated_rb"]
MAP_TYPES = {
    "cologne1": STANDARD,
    "grid4x4": STANDARD,
    "saltlake2_400sX200w": STANDARD,
    "saltlake2_stateXuniversity": STANDARD,
    "cologne3": STANDARD,
}
assert list(MAP_TYPES) == ALL_MAPS
DEFAULT_MAPS = [m for m in ALL_MAPS if m != "cologne1"]  # cologne1 already finished
DEFAULT_OUT = "experiments_2026-11-11"  # fixed name, so resuming on a later day works

# What each run type writes into RESCO's config.json; used to find a job's console.log.
TYPE_SIGNATURE = {
    "rl_rb80": ("IDQN", "RingAndBarrier", True),
    "rl_phase": ("IDQN", "Phase", True),
    "stochastic_rb80": ("STOCHASTIC", "RingAndBarrier", False),
    "actuated_rb": ("ACTUATED", "Phase", False),
    "rl_rb80_eval": ("IDQN", "RingAndBarrier", False),
}
EPISODE_RE = re.compile(rb"Episode: (\d+),")


class State:
    """Shared between worker threads and the display thread."""

    def __init__(self, pending):
        self.lock = threading.Lock()
        self.queued = {j["name"] for j in pending}
        self.running = {}   # name -> dict(job, start, console, episode, total, training)
        self.done = []
        self.failed = []
        self.events = deque(maxlen=20)
        self.procs = {}     # name -> Popen
        self.stopping = False


def now() -> str:
    return f"{dt.datetime.now():%Y-%m-%d %H:%M:%S}"


def fmt_dur(sec: float | None) -> str:
    if sec is None:
        return "--"
    sec = int(sec)
    h, rem = divmod(sec, 3600)
    m, s = divmod(rem, 60)
    return f"{h}h{m:02d}m" if h else f"{m}m{s:02d}s"


def event(state: State, events_file: Path, msg: str, echo: bool) -> None:
    line = f"[{now()}] {msg}"
    with state.lock:
        state.events.append(line)
    with open(events_file, "a", encoding="utf-8") as fh:
        fh.write(line + "\n")
    if echo:
        print(line, flush=True)


def find_console_log(out: Path, job, started: float, claimed: set) -> Path | None:
    """console.log of the RESCO run this job started (config.json newer than job start)."""
    algo, action_set, training = TYPE_SIGNATURE[job["type"]]
    for cfg_path in (out / job["map"]).glob("*/git_FINAL/*/config.json"):
        try:
            if cfg_path.stat().st_mtime < started - 5 or cfg_path.parent in claimed:
                continue
            c = json.loads(cfg_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue  # being written right now; try again next refresh
        if (c.get("algorithm") == algo and c.get("action_set") == action_set
                and c.get("training") is training and c.get("seed") == job["seed"]):
            return cfg_path.parent / "console.log"
    return None


def last_episode(console: Path) -> int:
    """Last 'Episode: N,' in the log; only reads the tail, so it stays cheap."""
    try:
        with open(console, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(0, size - 65536))
            matches = EPISODE_RE.findall(fh.read())
        return int(matches[-1]) if matches else 0
    except OSError:
        return 0


def run_job(job, args, out: Path, state: State, events_file: Path) -> None:
    if state.stopping:
        return
    _, status_dir, logs_dir = map_dirs(out, job["map"])
    cmd, cwd = build_command(job, args)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    for var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
        env[var] = "1"  # one thread per job, so parallel jobs don't fight over cores
    training = TYPE_SIGNATURE[job["type"]][2]
    total = args.episodes + args.testing if training else args.testing
    log_path = logs_dir / f"{job['name']}.log"
    start = time.time()
    with state.lock:
        state.queued.discard(job["name"])
        state.running[job["name"]] = {"job": job, "start": start, "console": None,
                                      "episode": 0, "total": total, "training": training}
    event(state, events_file, f"START {job['name']}", args.no_clear)

    with open(log_path, "w", encoding="utf-8") as fh:
        fh.write(" ".join(cmd) + "\n\n")
        fh.flush()
        proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=fh, stderr=subprocess.STDOUT)
        with state.lock:
            state.procs[job["name"]] = proc
        rc = proc.wait()
    took = time.time() - start
    with state.lock:
        state.procs.pop(job["name"], None)
        state.running.pop(job["name"], None)

    if rc == 0:
        (status_dir / f"{job['name']}.done").write_text(
            json.dumps({"seconds": took, "finished": dt.datetime.now().isoformat()}), encoding="utf-8")
        with state.lock:
            state.done.append(job["name"])
        event(state, events_file, f"DONE  {job['name']} in {fmt_dur(took)}", args.no_clear)
    else:
        with state.lock:
            state.failed.append(job["name"])
        msg = "STOPPED" if state.stopping else f"FAIL  (exit {rc})"
        event(state, events_file, f"{msg} {job['name']} after {fmt_dur(took)}; log: {log_path}",
              args.no_clear)


def update_progress(state: State, out: Path) -> None:
    with state.lock:
        items = list(state.running.items())
        claimed = {r["console"].parent for _, r in items if r["console"]}
    for name, r in items:
        console = r["console"]
        if console is None:
            console = find_console_log(out, r["job"], r["start"], claimed)
            if console is not None:
                claimed.add(console.parent)
        ep = last_episode(console) if console else 0
        with state.lock:
            if name in state.running:
                state.running[name]["console"] = console
                state.running[name]["episode"] = ep


def render(state: State, args, n_total: int, started: float) -> str:
    with state.lock:
        running = sorted(state.running.values(), key=lambda r: r["start"])
        n_done, n_fail, n_queue = len(state.done), len(state.failed), len(state.queued)
        events = list(state.events)
    t = time.time()
    lines = [
        f"RESCO run_all  |  {now()}  |  --jobs {args.jobs}  |  elapsed {fmt_dur(t - started)}"
        f"  |  out: {args.out}",
        f"Jobs: {n_total} pending at start  ->  done {n_done}  running {len(running)}  "
        f"queued {n_queue}  failed {n_fail}",
        "",
        f"{'job':<58} {'stage':<6} {'episode':>11} {'%':>6} {'elapsed':>8} {'ETA':>8}",
        "-" * 102,
    ]
    for r in running:
        ep, total = min(r["episode"], r["total"]), r["total"]
        if r["training"]:
            stage = "train" if ep < args.episodes else "test"
        else:
            stage = "test"
        el = t - r["start"]
        eta = el / ep * (total - ep) if ep else None
        lines.append(f"{r['job']['name']:<58} {stage:<6} {f'{ep}/{total}':>11} "
                     f"{100 * ep / total:>5.1f}% {fmt_dur(el):>8} {fmt_dur(eta):>8}")
    if not running:
        lines.append("(nothing running)")
    lines += ["", "Recent events:"] + (events[-12:] or ["(none yet)"])
    return "\n".join(lines)


def display_loop(state: State, args, out: Path, n_total: int, started: float, stop: threading.Event):
    while True:
        update_progress(state, out)
        text = render(state, args, n_total, started)
        if args.no_clear:
            print("\n" + text + "\n", flush=True)
        else:
            sys.stdout.write("\x1b[2J\x1b[H" + text + "\n")
            sys.stdout.flush()
        if stop.wait(args.refresh):
            return


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--jobs", type=int, required=True, help="jobs to run in parallel")
    ap.add_argument("--maps", nargs="+", default=DEFAULT_MAPS, choices=ALL_MAPS)
    ap.add_argument("--seeds", nargs="+", type=int, default=DEFAULT_SEEDS)
    ap.add_argument("--episodes", type=int, default=1400, help="training episodes (RL)")
    ap.add_argument("--testing", type=int, default=100, help="evaluation episodes")
    ap.add_argument("--out", default=DEFAULT_OUT,
                    help=f"results folder, relative to the repo root (default {DEFAULT_OUT})")
    ap.add_argument("--refresh", type=float, default=30, help="seconds between screen updates")
    ap.add_argument("--no-clear", action="store_true",
                    help="print each update below the last instead of redrawing the screen")
    ap.add_argument("--python", default=sys.executable, help="Python with torch/pfrl/traci installed")
    ap.add_argument("--libsumo", choices=["auto", "True", "False"], default="auto")
    ap.add_argument("--dry-run", action="store_true", help="list pending jobs and exit")
    args = ap.parse_args()

    if args.libsumo == "auto":
        args.libsumo = str(detect_libsumo(args.python))
    os.system("")  # turns on ANSI escape codes in the Windows console

    out = ROOT / args.out
    jobs = []
    for m in args.maps:
        jobs += job_list([m], MAP_TYPES[m], args.seeds, args.episodes, args.testing)
    jobs.sort(key=lambda j: -j["est"])  # longest first
    pending = [j for j in jobs if not is_done(out, j)]

    print(f"Python : {args.python}  (libsumo: {args.libsumo})")
    print(f"Output : {out}")
    for m in args.maps:
        n_all = sum(j["map"] == m for j in jobs)
        n_pend = sum(j["map"] == m for j in pending)
        print(f"  {m:<28} {n_all - n_pend:>3} done, {n_pend:>3} to run   ({', '.join(MAP_TYPES[m])})")
    print(f"Total  : {len(pending)} jobs to run, {args.jobs} at a time")
    if args.dry_run:
        for j in pending:
            print("  " + j["name"])
        return
    if not pending:
        print("Nothing to do.")
        return

    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                            text=True).stdout.strip()
    for m in args.maps:
        map_dir, status_dir, logs_dir = map_dirs(out, m)
        status_dir.mkdir(parents=True, exist_ok=True)
        logs_dir.mkdir(parents=True, exist_ok=True)
        cfg_file = map_dir / ("experiment_config.json" if not (map_dir / "experiment_config.json").exists()
                              else f"experiment_config_{dt.datetime.now():%Y%m%dT%H%M%S}.json")
        cfg_file.write_text(json.dumps({
            "map": m, "types": MAP_TYPES[m], "seeds": args.seeds, "episodes": args.episodes,
            "testing": args.testing, "jobs": args.jobs, "libsumo": args.libsumo,
            "git_commit": commit, "started": dt.datetime.now().isoformat(), "runner": "run_all.py",
        }, indent=2), encoding="utf-8")

    events_file = out / "run_all_events.log"
    state = State(pending)
    started = time.time()
    stop = threading.Event()
    display = threading.Thread(target=display_loop, args=(state, args, out, len(pending), started, stop),
                               daemon=True)
    display.start()

    pool = ThreadPoolExecutor(max_workers=args.jobs)
    futures = [pool.submit(run_job, j, args, out, state, events_file) for j in pending]
    try:
        while not all(f.done() for f in futures):
            time.sleep(1)
    except KeyboardInterrupt:
        state.stopping = True
        print("\nCtrl+C: stopping running jobs (finished jobs are kept; re-run to resume)...", flush=True)
        for f in futures:
            f.cancel()
        with state.lock:
            procs = list(state.procs.values())
        for p in procs:
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], capture_output=True) \
                if os.name == "nt" else p.kill()
    pool.shutdown(wait=True)
    for f in futures:
        if not f.cancelled() and f.exception():
            print(f"Runner error: {f.exception()!r}")
    stop.set()
    display.join()
    update_progress(state, out)
    print(render(state, args, len(pending), started))
    print(f"\nFinished. {len(state.done)} succeeded, {len(state.failed)} failed/stopped.")
    for name in state.failed:
        print(f"  FAILED: {name}  (log: {map_dirs(out, name.split('__')[0])[2] / (name + '.log')})")
    sys.exit(1 if state.failed or state.stopping else 0)


if __name__ == "__main__":
    main()
