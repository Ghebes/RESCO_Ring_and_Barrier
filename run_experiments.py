#!/usr/bin/env python3
"""Run the thesis experiment grid: maps x run types x seeds.

Run types (per map, per seed):
  rl_rb80          IDQN, RingAndBarrier, max green 80 s (left/split and through),
                   1400 training + 100 evaluation episodes
  rl_phase         IDQN, full Phase action set, 1400 training + 100 evaluation episodes
  stochastic_rb80  STOCHASTIC (random actions), RingAndBarrier, max green 80 s,
                   100 evaluation episodes (nothing to train)
  actuated_rb      ACTUATED: SUMO NEMA actuated ring-and-barrier, generated for every
                   signal of every map (resco_benchmark/utils/nema.py), with the tuned
                   Cologne1 parameters (agent.yaml), 100 evaluation episodes

Seeds: trial seed S drives RESCO's SUMO seeds.  Evaluation episodes use SUMO seeds
S..S+99 for every run type (RESCO restarts the seed counter for testing), so all
controllers see the same evaluation traffic.  Every run type goes through RESCO,
so traffic generation and metrics are identical.

Output layout (one folder per environment):
    <out>/<map>/                 RESCO results for that map (one folder + .json per run)
    <out>/<map>/_logs/<job>.log  console output of each job
    <out>/<map>/_status/         markers of finished jobs

Jobs are resumable: re-running the script with the same --out skips finished jobs.

Examples:
    python run_experiments.py --dry-run                     # list jobs + time estimate
    python run_experiments.py                               # everything, 4 in parallel
    python run_experiments.py --maps cologne1 --jobs 6
    python run_experiments.py --types rl_rb80 --seeds 90210 --episodes 5 --testing 2 \\
        --out experiments_smoke                             # quick test
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RESCO = ROOT / "resco_benchmark"

ALL_MAPS = [
    "cologne1",
    "grid4x4",
    "saltlake2_400sX200w",
    "saltlake2_stateXuniversity",
    "cologne3",
]
# arterial4x4 excluded: oversaturated demand (~1,150 s time loss even under fixed
# timing) swamps controller differences, and it would be ~60% of total compute.
RUN_TYPES = ["rl_rb80", "rl_phase", "stochastic_rb80", "actuated_rb"]
# Evaluation only: reload the model an rl_rb80 run already trained (same map, same seed,
# found in <out>/<map>/) and run the 100 evaluation episodes again.  Used for
# saltlake2_400sX200w, whose first evaluation was invalid (see SALTLAKE_EVALUATION_FIX.md).
EXTRA_TYPES = ["rl_rb80_eval"]
DEFAULT_SEEDS = [90210 + 10000 * i for i in range(10)]

# Rough wall-clock seconds per episode for ONE job on an idle machine (TraCI, no
# libsumo), measured with short runs.  Only used for the --dry-run estimate.
SEC_PER_EPISODE = {
    "cologne1": 5, "cologne3": 13, "grid4x4": 30, "arterial4x4": 110,
    "saltlake2_400sX200w": 17, "saltlake2_stateXuniversity": 17,  # measured on 400sX200w
}


def job_list(maps, types, seeds, episodes, testing):
    jobs = []
    for m in maps:
        for t in types:
            for s in seeds:
                if t in ("actuated_rb", "stochastic_rb80", "rl_rb80_eval"):
                    est = testing * SEC_PER_EPISODE[m]
                else:
                    est = (episodes + testing) * SEC_PER_EPISODE[m]
                jobs.append({"map": m, "type": t, "seed": s, "name": f"{m}__{t}__seed{s}", "est": est})
    # Longest first, so the slowest jobs don't all end up running at the end.
    jobs.sort(key=lambda j: -j["est"])
    return jobs


def resco_command(job, args):
    common = [
        f"episodes:{args.episodes}", f"testing:{args.testing}", f"seed:{job['seed']}",
        f"log_dir:{args.out}/{job['map']}", f"libsumo:{args.libsumo}", "save_console_log:True",
    ]
    m = f"@{job['map']}"
    if job["type"] == "rl_rb80":
        extra = ["@IDQN", "action_set:RingAndBarrier", "ring_max_green:80", "ring_through_max_green:80"]
    elif job["type"] == "rl_phase":
        extra = ["@IDQN", "action_set:Phase"]
    elif job["type"] == "stochastic_rb80":
        extra = ["@STOCHASTIC", "action_set:RingAndBarrier", "ring_max_green:80",
                 "ring_through_max_green:80", "training:False", "save_model:False"]
    elif job["type"] == "actuated_rb":
        extra = ["@ACTUATED", "action_set:Phase", "training:False", "save_model:False"]
    elif job["type"] == "rl_rb80_eval":
        model = find_trained_model(ROOT / args.out / job["map"], job["seed"], "RingAndBarrier")
        extra = ["@IDQN", "action_set:RingAndBarrier", "ring_max_green:80", "ring_through_max_green:80",
                 "training:False", "save_model:False", f"load_model:{model}"]
    else:
        raise ValueError(job["type"])
    return [args.python, "main.py", m, extra[0], *extra[1:], *common], RESCO


def find_trained_model(map_dir: Path, seed: int, action_set: str) -> str:
    """uuid folder of the IDQN run that trained (training:True) on this map/seed/action set."""
    for cfg_path in map_dir.glob("*/git_FINAL/*/config.json"):
        c = json.loads(cfg_path.read_text(encoding="utf-8"))
        if (c.get("algorithm") == "IDQN" and c.get("training") is True and c.get("seed") == seed
                and c.get("action_set") == action_set and any(cfg_path.parent.glob("agt_*.pt"))):
            return cfg_path.parent.name
    raise FileNotFoundError(f"No trained IDQN {action_set} model for seed {seed} in {map_dir}")


def build_command(job, args):
    return resco_command(job, args)


def detect_libsumo(python: str) -> bool:
    r = subprocess.run([python, "-c", "import libsumo"], capture_output=True)
    return r.returncode == 0


def fmt_hours(sec: float) -> str:
    return f"{sec / 3600:.1f} h"


_print_lock = threading.Lock()


def log(msg: str) -> None:
    with _print_lock:
        print(f"[{dt.datetime.now():%Y-%m-%d %H:%M:%S}] {msg}", flush=True)


def map_dirs(out: Path, map_name: str):
    """(map folder, status folder, logs folder) for one environment."""
    d = out / map_name
    return d, d / "_status", d / "_logs"


def is_done(out: Path, job) -> bool:
    return (map_dirs(out, job["map"])[1] / f"{job['name']}.done").exists()


def run_job(job, args, out):
    _, status_dir, logs_dir = map_dirs(out, job["map"])
    cmd, cwd = build_command(job, args)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    # One thread per job for torch/numpy, so parallel jobs don't fight over cores.
    for var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
        env[var] = "1"
    log_path = logs_dir / f"{job['name']}.log"
    start = time.time()
    log(f"START {job['name']}")
    with open(log_path, "w", encoding="utf-8") as fh:
        fh.write(" ".join(cmd) + "\n\n")
        fh.flush()
        rc = subprocess.run(cmd, cwd=cwd, env=env, stdout=fh, stderr=subprocess.STDOUT).returncode
    took = time.time() - start
    if rc == 0:
        (status_dir / f"{job['name']}.done").write_text(
            json.dumps({"seconds": took, "finished": dt.datetime.now().isoformat()}), encoding="utf-8")
        log(f"DONE  {job['name']} in {fmt_hours(took)}")
    else:
        log(f"FAIL  {job['name']} (exit {rc}) after {fmt_hours(took)}; see {log_path}")
    return job, rc, took


def main(default_maps=None, default_jobs=4, default_out=None, default_types=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--maps", nargs="+", default=default_maps or ALL_MAPS, choices=list(SEC_PER_EPISODE))
    ap.add_argument("--types", nargs="+", default=default_types or RUN_TYPES,
                    choices=RUN_TYPES + EXTRA_TYPES)
    ap.add_argument("--seeds", nargs="+", type=int, default=DEFAULT_SEEDS)
    ap.add_argument("--episodes", type=int, default=1400, help="training episodes (RL)")
    ap.add_argument("--testing", type=int, default=100, help="evaluation episodes")
    ap.add_argument("--jobs", type=int, default=default_jobs,
                    help=f"jobs to run in parallel (default {default_jobs})")
    ap.add_argument("--python", default=sys.executable, help="Python with torch/pfrl/traci installed")
    ap.add_argument("--libsumo", choices=["auto", "True", "False"], default="auto")
    ap.add_argument("--out", default=default_out or f"experiments_{dt.date.today():%Y-%m-%d}",
                    help="results folder, relative to the repo root")
    ap.add_argument("--dry-run", action="store_true", help="list jobs and estimate time only")
    args = ap.parse_args()

    if args.libsumo == "auto":
        args.libsumo = str(detect_libsumo(args.python))

    out = ROOT / args.out
    jobs = job_list(args.maps, args.types, args.seeds, args.episodes, args.testing)
    pending = [j for j in jobs if not is_done(out, j)]

    total = sum(j["est"] for j in pending)
    print(f"Python : {args.python}  (libsumo: {args.libsumo})")
    print(f"Output : {out}  (one subfolder per map: {', '.join(args.maps)})")
    print(f"Jobs   : {len(jobs)} total, {len(jobs) - len(pending)} already done, {len(pending)} to run")
    print(f"Estimate (rough, assumes each job runs as fast as alone): "
          f"{fmt_hours(total)} sequential, ~{fmt_hours(total / max(1, args.jobs))} with --jobs {args.jobs}")

    if args.dry_run:
        for j in pending:
            cmd, cwd = build_command(j, args)
            print(f"\n# {j['name']}  (~{fmt_hours(j['est'])})\ncd {cwd} && {' '.join(cmd)}")
        return

    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                            text=True).stdout.strip()
    for m in args.maps:
        map_dir, status_dir, logs_dir = map_dirs(out, m)
        status_dir.mkdir(parents=True, exist_ok=True)
        logs_dir.mkdir(parents=True, exist_ok=True)
        (map_dir / "experiment_config.json").write_text(json.dumps({
            "map": m, "types": args.types, "seeds": args.seeds, "episodes": args.episodes,
            "testing": args.testing, "jobs": args.jobs, "libsumo": args.libsumo,
            "git_commit": commit, "started": dt.datetime.now().isoformat(),
        }, indent=2), encoding="utf-8")

    failures = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run_job, j, args, out) for j in pending]
        for n, fut in enumerate(as_completed(futures), 1):
            job, rc, _ = fut.result()
            if rc != 0:
                failures.append(job["name"])
            log(f"progress {n}/{len(pending)}")
    print(f"\nFinished. {len(pending) - len(failures)} succeeded, {len(failures)} failed.")
    for name in failures:
        print(f"  FAILED: {name}  (log: {map_dirs(out, name.split('__')[0])[2] / (name + '.log')})")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
