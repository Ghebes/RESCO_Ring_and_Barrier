#!/usr/bin/env python3
"""saltlake2_400sX200w-only experiment grid.

Same as run_experiments.py with --maps saltlake2_400sX200w as the default:
10 seeds x {rl_rb80, rl_phase, stochastic_rb80, actuated_rb} = 40 jobs,
3 running at a time.  Results go next to the Cologne1 results, in
experiments_2026-11-07/saltlake2_400sX200w/, so all maps stay in one experiment folder.

    python run_experiments_saltlake2_400sX200w.py --dry-run
    python run_experiments_saltlake2_400sX200w.py
Re-running the same command resumes: finished jobs are skipped.
"""

from run_experiments import main

if __name__ == "__main__":
    main(default_maps=["saltlake2_400sX200w"], default_jobs=3, default_out="experiments_2026-11-07")
