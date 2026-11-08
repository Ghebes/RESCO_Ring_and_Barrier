#!/usr/bin/env python3
"""Cologne1-only experiment grid.

Same as run_experiments.py with --maps cologne1 as the default:
10 seeds x {rl_rb80, rl_phase, stochastic_rb80, actuated_rb} = 40 jobs,
4 running at a time.  Results go to experiments_<date>/cologne1/.

    python run_experiments_cologne1.py --dry-run
    python run_experiments_cologne1.py
    python run_experiments_cologne1.py --out experiments_2026-11-07   # resume on a later day
"""

from run_experiments import main

if __name__ == "__main__":
    main(default_maps=["cologne1"], default_jobs=4)
