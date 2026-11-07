#!/usr/bin/env python3
"""Cologne1-only experiment grid.

Same as run_experiments.py with --maps cologne1 as the default:
10 seeds x {rl_rb80, rl_phase, stochastic_rb80, actuated_rb}.

    python run_experiments_cologne1.py --dry-run
    python run_experiments_cologne1.py --jobs 6
"""

from run_experiments import main

if __name__ == "__main__":
    main(default_maps=["cologne1"])
