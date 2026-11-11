#!/usr/bin/env python3
"""saltlake2_400sX200w experiment grid, 3 jobs at a time.

The 10 rl_rb80 training runs are already done.  Their original evaluation was invalid
(see SALTLAKE_EVALUATION_FIX.md), so this script runs:
  rl_rb80_eval     reload each trained R&B model, run the 100 evaluation episodes again
  rl_phase         full training + evaluation (the first attempt crashed)
  stochastic_rb80  100 evaluation episodes
  actuated_rb      100 evaluation episodes
All evaluations use the same calendar hours and start from an empty network.
Results go to experiments_2026-11-07/saltlake2_400sX200w/.

    python run_experiments_saltlake2_400sX200w.py --dry-run
    python run_experiments_saltlake2_400sX200w.py
Re-running the same command resumes: finished jobs are skipped.
"""

from run_experiments import main

if __name__ == "__main__":
    main(default_maps=["saltlake2_400sX200w"], default_jobs=3, default_out="experiments_2026-11-07",
         default_types=["rl_rb80_eval", "rl_phase", "stochastic_rb80", "actuated_rb"])
