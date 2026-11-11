# Salt Lake evaluation fix

**Applies to:** both Salt Lake maps (`saltlake2_400sX200w` and `saltlake2_stateXuniversity`). Cologne1 and grid4x4 are not affected.

## How Salt Lake episodes work

- Each episode is **one hour of real 2023 traffic**. Episode 1 is Jan 1 00:00–01:00, episode 2 is 01:00–02:00, and so on through the calendar.
- At the end of each hour RESCO **saves the whole network** (every vehicle still on the road) and loads it at the start of the next hour, so traffic carries over from hour to hour.

## Problem 1: controllers were evaluated on different traffic

RESCO does not reset the calendar or the saved network when evaluation begins.

| Controller | Trains on | Was evaluated on |
|---|---|---|
| RL (IDQN) | hours 0–1399 (Jan 1 – Feb 28) | **Feb 28 – Mar 4**: normal weekdays with rush hours, starting from the backlog training left behind |
| Actuated, stochastic | (no training) | **Jan 1 – Jan 5**: light holiday traffic, starting from an empty network |

So the first comparison (RL R&B about 1,770 s against actuated about 65 s) measured different traffic, not different controllers.

**Fix** (`MultiSignal.start_saltlake_test_window` in `resco_benchmark/multi_signal.py`):
- **Same hours for everyone:** when evaluation starts, every controller moves to the same calendar position, the hour right after the 1,400 training hours. RESCO's own calendar logic is replayed, including its skip over missing data. On 400s×200w that is **Feb 28 2023, hour 8 onward**. No controller has seen these hours in training.
- **Empty network for everyone:** the saved network is deleted, so every controller starts evaluation from an empty network.

## Problem 2: full-phase runs crashed

- **The cause:** the saved network file also holds SUMO's running total of travel time. At midnight the clock goes back from 86,400 s to 0. A vehicle that enters before midnight and leaves after it therefore adds a *negative* travel time.
- **Why only full phase:** full-phase training built huge gridlocks, with up to 70,000 vehicles in the saved network. With that many vehicles crossing midnight, the total went negative. SUMO's file check rejects negative values, so it refused to load its own file and quit. Every crash happened two episodes after a midnight, at episode 24·k + 2.

**Fix:** load the saved network with `--xml-validation never`.
- This only skips the file check. The simulation is unchanged.
- The negative total is never used: RESCO's time loss comes from each vehicle's `timeLoss` and `departDelay`.

## Problem 3: parallel jobs deleted each other's files

This is fixed in `resco_benchmark/utils/utils.py`.
- **The cause:** a finishing job deleted the saved-network and trip files of *every* run in the map folder, not just its own.
- **Effect on Salt Lake:** this crashed or quietly reset runs.
- **Effect on Cologne1:** it dropped 1–2 episodes from some runs. The Cologne1 results are still valid; see the earlier analysis.

## What was rerun for 400s×200w

| Run type | What was done |
|---|---|
| RL R&B (`rl_rb80`) | Training kept, because training was not affected. Each saved model is reloaded and evaluated again on the fixed window (`rl_rb80_eval`). The old 1,500-episode curves are in `_rl_rb80_training_only/`. Use them for training curves only, not for evaluation. |
| RL Phase | Rerun from scratch, since it crashed. |
| Actuated, stochastic | Rerun on the fixed window. |

`saltlake2_stateXuniversity` has not been run yet, so it gets all three fixes from the start.
