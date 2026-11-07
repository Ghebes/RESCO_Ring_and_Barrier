# Plan: Generalizing Ring-and-Barrier to Six Maps

Target maps: **cologne1, grid4x4, saltlake2_400sX200w, saltlake2_stateXuniversity, arterial4x4, cologne3**

## 1. Key idea

The current `PHASE_MAP` in `resco_benchmark/mdp_options/actions.py` is a fixed ring layout written as Cologne1
phase numbers. Every RESCO map already lists its phases as movement pairs (`phase_pairs` in
`resco_benchmark/config/signal.yaml`) and, where needed, says which local phase each pair is (`pair_to_act_map`).
So the ring layout can be written **once, in movement terms**, and each signal's phase numbers can be derived from it
automatically.

### Ring layout (same for every map)

This is the layout the existing Cologne1 `PHASE_MAP` already uses.

```
                 Barrier 0 (E/W)            ‖        Barrier 1 (N/S)
  Ring 1:   E-N left  →  W-W through        ‖   S-E left  →  N-N through
  Ring 2:   W-S left  →  E-E through        ‖   N-W left  →  S-S through
                                            ‖
            rings cross the barrier together (both must be at their last phase)
```

Each state `(barrier, ring1_pos, ring2_pos)` (0 = left, 1 = through) is a pair of movements, for example
`(0,1,0)` = {W-W, W-S}. To get a signal's phase number:

1. Find the `phase_pairs` entry with exactly those two movements.
2. Convert it to the signal's own phase number through `pair_to_act_map[signal]`. Where that is `null`, the number
   is used as-is.

If no pair matches, that state doesn't exist at that signal.

## 2. What ring-and-barrier looks like on each map

### How to read the table

**State `(b, r1, r2)`** = (barrier, ring 1 position, ring 2 position).
- barrier: 0 = E/W, 1 = N/S
- ring position: 0 = left-turn phase, 1 = through phase

For example, `(0,1,0)` means "E/W barrier, ring 1 on through, ring 2 on left", which is the movements W-W + W-S.

**The numbers are phase indices.** Each one says which of the signal's own green phases the controller turns on in
that state. RESCO reads each signal's program from the `.net.xml` file, drops the yellow phases and any all-red phase,
and numbers the green phases that remain 0, 1, 2, … in file order (`resco_benchmark/traffic_signal.py`, the
`green_phases` loop in `__init__`). Action `k` tells SUMO to switch to green phase `k`.

Example, Cologne1:

| Index | SUMO state | Serves |
|---|---|---|
| 0 | `srrrrrrrrrGGGGGrrrrr` | W-W + W-S (one E/W approach, through and left) |
| 1 | `GGGGGrrrrrsrrrrrrrrr` | E-E + E-N (the other E/W approach) |
| 2 | `GGGggrrrrrGGGggrrrrr` | both E/W throughs |
| 3 | `rrrGGrrrrrrrrGGrrrrr` | both E/W lefts |
| 4–7 | … | the same four patterns for N/S |

So state `(0,1,0)` (W-W + W-S) is phase **0** in Cologne1.

**The same state has different numbers on different maps** because each network lists its phases in a different
order. "Both E/W lefts" is phase 3 in Cologne1, 5 in grid4x4 and 1 in arterial4x4. The ring logic is the same
everywhere; only the lookup differs.

**"—"** means the signal has no green phase for that combination, so the controller skips that state. For example,
arterial4x4 has no N/S protected-left phase, and cologne3 has no one-left/one-through phases.

**cologne3 `360082`:** that intersection has no south approach. RESCO labels its phase 2 as the "N/S lefts" pair.
The static check confirmed it: phase 2 gives a protected green to the northbound left (N-W), and nothing conflicts.

### Combined state → phase table

"—" means the state does not exist on that signal.

| State `(b,r1,r2)` | Movements | cologne1 | saltlake2 (both maps, A3 & B3) | grid4x4 (all 16) | arterial4x4 (all 16) | cologne3 `cluster_2415…` & `360086` | cologne3 `360082` |
|---|---|---|---|---|---|---|---|
| (0,0,0) both lefts      | W-S + E-N | 3 | 3 | 5 | 1 | 1 | 1 |
| (0,1,0) R1 thru, R2 left | W-W + W-S | 1 | 0 | 7 | 3 | — | — |
| (0,0,1) R1 left, R2 thru | E-E + E-N | 0 | 1 | 6 | 4 | — | — |
| (0,1,1) both through    | W-W + E-E | 2 | 2 | 4 | 2 | 0 | 0 |
| (1,0,0) both lefts      | S-E + N-W | 7 | 7 | 1 | — | 3 | 2 |
| (1,1,0) R1 thru, R2 left | N-N + N-W | 4 | 4 | 3 | — | — | — |
| (1,0,1) R1 left, R2 thru | S-S + S-E | 5 | 5 | 2 | — | — | — |
| (1,1,1) both through    | S-S + N-N | 6 | 6 | 0 | 0 | 2 | — |

### Per-map notes

**cologne1**: 1 signal (`cluster_357187_359543`), full 8-phase.
- **Labelling fix (2026-11-07).** Upstream RESCO's `signal.yaml` listed Cologne1's phases 0 and 1 as westbound-only and
  eastbound-only, but the network has them the other way round. Phase 0 greens lanes `28198821#3_*`, which `lane_sets`
  itself labels `E-E`/`E-N`. `show_phase_check.py` demonstrates this.
- `phase_pairs[0]` and `[1]` are now swapped, so `phase_pairs` agrees with `lane_sets` and the network.
- Effect on the original hard-coded `PHASE_MAP`: only states `(0,1,0)` and `(0,0,1)` swap (now phases 1 and 0).
  Earlier Cologne1 ring-and-barrier runs were valid ring-and-barrier control, but at the E/W barrier actions 1 and 2
  advanced the opposite ring from the documentation. Retrain Cologne1 so its results use the same ring meaning as
  the other maps.
- Other controllers affected on Cologne1 (anything that reads `phase_pairs`): MaxPressure and MaxWave, including
  when used as `uncontrolled_policy`; MPLight; CoSLight; and the `phase_queue` reward. Before the fix they sent
  westbound demand to the eastbound phase and vice versa.
- Not affected: IDQN and IPPO with `action_set: Phase`, FIXED and STOCHASTIC. These use phase indices, lane
  observations and wait rewards only.

**saltlake2_400sX200w**: 2 signals (A3, B3), full 8-phase. The `phase_pairs` order is the same as Cologne1's, so it
gets the same map. Each signal's first phase is all-red; `traffic_signal.py` already filters it out.

**saltlake2_stateXuniversity**: 2 signals (A3, B3), full 8-phase. The split combinations are the network's
"Westbound/Eastbound/Northbound/Southbound only" phases. Quirks (they don't block ring-and-barrier):
- **Network conflict at A3: fixed 2026-11-07.** In upstream RESCO, the "E/W protected lefts" phase (green phase 3)
  also gave westbound through lane `B3A3_2` (link 6) a protected green. That link conflicts with the eastbound left
  (link 14).
  - `check_saltlake_a3_collisions.py` measured 8 collisions in 900 s with the original phase, and 0 after the fix.
  - The fix: in the `.net.xml`, link 6 is now red in that phase (`srrrrrrsrrrrrrG`).
  - Every controller on this map, baselines included, must be re-run against the fixed network.
- `check_conflict_collisions.py` scans every phase on a map for simultaneous protected greens that SUMO lists as foes,
  and drives traffic through each pair. All six maps are clean.
  - saltlake2_400sX200w A3 has 8 pairs of protected-green links that SUMO lists as foes (its geometry is tight), but
    none collided in 600 s each, so that network is left unchanged.
- A3 has no westbound left turn (`'W-S': []`), so that ring 2 left-turn slot sees an empty queue.
- At A3, the south and north approaches share a lane between through and right-turn traffic.
- The map has lane closures built in (`closures`), so it is a disruption scenario. Mention this when comparing results.

**grid4x4**: 16 signals (A0–D3), full 8-phase, with a dedicated lane for every movement. Every signal uses the same
map. The static check (section 3, step 6) must confirm that the phase order in the network is the same at all 16.

**arterial4x4**: 16 signals (nt1–nt16), 5 green phases each.
- Barrier 0 (E/W) has all four ring combinations.
- Barrier 1 (N/S) has **only both-through** (phase 0, which includes permissive N/S lefts, the `g` links).
- Ring-and-barrier result: the N/S left phases are left out. The rings enter barrier 1 directly at through and
  cross back once it is served:
  ```
  E/W: lefts → (split) → both through  ‖  N/S: both through  ‖  back to E/W
  ```

**cologne3**: 3 signals. No signal has the split (one-left/one-through) combinations, so on these signals the rings
always move together (lead-lead only).
- `cluster_2415878664_254486231_359566_359576` and `360086`: both barriers have both-lefts → both-through.
  At `360086` the N and S approaches are single shared lanes, so a protected N/S left also holds through traffic
  queued behind it.
- `360082`: a T-intersection with no south approach. Barrier 0 has both-lefts → both-through. Barrier 1 has a
  **single state**: phase 2, which serves the whole north approach. `pair_to_act_map` labels it as the N/S left pair.
  ⚠ Verify this with the static check. That state is both the entry and the terminal state of barrier 1.
- In the through phases, lefts are permissive (`g`).

## 3. Implementation steps

### Step 1: Generalize `RingAndBarrier` (`resco_benchmark/mdp_options/actions.py`)
- Remove `SIGNAL_ID`, the `cfg.map != "cologne1"` check and `EXPECTED_PHASE_COUNT`.
- Add `RING_LAYOUT` (the movement table in section 1). Allow an optional override per map in `signal.yaml` under a
  `ring_barrier:` key.
- In `__init__`, build `self.phase_maps[signal_id]` (state → local phase) and
  `self.available[signal_id][barrier]` (the set of `(r1, r2)` states each barrier has).
- Raise an error if a barrier has no states at all, or if two states resolve to the same local phase.
- Keep `num_acts = 4` on every signal.

### Step 2: Change the stepping rules to handle missing states
- **Entry state.** When a barrier starts, the rings go to the first state that exists in the order `(0,0)`,
  `(0,1)`/`(1,0)`, `(1,1)`. On arterial4x4's N/S barrier, that is both-through.
- **Terminal state.** Each barrier's terminal state is `(1,1)` if it exists, otherwise the last state it has.
  `ADVANCE_BOTH` from the terminal state crosses the barrier. This replaces the hard-coded `positions == [1,1]`
  check, so `360082`'s single-state barrier still cycles.
- **Advance into a missing state.** On barriers that only have both-left and both-through (cologne3), a single-ring
  advance has nowhere to go and becomes `HOLD`. Option: turn it into `ADVANCE_BOTH` instead. Recommended: `HOLD`,
  because it never moves the signal on faster than the agent asked.
- **Safety and green times.** `_safe_action` and `ring_through_min_green` use the terminal state instead of `[1,1]`.
  The max-green force-off only chooses actions that lead to states that exist.
- **Reset.** `reset()` uses barrier 0's entry state. `last_phases` is set from the derived map.

### Step 3: Agent compatibility
- Shared agents that treat an action as a `phase_pairs` index break with 4 ring-and-barrier actions. These are
  SharedDQN/MPLight with `pair_to_act_map` / `reverse_valid`, and CoSLight.
- Support IDQN and IPPO (independent per-signal agents) first. Make other combinations fail with a clear error.

### Step 4: Observation (recommended)
On cologne3, two states can now map to the same phase. Add the barrier, the two ring positions and the time in the
current phase to the agent's state, either through a small hook in `resco_benchmark/mdp_options/states.py` or a
wrapper.

### Step 5: Config (`resco_benchmark/config/config.yaml`)
- Add `ring_max_green` and `ring_through_max_green` overrides per map in the map profiles. grid4x4 and arterial4x4
  need shorter values than Cologne1's 120 s.
- Change the Cologne1-specific `action_set` comment.

### Step 6: Verification (new `test_ring_barrier.py`)
1. **Static check without SUMO.** For every signal on the 6 maps:
   - Build the phase maps.
   - Parse the network's `<connection>` link indices.
   - Check that each mapped phase gives green (`G`/`g`) to both of its movements and red to the opposing through
     movements.
   - This catches a `pair_to_act_map` that disagrees with the phase order in the network file (see the ⚠ on
     cologne3 `360082`).
2. **Cologne1 regression.** The derived map equals the existing `PHASE_MAP`.
3. **Random-action test.** Run 10k random actions per signal and check:
   - every output phase exists;
   - barriers alternate;
   - no barrier is skipped;
   - no state lasts longer than max green.
4. **Smoke runs.** Run 2–3 short IDQN episodes per map with SUMO.

### Step 7 (optional): NEMA actuated baseline for every map
Generalize `run_cologne1_nema.py` to `run_nema.py --map X`, which generates `<map>_nema.add.xml` from the same ring
layout and link-index mapping.
- grid4x4 and arterial4x4 have 16 signals each.
- Phases that don't exist (arterial N/S lefts, cologne3 split phases) are left out of the NEMA rings.
- Salt Lake also needs its own begin/end times and flow file.

## 4. Order of work
1. Steps 1–2, plus the static check, on grid4x4 and both saltlake2 maps (full 8-phase).
2. arterial4x4 (N/S left phases left out).
3. cologne3 (rings move together; single-state barrier).
4. Steps 3–5.
5. Smoke runs on all six maps.
6. NEMA baseline (step 7) last.

## 5. Findings during implementation (2026-11-07)

### grid4x4 / arterial4x4 route files
- Each map's 1,400 per-episode route files are now extracted from `<map>.zip` into
  `resco_benchmark/environments/<map>/<map>/`. These folders are git-ignored (about 0.9 GB).
- `multi_signal.py` now loads `<map>_<n>.rou.xml` for episode n. Upstream RESCO v2 passed the folder itself to
  SUMO, so these maps could not run.
- After 1,400 episodes the files cycle, so testing episodes 1401–1500 reuse files 1–100.
- Flow scaling (`flow` ≠ 1.0) is not supported for these maps.
- Smoke runs (2 episodes each) pass on both maps.

### Junction collisions
SUMO only detects collisions inside junctions when `collision.check-junctions` is on. RESCO leaves it off.
`check_network_collisions.py` turns it on and runs traffic on every signalised movement for 1 h.
- **stateXuniversity A3, protected-vs-protected conflict:** a real network error, now fixed (see section 2).
- **Collisions remain on every map, involving left turns:** permissive left vs opposing through, or left vs left
  around a phase change.
  - They also happen under the network's own fixed-time program with no RESCO control. Cologne1 had 44–62 colliding
    pairs per hour that way, versus 38 under random ring-and-barrier.
  - So they come from how SUMO models left turns through the junction, not from the ring-and-barrier controller or
    from A3.
  - A 5 s all-red after every change cuts them only partly, and causes heavy teleporting and throughput loss.
  - Published RESCO results were produced with junction collision checking off, as this repo still is.
