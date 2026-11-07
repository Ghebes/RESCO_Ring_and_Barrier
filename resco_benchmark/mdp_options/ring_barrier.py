"""Map-independent dual-ring, two-barrier logic.

This module has no dependency on the RESCO config so it can be unit tested
directly.  ``resco_benchmark.mdp_options.actions.RingAndBarrier`` wraps it.

The ring layout is written once in RESCO movement labels (``'<travel
direction>-<exit direction>'``, e.g. ``'W-S'`` is the westbound left turn).
A ring-and-barrier state ``(barrier, ring1_pos, ring2_pos)`` is the pair of
movements served by the two rings; position 0 is the protected left and
position 1 is the through phase.  Each signal's local green-phase index for a
state is found by matching that movement pair against the map's
``phase_pairs`` and translating through ``pair_to_act_map`` when present.

States with no matching phase do not exist at that signal.  Two cases occur
in the RESCO maps:

* omitted phases (arterial4x4 N/S has no protected lefts): the barrier is
  entered directly at the first state that exists;
* missing split combinations (cologne3 only has both-left and both-through):
  the rings move together and a single-ring advance becomes a hold.

FHWA Traffic Signal Timing Manual, section 4.2.2:
https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter4.htm
"""

from __future__ import annotations

from typing import Mapping, Optional, Sequence

# barrier -> ring -> (left movement, through movement)
RING_LAYOUT = {
    0: {"ring1": ("E-N", "W-W"), "ring2": ("W-S", "E-E")},  # east-west
    1: {"ring1": ("S-E", "N-N"), "ring2": ("N-W", "S-S")},  # north-south
}

BARRIERS = (0, 1)
# Order in which a barrier is entered: lefts first, then splits, then through.
ENTRY_ORDER = ((0, 0), (0, 1), (1, 0), (1, 1))
TERMINAL_ORDER = tuple(reversed(ENTRY_ORDER))

HOLD_BOTH = 0
ADVANCE_RING_1 = 1
ADVANCE_RING_2 = 2
ADVANCE_BOTH = 3
NUM_ACTS = 4


def state_movements(barrier: int, r1: int, r2: int, layout=RING_LAYOUT) -> frozenset:
    """Movements served by ring-and-barrier state ``(barrier, r1, r2)``."""
    return frozenset((layout[barrier]["ring1"][r1], layout[barrier]["ring2"][r2]))


def _lookup_local(pair_to_act: Optional[Mapping], pair_idx: int) -> Optional[int]:
    if pair_to_act is None:
        return pair_idx
    for key in (pair_idx, str(pair_idx)):
        if key in pair_to_act:
            return int(pair_to_act[key])
    return None


def build_phase_map(
    phase_pairs: Sequence[Sequence[str]],
    pair_to_act: Optional[Mapping] = None,
    num_green: Optional[int] = None,
    layout=RING_LAYOUT,
    signal_id: str = "?",
) -> dict:
    """Return ``{(barrier, r1, r2): local_green_phase_index}`` for one signal.

    Raises ``ValueError`` if a barrier has no states, two states share a
    phase, or a phase index is outside the signal's green phases.
    """
    lookup = {}
    for idx, pair in enumerate(phase_pairs):
        lookup.setdefault(frozenset(pair), idx)

    phase_map = {}
    for barrier in BARRIERS:
        for r1, r2 in ENTRY_ORDER:
            pair_idx = lookup.get(state_movements(barrier, r1, r2, layout))
            if pair_idx is None:
                continue
            local = _lookup_local(pair_to_act, pair_idx)
            if local is None:
                continue
            phase_map[(barrier, r1, r2)] = local

    for barrier in BARRIERS:
        if not any(state[0] == barrier for state in phase_map):
            raise ValueError(
                f"Signal {signal_id!r}: barrier {barrier} has no ring-and-barrier "
                "state matching the map's phase_pairs"
            )
    locals_ = list(phase_map.values())
    if len(set(locals_)) != len(locals_):
        raise ValueError(
            f"Signal {signal_id!r}: two ring-and-barrier states share a phase: "
            f"{phase_map}"
        )
    if num_green is not None:
        bad = [p for p in locals_ if not 0 <= p < num_green]
        if bad:
            raise ValueError(
                f"Signal {signal_id!r}: phase indices {bad} outside the "
                f"{num_green} green phases"
            )
    return phase_map


class RingBarrierSignal:
    """Ring-and-barrier state machine for one signal.

    ``step(action, step_length)`` applies the safety rules, advances the rings
    and returns the local green phase to show.
    """

    def __init__(
        self,
        phase_map: Mapping,
        max_green: Optional[float] = None,
        through_min_green: Optional[float] = None,
        through_max_green: Optional[float] = None,
        signal_id: str = "?",
    ):
        self.signal_id = signal_id
        self.phase_map = dict(phase_map)
        self.available = {
            b: {(r1, r2) for (bb, r1, r2) in self.phase_map if bb == b}
            for b in BARRIERS
        }
        self.entry = {
            b: next(s for s in ENTRY_ORDER if s in self.available[b]) for b in BARRIERS
        }
        self.terminal = {
            b: next(s for s in TERMINAL_ORDER if s in self.available[b])
            for b in BARRIERS
        }
        # True when this barrier can serve one ring's left with the other's through.
        self.has_splits = {
            b: bool(self.available[b] & {(0, 1), (1, 0)}) for b in BARRIERS
        }
        self.max_green = max_green
        self.through_min_green = through_min_green
        self.through_max_green = through_max_green
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.barrier = 0
        self.positions = self.entry[0]
        self.phase_elapsed = 0
        self.last_phase = self.phase

    @property
    def state(self) -> tuple:
        return (self.barrier, *self.positions)

    @property
    def phase(self) -> int:
        return self.phase_map[self.state]

    @property
    def at_terminal(self) -> bool:
        return self.positions == self.terminal[self.barrier]

    # ------------------------------------------------------------------ rules
    def safe_action(self, action: int) -> int:
        """Enforce terminal-phase min green and max-green force-offs."""
        if (
            self.at_terminal
            and self.through_min_green is not None
            and self.phase_elapsed < self.through_min_green
        ):
            return HOLD_BOTH
        max_green = self.through_max_green if self.at_terminal else self.max_green
        if max_green is None or self.phase_elapsed < max_green:
            return action
        # ADVANCE_BOTH always leaves the current state: it moves any ring still
        # on its left to through, or crosses the barrier from the terminal state.
        return ADVANCE_BOTH

    def advance(self, action: int) -> None:
        if action == HOLD_BOTH:
            return
        if action == ADVANCE_BOTH and self.at_terminal:
            self.barrier = 1 - self.barrier
            self.positions = self.entry[self.barrier]
            return

        r1, r2 = self.positions
        if action == ADVANCE_RING_1:
            target = (1, r2)
        elif action == ADVANCE_RING_2:
            target = (r1, 1)
        elif action == ADVANCE_BOTH:
            target = (1, 1)
        else:
            raise ValueError(f"Unknown ring-and-barrier action {action}")

        if target in self.available[self.barrier]:
            self.positions = target
        elif action == ADVANCE_BOTH:
            # (1, 1) is omitted at this barrier; finish at its terminal state.
            self.positions = self.terminal[self.barrier]
        # Otherwise a single-ring advance into a missing split combination is a
        # hold: the rings can only move together at this barrier.

    def step(self, action: int, step_length: float) -> int:
        action = self.safe_action(int(action))
        self.advance(action)
        phase = self.phase
        if phase == self.last_phase:
            self.phase_elapsed += step_length
        else:
            self.last_phase = phase
            self.phase_elapsed = 0
        return phase

    def features(self, elapsed_scale: float = 120.0) -> list:
        """Observation features: barrier, ring positions, scaled time in phase."""
        return [
            float(self.barrier),
            float(self.positions[0]),
            float(self.positions[1]),
            float(self.phase_elapsed) / elapsed_scale,
        ]
