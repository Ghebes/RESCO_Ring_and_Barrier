#!/usr/bin/env python3
"""Checks for the generalized ring-and-barrier controller.

Runs without SUMO or torch:
    python -m unittest test_ring_barrier -v
    python test_ring_barrier.py --report      # print per-signal phase tables

1. Derived state -> phase maps match the tables in plan.md (and the original
   hard-coded Cologne1 PHASE_MAP).
2. Static check against each .net.xml: every mapped phase gives green to both
   of its movements' links, and no conflicting movement gets a protected green.
3. Random-action runs: only mapped phases are shown, barriers alternate only
   from their terminal state, rings never move backwards inside a barrier,
   every state is reachable and max green is respected.
"""

from __future__ import annotations

import random
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from resco_benchmark.mdp_options import ring_barrier as rb

ROOT = Path(__file__).resolve().parent
ENV_DIR = ROOT / "resco_benchmark" / "environments"
SIGNAL_YAML = ROOT / "resco_benchmark" / "config" / "signal.yaml"

MAPS = [
    "cologne1",
    "grid4x4",
    "saltlake2_400sX200w",
    "saltlake2_stateXuniversity",
    "arterial4x4",
    "cologne3",
]

FULL_EW_NS = {"000": 3, "010": 0, "001": 1, "011": 2, "100": 7, "110": 4, "101": 5, "111": 6}
# Cologne1's network lists eastbound-only before westbound-only (see signal.yaml note).
COLOGNE1 = {"000": 3, "010": 1, "001": 0, "011": 2, "100": 7, "110": 4, "101": 5, "111": 6}
GRID = {"000": 5, "010": 7, "001": 6, "011": 4, "100": 1, "110": 3, "101": 2, "111": 0}
ARTERIAL = {"000": 1, "010": 3, "001": 4, "011": 2, "111": 0}
COLOGNE3_4 = {"000": 1, "011": 0, "100": 3, "111": 2}

# Expected maps, as documented in plan.md.  None = same map on every signal.
EXPECTED = {
    "cologne1": COLOGNE1,
    "saltlake2_400sX200w": FULL_EW_NS,
    "saltlake2_stateXuniversity": FULL_EW_NS,
    "grid4x4": GRID,
    "arterial4x4": ARTERIAL,
    "cologne3": {
        "cluster_2415878664_254486231_359566_359576": COLOGNE3_4,
        "360086": COLOGNE3_4,
        "360082": {"000": 1, "011": 0, "100": 2},
    },
}

# Original hard-coded Cologne1 map from before generalization.  It came from the
# upstream phase_pairs order, which had phases 0 and 1 swapped.
ORIGINAL_COLOGNE1 = {
    (0, 0, 0): 3, (0, 1, 0): 0, (0, 0, 1): 1, (0, 1, 1): 2,
    (1, 0, 0): 7, (1, 1, 0): 4, (1, 0, 1): 5, (1, 1, 1): 6,
}

LEFT = {("S", "E"), ("W", "S"), ("N", "W"), ("E", "N")}
RIGHT = {("S", "W"), ("W", "N"), ("N", "E"), ("E", "S")}
TURN_DIRS = {"through": {"s"}, "left": {"l", "L"}, "right": {"r", "R"}}


# --------------------------------------------------------------------- loading
_signal_cfg = None


def signal_config() -> dict:
    global _signal_cfg
    if _signal_cfg is None:
        _signal_cfg = yaml.safe_load(SIGNAL_YAML.read_text(encoding="utf-8"))
    return _signal_cfg


_net_cache: dict = {}


def load_net(map_name: str):
    """Return ({tl_id: [green states]}, {tl_id: [(from_lane, dir, linkIndex)]})."""
    if map_name in _net_cache:
        return _net_cache[map_name]
    net = ENV_DIR / map_name / f"{map_name}.net.xml"
    greens, links = {}, {}
    for _, elem in ET.iterparse(net, events=("end",)):
        if elem.tag == "tlLogic":
            states = []
            for phase in elem.findall("phase"):
                st = phase.get("state")
                # Same filter as traffic_signal.Signal: no yellow, not all red/stop.
                if "y" not in st and st.count("r") + st.count("s") != len(st):
                    states.append(st)
            greens[elem.get("id")] = states
        elif elem.tag == "connection" and elem.get("tl") is not None:
            lane = f"{elem.get('from')}_{elem.get('fromLane')}"
            links.setdefault(elem.get("tl"), []).append(
                (lane, elem.get("dir"), int(elem.get("linkIndex")))
            )
        elif elem.tag in ("edge", "junction"):
            elem.clear()
    _net_cache[map_name] = (greens, links)
    return greens, links


def map_signals(map_name: str) -> list:
    greens, _ = load_net(map_name)
    mcfg = signal_config()[map_name]
    return [sid for sid in greens if sid in mcfg]


def phase_map_for(map_name: str, signal_id: str) -> dict:
    mcfg = signal_config()[map_name]
    greens, _ = load_net(map_name)
    p2a = mcfg.get("pair_to_act_map")
    p2a = None if p2a is None else p2a[signal_id]
    return rb.build_phase_map(
        mcfg["phase_pairs"], p2a, num_green=len(greens[signal_id]), signal_id=signal_id
    )


def turn_type(movement: str) -> str:
    a, b = movement.split("-")
    if a == b:
        return "through"
    if (a, b) in LEFT:
        return "left"
    return "right"


def movement_links(map_name: str, signal_id: str, movement: str) -> list:
    lanes = set(signal_config()[map_name][signal_id]["lane_sets"].get(movement, []))
    dirs = TURN_DIRS[turn_type(movement)]
    _, links = load_net(map_name)
    return sorted(
        {idx for lane, d, idx in links.get(signal_id, []) if lane in lanes and d in dirs}
    )


def as_str_map(phase_map: dict) -> dict:
    return {"".join(map(str, k)): v for k, v in phase_map.items()}


# ----------------------------------------------------------------------- tests
class DerivedMapTests(unittest.TestCase):
    def test_cologne1_differs_from_original_only_in_ew_splits(self):
        derived = phase_map_for("cologne1", "cluster_357187_359543")
        corrected = dict(ORIGINAL_COLOGNE1)
        corrected[(0, 1, 0)], corrected[(0, 0, 1)] = (
            ORIGINAL_COLOGNE1[(0, 0, 1)], ORIGINAL_COLOGNE1[(0, 1, 0)],
        )
        self.assertEqual(derived, corrected)

    def test_maps_match_plan_tables(self):
        for map_name in MAPS:
            for sid in map_signals(map_name):
                expected = EXPECTED[map_name]
                if map_name == "cologne3":
                    expected = expected[sid]
                with self.subTest(map=map_name, signal=sid):
                    self.assertEqual(as_str_map(phase_map_for(map_name, sid)), expected)

    def test_signal_counts(self):
        counts = {m: len(map_signals(m)) for m in MAPS}
        self.assertEqual(
            counts,
            {
                "cologne1": 1, "grid4x4": 16, "saltlake2_400sX200w": 2,
                "saltlake2_stateXuniversity": 2, "arterial4x4": 16, "cologne3": 3,
            },
        )

    def test_every_green_phase_is_used(self):
        greens = {m: load_net(m)[0] for m in MAPS}
        for map_name in MAPS:
            for sid in map_signals(map_name):
                with self.subTest(map=map_name, signal=sid):
                    used = set(phase_map_for(map_name, sid).values())
                    self.assertEqual(used, set(range(len(greens[map_name][sid]))))


# Problems in the upstream RESCO networks themselves, kept so the test stays
# honest: anything not listed here fails.  See check_saltlake_a3_collisions.py.
KNOWN_NETWORK_ISSUES: dict = {
    # saltlake2_stateXuniversity A3 used to give westbound through lane B3A3_2
    # (link 6) a protected green during 'E/W protected lefts', colliding with
    # the eastbound left.  Fixed in the .net.xml (2026-11-07); see
    # check_saltlake_a3_collisions.py.
}


class NetworkStaticTests(unittest.TestCase):
    """Check each mapped phase against the SUMO link states."""

    def check_signal(self, map_name, sid):
        greens, _ = load_net(map_name)
        layout = rb.RING_LAYOUT
        problems = []
        for (b, r1, r2), phase in phase_map_for(map_name, sid).items():
            state = greens[sid][phase]
            served = {layout[b]["ring1"][r1], layout[b]["ring2"][r2]}
            for mv in served:
                for idx in movement_links(map_name, sid, mv):
                    if state[idx] not in "Gg":
                        problems.append(f"{b}{r1}{r2} phase {phase}: {mv} link {idx} is {state[idx]!r}")
            # Unserved movements of this barrier's rings and every movement of
            # the other barrier must not hold a protected green.  Unserved
            # lefts may run permissively ('g') with the through phase.
            other = 1 - b
            unserved = [
                mv for ring in ("ring1", "ring2") for mv in layout[b][ring] if mv not in served
            ] + [mv for ring in ("ring1", "ring2") for mv in layout[other][ring]]
            for mv in unserved:
                permissive_ok = turn_type(mv) == "left" and mv in (
                    layout[b]["ring1"][0], layout[b]["ring2"][0]
                )
                for idx in movement_links(map_name, sid, mv):
                    bad = "G" if permissive_ok else "Gg"
                    if state[idx] in bad:
                        problems.append(
                            f"{b}{r1}{r2} phase {phase}: unserved {mv} link {idx} is {state[idx]!r}"
                        )
        return problems

    def test_phases_serve_their_movements(self):
        for map_name in MAPS:
            for sid in map_signals(map_name):
                with self.subTest(map=map_name, signal=sid):
                    problems = self.check_signal(map_name, sid)
                    known = KNOWN_NETWORK_ISSUES.get((map_name, sid), [])
                    self.assertEqual(sorted(problems), sorted(known))


class RandomActionTests(unittest.TestCase):
    STEPS = 10_000
    STEP = 5

    def run_signal(self, phase_map, max_green, through_max_green, seed):
        ctrl = rb.RingBarrierSignal(
            phase_map, max_green=max_green, through_max_green=through_max_green
        )
        rng = random.Random(seed)
        visited = {ctrl.state}
        crossings = 0
        prev_state = ctrl.state
        for _ in range(self.STEPS):
            phase = ctrl.step(rng.randrange(rb.NUM_ACTS), self.STEP)
            state = ctrl.state
            self.assertIn(state, phase_map)
            self.assertEqual(phase, phase_map[state])
            pb, p1, p2 = prev_state
            b, s1, s2 = state
            if b != pb:
                crossings += 1
                self.assertEqual((p1, p2), ctrl.terminal[pb], "crossed from non-terminal")
                self.assertEqual((s1, s2), ctrl.entry[b], "barrier entered mid-sequence")
            else:
                self.assertTrue(s1 >= p1 and s2 >= p2, f"ring moved backwards {prev_state}->{state}")
            limit = through_max_green if ctrl.at_terminal else max_green
            self.assertLessEqual(ctrl.phase_elapsed, limit)
            visited.add(state)
            prev_state = state
        self.assertEqual(visited, set(phase_map), "unreachable ring-and-barrier states")
        self.assertGreater(crossings, 10)

    def test_random_actions_all_signals(self):
        seen = set()
        for map_name in MAPS:
            for sid in map_signals(map_name):
                phase_map = phase_map_for(map_name, sid)
                key = tuple(sorted(phase_map.items()))
                if key in seen:
                    continue
                seen.add(key)
                with self.subTest(map=map_name, signal=sid):
                    self.run_signal(phase_map, 40, 60, seed=len(seen))

    def test_hold_forever_is_forced_off(self):
        for map_name in MAPS:
            sid = map_signals(map_name)[0]
            ctrl = rb.RingBarrierSignal(
                phase_map_for(map_name, sid), max_green=20, through_max_green=30
            )
            barriers = set()
            for _ in range(500):
                ctrl.step(rb.HOLD_BOTH, self.STEP)
                barriers.add(ctrl.barrier)
            with self.subTest(map=map_name):
                self.assertEqual(barriers, {0, 1})

    def test_cologne3_single_ring_advance_holds(self):
        ctrl = rb.RingBarrierSignal(phase_map_for("cologne3", "360086"))
        self.assertEqual(ctrl.state, (0, 0, 0))
        ctrl.step(rb.ADVANCE_RING_1, self.STEP)
        ctrl.step(rb.ADVANCE_RING_2, self.STEP)
        self.assertEqual(ctrl.state, (0, 0, 0))
        ctrl.step(rb.ADVANCE_BOTH, self.STEP)
        self.assertEqual(ctrl.state, (0, 1, 1))
        ctrl.step(rb.ADVANCE_BOTH, self.STEP)
        self.assertEqual(ctrl.state, (1, 0, 0))

    def test_arterial_ns_barrier_enters_at_through(self):
        ctrl = rb.RingBarrierSignal(phase_map_for("arterial4x4", "nt1"))
        ctrl.step(rb.ADVANCE_BOTH, self.STEP)  # E/W lefts -> both through
        ctrl.step(rb.ADVANCE_BOTH, self.STEP)  # cross to N/S
        self.assertEqual(ctrl.state, (1, 1, 1))
        ctrl.step(rb.ADVANCE_BOTH, self.STEP)  # cross back
        self.assertEqual(ctrl.state, (0, 0, 0))

    def test_cologne1_sequence_unchanged(self):
        """Replays the original controller's rules on a fixed action sequence."""
        ctrl = rb.RingBarrierSignal(ORIGINAL_COLOGNE1)
        seq = [
            (rb.ADVANCE_RING_1, (0, 1, 0)), (rb.ADVANCE_RING_1, (0, 1, 0)),
            (rb.ADVANCE_BOTH, (0, 1, 1)), (rb.HOLD_BOTH, (0, 1, 1)),
            (rb.ADVANCE_BOTH, (1, 0, 0)), (rb.ADVANCE_RING_2, (1, 0, 1)),
            (rb.ADVANCE_BOTH, (1, 1, 1)), (rb.ADVANCE_RING_1, (1, 1, 1)),
            (rb.ADVANCE_BOTH, (0, 0, 0)),
        ]
        for action, expected in seq:
            ctrl.step(action, self.STEP)
            self.assertEqual(ctrl.state, expected)


# ---------------------------------------------------------------------- report
def report() -> None:
    names = {(0, 0, 0): "both lefts", (0, 1, 0): "R1 thru + R2 left",
             (0, 0, 1): "R1 left + R2 thru", (0, 1, 1): "both through"}
    for map_name in MAPS:
        greens, _ = load_net(map_name)
        print(f"\n=== {map_name}")
        seen = {}
        for sid in map_signals(map_name):
            pm = as_str_map(phase_map_for(map_name, sid))
            seen.setdefault(tuple(sorted(pm.items())), []).append(sid)
        for key, sids in seen.items():
            sid = sids[0]
            print(f"  signals: {', '.join(sids)}")
            for state, phase in sorted(phase_map_for(map_name, sid).items()):
                b, r1, r2 = state
                mv = sorted(rb.state_movements(b, r1, r2))
                label = names[(0, r1, r2)]
                print(f"    {b}{r1}{r2} {'EW' if b == 0 else 'NS'} {label:18} "
                      f"{'+'.join(mv):10} -> phase {phase}: {greens[sid][phase]}")


if __name__ == "__main__":
    if "--report" in sys.argv:
        report()
    else:
        unittest.main()
