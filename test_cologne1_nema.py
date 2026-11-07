#!/usr/bin/env python3
"""Tests for the isolated Cologne1 native NEMA baseline."""

from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from run_cologne1_nema import CONTROLLER, NETWORK, PARAM_DEFAULTS, run_once, write_controller


SIGNAL_ID = "cluster_357187_359543"
EXPECTED_COMBINATIONS = {
    frozenset((1, 5)): "srrGGsrrrrsrrGGsrrrr",
    frozenset((2, 5)): "srrrrsrrrrGGGGGsrrrr",
    frozenset((1, 6)): "GGGGGsrrrrsrrrrsrrrr",
    frozenset((2, 6)): "GGGggsrrrrGGGggsrrrr",
    frozenset((3, 7)): "srrrrsrrGGsrrrrsrrGG",
    frozenset((4, 7)): "srrrrsrrrrsrrrrGGGGG",
    frozenset((3, 8)): "srrrrGGGGGsrrrrsrrrr",
    frozenset((4, 8)): "srrrrGGGggsrrrrGGGgg",
}

EXISTING_GREEN_STATES = {
    "srrrrrrrrrGGGGGrrrrr",
    "GGGGGrrrrrsrrrrrrrrr",
    "GGGggrrrrrGGGggrrrrr",
    "rrrGGrrrrrrrrGGrrrrr",
    "rrrrrGGGGGrrrrrsrrrr",
    "rrrrrsrrrrrrrrrGGGGG",
    "rrrrrGGGggrrrrrGGGgg",
    "rrrrrrrrGGrrrrrrrrGG",
}


def combine(first: str, second: str) -> str:
    priority = {"r": 0, "s": 1, "g": 2, "G": 3}
    return "".join(max(chars, key=priority.get) for chars in zip(first, second))


class ControllerStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.logic = ET.parse(CONTROLLER).getroot().find("tlLogic")
        assert cls.logic is not None
        cls.phases = {int(p.get("name")): p for p in cls.logic.findall("phase")}

    def test_native_nema_controller_and_rings(self) -> None:
        self.assertEqual(self.logic.get("id"), SIGNAL_ID)
        self.assertEqual(self.logic.get("type"), "NEMA")
        params = {p.get("key"): p.get("value") for p in self.logic.findall("param")}
        self.assertEqual(params["ring1"], "1,2,3,4")
        self.assertEqual(params["ring2"], "5,6,7,8")
        self.assertEqual(params["barrierPhases"], "2,6")
        self.assertEqual(params["barrier2Phases"], "4,8")

    def test_all_phase_states_match_network_signal_count(self) -> None:
        connections = [
            c for c in ET.parse(NETWORK).getroot().findall("connection")
            if c.get("tl") == SIGNAL_ID
        ]
        signal_count = max(int(c.get("linkIndex")) for c in connections) + 1
        self.assertEqual(set(self.phases), set(range(1, 9)))
        for phase in self.phases.values():
            self.assertEqual(len(phase.get("state")), signal_count)

    def test_ring_combinations_reproduce_cologne1_green_movements(self) -> None:
        existing_active = {
            frozenset(i for i, color in enumerate(state) if color in "Gg")
            for state in EXISTING_GREEN_STATES
        }
        generated_active = set()
        for pair, expected in EXPECTED_COMBINATIONS.items():
            first, second = sorted(pair)
            actual = combine(self.phases[first].get("state"), self.phases[second].get("state"))
            self.assertEqual(actual, expected, f"NEMA combination {first}+{second}")
            generated_active.add(
                frozenset(i for i, color in enumerate(actual) if color in "Gg")
            )
        self.assertEqual(generated_active, existing_active)

    def test_parameterization_does_not_modify_source(self) -> None:
        original = CONTROLLER.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "controller.xml"
            params = dict(PARAM_DEFAULTS, vehicle_extension=3.0, left_max_green=30.0)
            write_controller(CONTROLLER, target, params)
            phases = ET.parse(target).getroot().find("tlLogic").findall("phase")
            self.assertTrue(all(p.get("vehext") == "3" for p in phases))
            self.assertTrue(all(p.get("maxDur") == "30" for p in phases if int(p.get("name")) % 2))
        self.assertEqual(CONTROLLER.read_bytes(), original)


class ControllerSmokeTests(unittest.TestCase):
    def test_sumo_loads_and_run_is_reproducible(self) -> None:
        first = run_once(1234, begin=25200, end=25320)
        second = run_once(1234, begin=25200, end=25320)
        self.assertEqual(first, second)
        self.assertGreater(first["arrived"], 0)
        self.assertGreaterEqual(first["mean_time_loss"], 0)


if __name__ == "__main__":
    unittest.main()
