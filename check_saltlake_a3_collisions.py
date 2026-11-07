#!/usr/bin/env python3
"""Does saltlake2_stateXuniversity A3's "E/W protected lefts" phase cause collisions?

That phase (green phase 3, state srrrrrGsrrrrrrG) gives protected green to the
eastbound left (link 14) AND westbound through lane B3A3_2 (link 6), which
SUMO's junction model lists as foes.  This script sends traffic on exactly
those two movements and holds the signal on one phase, then counts what SUMO
reports:

  * collisions        (junction collision checking switched on)
  * emergency braking (vehicles braking harder than they physically should)
  * teleports         (vehicles stuck long enough for SUMO to remove them)
  * throughput        (vehicles that made it through on each movement)

The same traffic is run against a control phase with no conflict.

Usage:
    python check_saltlake_a3_collisions.py            # headless, prints results
    python check_saltlake_a3_collisions.py --gui      # watch it in sumo-gui
"""

from __future__ import annotations

import argparse
import tempfile
from pathlib import Path

import traci

ROOT = Path(__file__).resolve().parent
NET = ROOT / "resco_benchmark" / "environments" / "saltlake2_stateXuniversity" / "saltlake2_stateXuniversity.net.xml"
SIGNAL = "A3"

# Eastbound left from the left-turn lane, westbound through from lane 2 only.
ROUTES = """<routes>
    <vType id="car" lcStrategic="0" lcSpeedGain="0" lcKeepRight="0"/>
    <route id="eb_left" edges="left3A3 A3top0"/>
    <route id="wb_thru" edges="B3A3 A3left3"/>
    <flow id="EB_left"  type="car" route="eb_left" begin="0" end="{end}" vehsPerHour="600" departLane="3" departSpeed="max"/>
    <flow id="WB_thru2" type="car" route="wb_thru" begin="0" end="{end}" vehsPerHour="600" departLane="2" departSpeed="max"/>
</routes>
"""

ORIGINAL_STATE = "srrrrrGsrrrrrrG"  # upstream RESCO network, before the fix


def network_phase_3() -> str:
    """'E/W protected lefts' as it is now in the .net.xml."""
    import xml.etree.ElementTree as ET
    for _, elem in ET.iterparse(NET, events=("end",)):
        if elem.tag == "tlLogic" and elem.get("id") == SIGNAL:
            for ph in elem.findall("phase"):
                if (ph.get("note") or "").startswith("East/West protected lefts"):
                    return ph.get("state")
    raise ValueError("phase not found")


PHASES = {
    "ORIGINAL upstream phase 3 (link 6 green)": ORIGINAL_STATE,
    "CURRENT network phase 3": network_phase_3(),
}


def run(label: str, state: str, seconds: int, gui: bool, delay: int) -> dict:
    with tempfile.TemporaryDirectory() as tmp:
        rou = Path(tmp) / "a3.rou.xml"
        rou.write_text(ROUTES.format(end=seconds), encoding="utf-8")
        cmd = [
            "sumo-gui" if gui else "sumo",
            "-n", str(NET), "-r", str(rou),
            "--collision.check-junctions", "true",
            "--collision.action", "warn",   # keep vehicles so every collision is counted
            "--time-to-teleport", "120",
            "--no-step-log", "true",
            "--no-warnings", "true",
        ]
        if gui:
            cmd += ["--start", "--delay", str(delay)]
        traci.start(cmd, label=label)
        try:
            if gui:
                x, y = traci.junction.getPosition(SIGNAL)
                traci.gui.setOffset("View #0", x, y)
                traci.gui.setZoom("View #0", 1500)
            collisions, colliding_pairs, ebrake, teleports = 0, set(), 0, 0
            seen_vehicle_pairs = set()
            arrived = {"EB_left": 0, "WB_thru2": 0}
            for step in range(seconds + 120):
                traci.trafficlight.setRedYellowGreenState(SIGNAL, state)
                traci.simulationStep()
                for c in traci.simulation.getCollisions():
                    # SUMO reports a collision every step the vehicles overlap;
                    # count each vehicle pair once.
                    pair = frozenset((c.collider, c.victim))
                    if pair in seen_vehicle_pairs:
                        continue
                    seen_vehicle_pairs.add(pair)
                    collisions += 1
                    colliding_pairs.add(tuple(sorted(v.split(".")[0] for v in pair)))
                    if collisions <= 5:
                        print(f"    t={step:4}s collision: {c.collider} hit {c.victim} ({c.type}, lane {c.lane})")
                ebrake += traci.simulation.getEmergencyStoppingVehiclesNumber()
                teleports += traci.simulation.getStartingTeleportNumber()
                for vid in traci.simulation.getArrivedIDList():
                    arrived[vid.split(".")[0]] += 1
        finally:
            traci.close()
    return {
        "collisions": collisions,
        "movements involved": sorted(colliding_pairs),
        "emergency stops": ebrake,
        "teleports": teleports,
        "arrived": arrived,
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--seconds", type=int, default=900)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--delay", type=int, default=50, help="sumo-gui ms per step")
    args = p.parse_args()

    print(f"Network: {NET.name}, signal {SIGNAL}")
    print("Traffic: 600 veh/h eastbound left (lane left3A3_3) + 600 veh/h westbound through (lane B3A3_2)")
    print(f"Signal held on one phase for {args.seconds}s; junction collision checking ON\n")
    for label, state in PHASES.items():
        print(f"--- {label}: {state}")
        print(f"    link 6 (WB through lane 2) = {state[6]!r}, link 14 (EB left) = {state[14]!r}")
        res = run(label, state, args.seconds, args.gui, args.delay)
        for k, v in res.items():
            print(f"    {k:18}: {v}")
        print()


if __name__ == "__main__":
    main()
