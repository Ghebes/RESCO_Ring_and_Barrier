#!/usr/bin/env python3
"""Stress-test a network for collisions under ring-and-barrier control.

Runs SUMO with junction collision checking ON (RESCO leaves it off) and:
  * traffic on EVERY signalised movement (one flow per link, exact lanes);
  * each signal driven by the ring-and-barrier controller with random
    actions every step_length seconds;
  * RESCO's own transition rules: yellow (3 s) on links losing green, then a
    2 s all-red clearance if the old phase had permissive ('g') links.

It reports collisions (unique vehicle pairs, with the links involved),
emergency braking, teleports and throughput, for each seed.

Usage:
    python check_network_collisions.py saltlake2_stateXuniversity
    python check_network_collisions.py saltlake2_stateXuniversity --net path/to/other.net.xml
    python check_network_collisions.py saltlake2_400sX200w --seeds 1 2 3 --vph 300
"""

from __future__ import annotations

import argparse
import random
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import traci
import yaml

from resco_benchmark.mdp_options import ring_barrier as rb

ROOT = Path(__file__).resolve().parent
ENV = ROOT / "resco_benchmark" / "environments"
SIGNAL_YAML = ROOT / "resco_benchmark" / "config" / "signal.yaml"

STEP_LENGTH = 5      # RESCO step_length
YELLOW = 3           # RESCO yellow_length
CLEARANCE = 2        # RESCO clearance_length (override with --clearance)
ALWAYS_CLEAR = False  # RESCO only clears after phases with 'g' (--always-clear)


def parse_net(net: Path):
    greens, links = {}, {}
    for _, e in ET.iterparse(net, events=("end",)):
        if e.tag == "tlLogic":
            greens[e.get("id")] = [
                p.get("state") for p in e.findall("phase")
                if "y" not in p.get("state")
                and p.get("state").count("r") + p.get("state").count("s") != len(p.get("state"))
            ]
        elif e.tag == "connection" and e.get("tl"):
            links.setdefault(e.get("tl"), []).append(
                (int(e.get("linkIndex")), e.get("from"), e.get("fromLane"), e.get("to"), e.get("dir"))
            )
        elif e.tag in ("edge", "junction"):
            e.clear()
    return greens, links


def yellow_state(cur: str, nxt: str) -> str | None:
    """Same rule as traffic_signal.create_yellows."""
    out = ["y" if c in "Gg" and n in "rs" else c for c, n in zip(cur, nxt)]
    return "".join(out) if "y" in out else None


class SignalSim:
    """Replays traffic_signal.Signal's switching for one intersection."""

    def __init__(self, sid, greens, ctrl):
        self.sid, self.greens, self.ctrl = sid, greens, ctrl
        self.current = ctrl.phase
        self.next = None
        self.t = 0
        self.clearance = 0
        traci.trafficlight.setRedYellowGreenState(sid, greens[self.current])

    def switch(self, new):
        if new == self.current:
            return
        self.t = 0
        y = yellow_state(self.greens[self.current], self.greens[new])
        if y is not None:
            traci.trafficlight.setRedYellowGreenState(self.sid, y)
            self.next = new
            if ALWAYS_CLEAR or "g" in self.greens[self.current]:
                self.clearance = CLEARANCE
        else:
            traci.trafficlight.setRedYellowGreenState(self.sid, self.greens[new])
            self.current, self.next = new, None

    def step(self):
        self.t += 1
        if self.next is not None and self.t >= YELLOW:
            if self.clearance:
                traci.trafficlight.setRedYellowGreenState(self.sid, "r" * len(self.greens[self.current]))
                self.clearance -= 1
            else:
                traci.trafficlight.setRedYellowGreenState(self.sid, self.greens[self.next])
                self.t, self.current, self.next = 0, self.next, None


def run(map_name, net, seed, seconds, vph, controller="rb"):
    greens, links = parse_net(net)
    mcfg = yaml.safe_load(SIGNAL_YAML.read_text(encoding="utf-8"))[map_name]
    sigs = [s for s in greens if s in mcfg]
    rng = random.Random(seed)

    flows, link_of_flow = [], {}
    for sid in sigs:
        seen = set()
        for idx, frm, lane, to, d in links[sid]:
            if d == "t" or (frm, lane, to) in seen:
                continue
            seen.add((frm, lane, to))
            fid = f"{sid}_L{idx}_{frm}_{lane}"
            link_of_flow[fid] = f"{sid} link {idx} ({d})"
            flows.append(
                f'  <flow id="{fid}" from="{frm}" to="{to}" begin="0" end="{seconds}" '
                f'vehsPerHour="{vph}" departLane="{lane}" departSpeed="max"/>'
            )
    routes = "<routes>\n" + "\n".join(flows) + "\n</routes>\n"

    with tempfile.TemporaryDirectory() as tmp:
        rou = Path(tmp) / "all.rou.xml"
        rou.write_text(routes, encoding="utf-8")
        traci.start([
            "sumo", "-n", str(net), "-r", str(rou), "--seed", str(seed),
            "--collision.check-junctions", "true", "--collision.action", "warn",
            "--time-to-teleport", "300", "--no-step-log", "true", "--no-warnings", "true",
        ], label=f"stress{seed}")
        try:
            p2a = mcfg.get("pair_to_act_map")
            sims = {}
            for sid in (sigs if controller != "static" else []):
                pm = rb.build_phase_map(mcfg["phase_pairs"], None if p2a is None else p2a[sid],
                                        num_green=len(greens[sid]), signal_id=sid)
                sims[sid] = SignalSim(sid, greens[sid], rb.RingBarrierSignal(
                    pm, max_green=120, through_max_green=120, signal_id=sid))
            pairs, kinds, ebrake, teleports, arrived = set(), Counter(), 0, 0, 0
            for t in range(seconds + 300):
                if t % STEP_LENGTH == 0:
                    for s in sims.values():
                        if controller == "phase":   # Phase action set: any green phase
                            s.switch(rng.randrange(len(s.greens)))
                        else:
                            s.switch(s.ctrl.step(rng.randrange(rb.NUM_ACTS), STEP_LENGTH))
                traci.simulationStep()
                for s in sims.values():
                    s.step()
                for c in traci.simulation.getCollisions():
                    key = frozenset((c.collider, c.victim))
                    if key not in pairs:
                        pairs.add(key)
                        a, b = (link_of_flow[v.rsplit(".", 1)[0]] for v in (c.collider, c.victim))
                        kinds[(a, b, c.type)] += 1
                ebrake += traci.simulation.getEmergencyStoppingVehiclesNumber()
                teleports += traci.simulation.getStartingTeleportNumber()
                arrived += traci.simulation.getArrivedNumber()
        finally:
            traci.close()
    return {"collisions": len(pairs), "by link pair": kinds, "emergency stops": ebrake,
            "teleports": teleports, "arrived": arrived, "flows": len(flows)}


def main():
    global CLEARANCE, ALWAYS_CLEAR
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map")
    ap.add_argument("--net", type=Path, help="network file (default: the map's .net.xml)")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--seconds", type=int, default=3600)
    ap.add_argument("--vph", type=int, default=200, help="vehicles per hour on each link")
    ap.add_argument("--controller", choices=["rb", "phase", "static"], default="rb",
                    help="rb: random ring-and-barrier; phase: random Phase action set; "
                         "static: the network's own fixed-time program (no RESCO control)")
    ap.add_argument("--clearance", type=int, default=CLEARANCE, help="all-red seconds")
    ap.add_argument("--always-clear", action="store_true",
                    help="all-red after every phase change, not only after 'g' phases")
    args = ap.parse_args()
    CLEARANCE, ALWAYS_CLEAR = args.clearance, args.always_clear
    net = args.net or ENV / args.map / f"{args.map}.net.xml"
    print(f"{args.map}: {net}")
    print(f"{args.vph} veh/h on every signalised link, {args.seconds}s, controller={args.controller}, "
          f"yellow {YELLOW}s, all-red {CLEARANCE}s {'after every change' if ALWAYS_CLEAR else 'after g phases'}, "
          "junction collision checking ON")
    for seed in args.seeds:
        r = run(args.map, net, seed, args.seconds, args.vph, args.controller)
        print(f"  seed {seed}: collisions {r['collisions']}, emergency stops {r['emergency stops']}, "
              f"teleports {r['teleports']}, arrived {r['arrived']} ({r['flows']} flows)")
        for (a, b, kind), n in r["by link pair"].most_common(6):
            print(f"      {n:3} x {kind}: {a}  <->  {b}")


if __name__ == "__main__":
    main()
