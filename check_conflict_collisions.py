#!/usr/bin/env python3
"""Find signal phases whose simultaneous protected greens actually collide.

For every green phase of every signal on a map, this script finds pairs of
links that are both protected green ('G') and that SUMO's junction model lists
as foes.  For each such pair it runs a short simulation:

  * 600 veh/h on each of the two links (exact from/to lanes);
  * the signal held on that phase;
  * junction collision checking on (RESCO leaves it off).

It then reports how many vehicle pairs collided.

Usage:
    python check_conflict_collisions.py saltlake2_400sX200w
    python check_conflict_collisions.py saltlake2_400sX200w --signal A3 --seconds 600
    python check_conflict_collisions.py cologne1 grid4x4 ...   # any RESCO map
"""

from __future__ import annotations

import argparse
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import traci

ROOT = Path(__file__).resolve().parent
ENV = ROOT / "resco_benchmark" / "environments"


def parse(net: Path):
    """tl -> greens, tl -> {idx: [(from, fromLane, to, toLane, dir)]}, junction foes."""
    greens, links, foes = {}, {}, {}
    for _, e in ET.iterparse(net, events=("end",)):
        if e.tag == "tlLogic":
            greens[e.get("id")] = [
                p.get("state") for p in e.findall("phase")
                if "y" not in p.get("state")
                and p.get("state").count("r") + p.get("state").count("s") != len(p.get("state"))
            ]
        elif e.tag == "connection" and e.get("tl"):
            links.setdefault(e.get("tl"), {}).setdefault(int(e.get("linkIndex")), []).append(
                (e.get("from"), e.get("fromLane"), e.get("to"), e.get("toLane"), e.get("dir"))
            )
        elif e.tag == "junction":
            reqs = e.findall("request")
            if reqs:
                foes[e.get("id")] = {
                    int(r.get("index")): {j for j, c in enumerate(reversed(r.get("foes"))) if c == "1"}
                    for r in reqs
                }
            e.clear()
    return greens, links, foes


def conflict_pairs(greens, foes, sid):
    out = []
    for p, st in enumerate(greens[sid]):
        for i, c in enumerate(st):
            if c != "G":
                continue
            for j in sorted(foes.get(sid, {}).get(i, ())):
                if i < j < len(st) and st[j] == "G":
                    out.append((p, st, i, j))
    return out


def run_pair(net, sid, state, la, lb, seconds) -> int:
    (fa, fla, ta, _tla, _), (fb, flb, tb, _tlb, _) = la, lb
    routes = f"""<routes>
    <vType id="car" lcStrategic="0" lcSpeedGain="0" lcKeepRight="0" lcCooperative="0"/>
    <flow id="A" type="car" from="{fa}" to="{ta}" begin="0" end="{seconds}" vehsPerHour="600" departLane="{fla}" departSpeed="max"/>
    <flow id="B" type="car" from="{fb}" to="{tb}" begin="0" end="{seconds}" vehsPerHour="600" departLane="{flb}" departSpeed="max"/>
</routes>"""
    with tempfile.TemporaryDirectory() as tmp:
        rou = Path(tmp) / "pair.rou.xml"
        rou.write_text(routes, encoding="utf-8")
        traci.start([
            "sumo", "-n", str(net), "-r", str(rou),
            "--collision.check-junctions", "true", "--collision.action", "warn",
            "--time-to-teleport", "120", "--no-step-log", "true", "--no-warnings", "true",
        ], label="pair")
        seen = set()
        try:
            for _ in range(seconds + 120):
                traci.trafficlight.setRedYellowGreenState(sid, state)
                traci.simulationStep()
                for c in traci.simulation.getCollisions():
                    seen.add(frozenset((c.collider, c.victim)))
        finally:
            traci.close()
    return len(seen)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("maps", nargs="+")
    ap.add_argument("--signal")
    ap.add_argument("--seconds", type=int, default=600)
    args = ap.parse_args()

    for m in args.maps:
        net = ENV / m / f"{m}.net.xml"
        greens, links, foes = parse(net)
        sigs = [args.signal] if args.signal else sorted(greens)
        print(f"=== {m}")
        total = 0
        for sid in sigs:
            pairs = conflict_pairs(greens, foes, sid)
            if not pairs:
                print(f"  {sid}: no simultaneous protected greens that SUMO marks as foes")
                continue
            for p, st, i, j in pairs:
                la, lb = links[sid][i][0], links[sid][j][0]
                n = run_pair(net, sid, st, la, lb, args.seconds)
                total += n
                flag = "COLLIDES" if n else "no collisions"
                print(f"  {sid} phase {p} {st}: link {i} ({la[0]}_{la[1]}->{la[2]}, {la[4]}) x "
                      f"link {j} ({lb[0]}_{lb[1]}->{lb[2]}, {lb[4]}): {n} collisions  [{flag}]")
        print(f"  total colliding vehicle pairs: {total}\n")


if __name__ == "__main__":
    main()
