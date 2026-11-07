#!/usr/bin/env python3
"""Show, in sumo-gui, what each green phase really serves versus RESCO's label.

Two checks found by test_ring_barrier.py:

  cologne1     Upstream RESCO's signal.yaml said phase 0 = westbound (W-W, W-S)
               and phase 1 = eastbound (E-E, E-N); SUMO shows the opposite.
               After the fix in signal.yaml every phase should say MATCHES.
  stateXuni    saltlake2_stateXuniversity signal A3, "E/W protected lefts":
               does a westbound through lane get a protected green at the same
               time as the eastbound left?

Everything printed is read from SUMO itself:
  * travel direction  - from the incoming lane's geometry (SUMO +y is north)
  * turn type         - from SUMO's connection direction (s / l / r / t)
  * conflicts         - from SUMO's junction model (<request foes=...>)

Usage:
    python show_phase_check.py                 # both checks, sumo-gui
    python show_phase_check.py cologne1
    python show_phase_check.py stateXuni --hold 30
    python show_phase_check.py --nogui         # prints only, no window

Tip: in sumo-gui, Edit > Edit Visualisation > Junctions > "Show link tls index"
draws the link numbers printed below onto the intersection.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import traci
import yaml

ROOT = Path(__file__).resolve().parent
ENV = ROOT / "resco_benchmark" / "environments"
SIGNAL_YAML = ROOT / "resco_benchmark" / "config" / "signal.yaml"

CHECKS = {
    "cologne1": {
        "map": "cologne1",
        "signal": "cluster_357187_359543",
        "routes": ENV / "cologne1" / "cologne1.rou.xml",
        "begin": 25200,
        "phases": [0, 1, 2, 3],
        "question": "Do phases 0 and 1 serve what signal.yaml's phase_pairs says?",
    },
    "stateXuni": {
        "map": "saltlake2_stateXuniversity",
        "signal": "A3",
        "routes": None,  # signals only; the Salt Lake demand is built per date by RESCO
        "begin": 0,
        "phases": [3, 2, 0, 1],
        "question": "Does 'E/W protected lefts' also give a westbound through a protected green?",
    },
    "400sA3": {
        "map": "saltlake2_400sX200w",
        "signal": "A3",
        "routes": None,
        "begin": 0,
        "phases": [2, 3, 6, 7],
        "question": "Which simultaneous protected greens does SUMO mark as conflicting?",
    },
}

COMPASS = {"E": "eastbound", "N": "northbound", "W": "westbound", "S": "southbound"}
LEFT_OF = {"E": "N", "N": "W", "W": "S", "S": "E"}
RIGHT_OF = {v: k for k, v in LEFT_OF.items()}
TURN = {"s": "through", "l": "left", "L": "left", "r": "right", "R": "right", "t": "U-turn"}


def heading(lane_id: str) -> str:
    """Compass travel direction at the end of an incoming lane."""
    (x1, y1), (x2, y2) = traci.lane.getShape(lane_id)[-2:]
    ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
    if -45 <= ang < 45:
        return "E"
    if 45 <= ang < 135:
        return "N"
    if -135 <= ang < -45:
        return "S"
    return "W"


def resco_label(travel: str, turn: str) -> str:
    if turn == "through":
        return f"{travel}-{travel}"
    if turn == "left":
        return f"{travel}-{LEFT_OF[travel]}"
    if turn == "right":
        return f"{travel}-{RIGHT_OF[travel]}"
    return "U-turn"


def link_info(signal_id: str) -> list[dict]:
    info = []
    for idx, conns in enumerate(traci.trafficlight.getControlledLinks(signal_id)):
        if not conns:
            info.append(None)
            continue
        in_lane, out_lane, _via = conns[0]
        turn = "?"
        for link in traci.lane.getLinks(in_lane, True):
            if link[0] == out_lane:
                turn = TURN.get(link[6], link[6])
                break
        travel = heading(in_lane)
        info.append(
            {
                "idx": idx, "in": in_lane, "out": out_lane, "travel": travel,
                "turn": turn, "label": resco_label(travel, turn),
            }
        )
    return info


def junction_foes(net_path: Path, junction_id: str) -> dict[int, set[int]]:
    """SUMO junction model: link index -> set of conflicting link indices."""
    for _, elem in ET.iterparse(net_path, events=("end",)):
        if elem.tag == "junction" and elem.get("id") == junction_id:
            foes = {}
            for req in elem.findall("request"):
                i = int(req.get("index"))
                bits = req.get("foes")
                foes[i] = {j for j, c in enumerate(reversed(bits)) if c == "1"}
            return foes
    return {}


def green_phases(signal_id: str) -> list[str]:
    """Same filter as resco_benchmark.traffic_signal.Signal."""
    out = []
    for p in traci.trafficlight.getAllProgramLogics(signal_id)[0].getPhases():
        if "y" not in p.state and p.state.count("r") + p.state.count("s") != len(p.state):
            out.append(p.state)
    return out


def resco_phase_labels(map_name: str, signal_id: str, n: int) -> dict[int, list[str]]:
    mcfg = yaml.safe_load(SIGNAL_YAML.read_text(encoding="utf-8"))[map_name]
    pairs = mcfg["phase_pairs"]
    p2a = mcfg.get("pair_to_act_map")
    if p2a is None:
        return {i: pairs[i] for i in range(min(n, len(pairs)))}
    return {int(local): pairs[int(pair)] for pair, local in p2a[signal_id].items()}


def movement_turn(mv: str) -> str:
    a, b = mv.split("-")
    if a == b:
        return "through"
    return "left" if LEFT_OF[a] == b else "right"


def missing_ls(label: set[str], lane_sets: dict) -> set[str]:
    """Labelled movements that have no lanes in lane_sets (e.g. A3's W-S)."""
    return {mv for mv in label if not lane_sets.get(mv)}


def describe(movements: set[str]) -> str:
    def one(m):
        a, b = m.split("-")
        kind = "through" if a == b else ("left" if LEFT_OF[a] == b else "right")
        return f"{m} ({COMPASS[a]} {kind})"
    return ", ".join(one(m) for m in sorted(movements)) or "(none)"


def pause(seconds: float, interactive: bool) -> None:
    if interactive:
        input("    ... press Enter for the next phase ")
    else:
        time.sleep(seconds)


def run_check(name: str, args) -> None:
    chk = CHECKS[name]
    net = ENV / chk["map"] / f"{chk['map']}.net.xml"
    sid = chk["signal"]
    binary = "sumo" if args.nogui else "sumo-gui"
    cmd = [binary, "-n", str(net), "--begin", str(chk["begin"]), "--no-warnings", "true",
           "--no-step-log", "true"]
    if chk["routes"]:
        cmd += ["-r", str(chk["routes"])]
    if not args.nogui:
        cmd += ["--start", "--delay", str(args.delay)]

    print("=" * 78)
    print(f"CHECK: {name}  ({chk['map']}, signal {sid})")
    print(f"QUESTION: {chk['question']}")
    print("=" * 78)
    traci.start(cmd, label=name)
    try:
        links = link_info(sid)
        greens = green_phases(sid)
        labels = resco_phase_labels(chk["map"], sid, len(greens))
        foes = junction_foes(net, sid)
        lane_sets = yaml.safe_load(SIGNAL_YAML.read_text(encoding="utf-8"))[chk["map"]][sid]["lane_sets"]

        print("\nIncoming links (directions measured from the lane geometry):")
        for li in links:
            if li:
                print(f"  link {li['idx']:2}: lane {li['in']:<18} {COMPASS[li['travel']]:<11} "
                      f"{li['turn']:<7} -> RESCO label {li['label']}")

        if not args.nogui:
            xs, ys = zip(*[traci.lane.getShape(li["in"])[-1] for li in links if li])
            traci.gui.setOffset("View #0", sum(xs) / len(xs), sum(ys) / len(ys))
            traci.gui.setZoom("View #0", args.zoom)

        for phase in chk["phases"]:
            state = greens[phase]
            traci.trafficlight.setRedYellowGreenState(sid, state)
            protected = {li["label"] for li in links if li and state[li["idx"]] == "G"
                         and li["turn"] in ("through", "left")}
            permissive = {li["label"] for li in links if li and state[li["idx"]] == "g"
                          and li["turn"] in ("through", "left")}
            label = set(labels.get(phase, []))
            print("\n" + "-" * 78)
            print(f"PHASE {phase}: state {state}")
            print(f"  RESCO signal.yaml label : {describe(label)}")
            print(f"  SUMO protected green (G): {describe(protected)}")
            if permissive:
                print(f"  SUMO permissive green(g): {describe(permissive)}")
            served = protected | permissive
            existing = {li["label"] for li in links if li}
            missing = label - existing
            if missing:
                print(f"  (not present at this intersection: {describe(missing)})")
            label_here = label & existing
            verdict = "MATCHES" if label_here <= served else "DOES NOT MATCH"
            print(f"  -> label {verdict} what SUMO serves")
            extra = protected - label
            if extra:
                print(f"  -> SUMO also gives a protected green to: {describe(extra)}")

            # RESCO vs RESCO: label the green lanes using signal.yaml's own
            # lane_sets (no geometry involved) and compare with phase_pairs.
            print("  RESCO-internal check (lane_sets vs phase_pairs, no geometry used):")
            ls_labels = set()
            for li in links:
                if not li or state[li["idx"]] not in "Gg" or li["turn"] not in ("through", "left"):
                    continue
                for mv, lanes in lane_sets.items():
                    if li["in"] in lanes and movement_turn(mv) == li["turn"]:
                        ls_labels.add(mv)
                        print(f"     link {li['idx']:2} ({state[li['idx']]}) lane {li['in']:<18} "
                              f"{li['turn']:<7} lane_sets calls it {mv}")
            agree = label <= ls_labels | missing_ls(label, lane_sets)
            print(f"     lane_sets labels of green lanes: {sorted(ls_labels)}")
            print(f"     phase_pairs label of this phase: {sorted(label)}")
            print(f"     -> signal.yaml {'AGREES WITH' if agree else 'CONTRADICTS'} itself for this phase")

            clashes = []
            g_links = [li for li in links if li and state[li["idx"]] == "G"]
            for a in g_links:
                for b in g_links:
                    if a["idx"] < b["idx"] and b["idx"] in foes.get(a["idx"], set()):
                        clashes.append((a, b))
            if clashes:
                print("  !! SUMO's junction model marks these simultaneous protected greens as FOES:")
                for a, b in clashes:
                    print(f"       link {a['idx']} {COMPASS[a['travel']]} {a['turn']}  x  "
                          f"link {b['idx']} {COMPASS[b['travel']]} {b['turn']}")
            else:
                print("  no conflicting protected greens")

            for _ in range(args.hold):
                traci.simulationStep()
            pause(args.pause, args.interactive)
    finally:
        traci.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("checks", nargs="*", choices=[[], *CHECKS], default=[])
    parser.add_argument("--nogui", action="store_true", help="print only, no sumo-gui window")
    parser.add_argument("--hold", type=int, default=20, help="simulation seconds per phase")
    parser.add_argument("--delay", type=int, default=150, help="sumo-gui ms per simulation step")
    parser.add_argument("--pause", type=float, default=4.0, help="extra seconds between phases")
    parser.add_argument("--zoom", type=float, default=1200.0)
    parser.add_argument("--interactive", action="store_true",
                        help="wait for Enter between phases instead of --pause")
    args = parser.parse_args()
    for name in args.checks or list(CHECKS):
        run_check(name, args)


if __name__ == "__main__":
    main()
