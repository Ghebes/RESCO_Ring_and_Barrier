"""Generate SUMO NEMA (dual-ring, two-barrier, fully actuated) controllers.

Works for any RESCO map.  Every movement phase is built from the network's own
links, classified with the map's ``lane_sets`` and SUMO's connection
directions.  Its colours follow the network's original signal program:

  * a left turn gets its own protected phase only where the original program
    ever gives it a protected green ('G');
  * otherwise, and in the through phase, it runs permissively ('g') if the
    original program ever shows it as 'g';
  * right turns get green with their approach's through phase, and keep 's'
    (stop, then go) elsewhere if the original program uses 's' for them.

NEMA numbering matches the hand-written Cologne1 controller
(environments/cologne1/cologne1_nema.add.xml):

    Ring 1:  1 westbound left   2 eastbound through  | 3 northbound left  4 southbound through
    Ring 2:  5 eastbound left   6 westbound through  | 7 southbound left  8 northbound through
             barrier 1 = E/W (phases 1,2,5,6)        | barrier 2 = N/S (phases 3,4,7,8)

Movement labels are RESCO's: '<travel direction>-<exit direction>'.  A phase
that has no links at an intersection is written as 0 in its ring, which is
SUMO's NEMA convention for an omitted phase.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping

LEFT_OF = {"E": "N", "N": "W", "W": "S", "S": "E"}
RIGHT_OF = {v: k for k, v in LEFT_OF.items()}

# NEMA phase -> (kind, travel direction)
NEMA_PHASES = {
    1: ("left", "W"), 2: ("through", "E"), 3: ("left", "N"), 4: ("through", "S"),
    5: ("left", "E"), 6: ("through", "W"), 7: ("left", "S"), 8: ("through", "N"),
}
RING1 = (1, 2, 3, 4)
RING2 = (5, 6, 7, 8)
BARRIER_GROUPS = ((1, 2, 5, 6), (3, 4, 7, 8))

TURN_OF_DIR = {"s": "through", "l": "left", "L": "left", "r": "right", "R": "right", "t": "uturn"}
PRIORITY = {"r": 0, "s": 1, "g": 2, "G": 3}

DEFAULT_PARAMS = {
    # Tuned Cologne1 values (cologne1_nema_100_episodes.json)
    "detector_length": 40.0,
    "detector_length_left": 20.0,
    "left_min_green": 3.0,
    "left_max_green": 12.0,
    "through_min_green": 5.0,
    "through_max_green": 80.0,
    "vehicle_extension": 1.0,
    "yellow": 3.0,
    "red": 2.0,
}


def _num(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else str(v)


def parse_net(net_path) -> dict:
    """tl id -> {"links": {idx: (lane, dir)}, "states": [original phase states]}."""
    tls: dict = {}
    for _, e in ET.iterparse(net_path, events=("end",)):
        if e.tag == "tlLogic" and e.get("type", "static") != "NEMA":
            tls.setdefault(e.get("id"), {"links": {}})["states"] = [
                p.get("state") for p in e.findall("phase")
            ]
        elif e.tag == "connection" and e.get("tl") is not None:
            tl = tls.setdefault(e.get("tl"), {"links": {}})
            tl["links"].setdefault(
                int(e.get("linkIndex")), (f"{e.get('from')}_{e.get('fromLane')}", e.get("dir"))
            )
        elif e.tag in ("edge", "junction"):
            e.clear()
    return tls


def classify_links(links: Mapping, lane_sets: Mapping) -> dict:
    """link index -> (travel direction, turn kind).  Raises if a link can't be placed."""
    lane_approach = {}
    for mv, lanes in lane_sets.items():
        for lane in lanes:
            lane_approach.setdefault(lane, mv.split("-")[0])
    out = {}
    for idx, (lane, d) in links.items():
        turn = TURN_OF_DIR.get(d)
        if turn is None:
            raise ValueError(f"link {idx}: unknown connection direction {d!r}")
        travel = None
        if turn != "uturn":
            for mv, lanes in lane_sets.items():
                a, b = mv.split("-")
                kind = "through" if a == b else ("left" if LEFT_OF[a] == b else "right")
                if lane in lanes and kind == turn:
                    travel = a
                    break
        if travel is None:
            travel = lane_approach.get(lane)
        if travel is None:
            raise ValueError(f"link {idx} (lane {lane}, dir {d}) is not in any lane_sets movement")
        out[idx] = (travel, turn)
    return out


def build_phases(classes: Mapping, original_states: list) -> dict:
    """NEMA phase number -> state string, for phases that exist at this signal."""
    n = len(original_states[0])
    best = {i: max((s[i] for s in original_states), key=lambda c: PRIORITY.get(c, 0)) for i in range(n)}
    seen = {i: {s[i] for s in original_states} for i in range(n)}
    default = ["s" if "s" in seen[i] else "r" for i in range(n)]

    phases = {}
    for num, (kind, travel) in NEMA_PHASES.items():
        state = list(default)
        mine = {i: t for i, (a, t) in classes.items() if a == travel}
        if kind == "left":
            lefts = [i for i, t in mine.items() if t == "left"]
            if not lefts or any(best[i] != "G" for i in lefts):
                continue  # no protected left here: the left runs permissively, if at all
            for i, t in mine.items():
                if t == "left" or (t == "uturn" and best[i] == "G"):
                    state[i] = "G"
        else:
            if not any(t in ("through", "right") for t in mine.values()):
                continue
            for i, t in mine.items():
                if t in ("through", "right"):
                    state[i] = "G" if best[i] == "G" else "g"
                elif "g" in seen[i]:
                    state[i] = "g"  # permissive left / U-turn
        phases[num] = "".join(state)
    return phases


def combine(*states: str) -> str:
    return "".join(max(chars, key=lambda c: PRIORITY[c]) for chars in zip(*states))


def ring_string(ring, phases) -> str:
    return ",".join(str(p) if p in phases else "0" for p in ring)


def barrier_phases(group, phases) -> str:
    """Last existing phase of each ring in a barrier group (SUMO barrierPhases)."""
    r1 = [p for p in group if p in RING1 and p in phases]
    r2 = [p for p in group if p in RING2 and p in phases]
    if not r1 or not r2:
        raise ValueError(f"barrier group {group} needs a phase in both rings; have {sorted(phases)}")
    return f"{r1[-1]},{r2[-1]}"


def signal_logic(signal_id: str, phases: Mapping, params: Mapping) -> ET.Element:
    p = {**DEFAULT_PARAMS, **params}
    tl = ET.Element("tlLogic", id=signal_id, type="NEMA", programID="NEMA-actuated", offset="0")
    through_barrier1 = [ph for ph in (2, 6) if ph in phases]
    lefts = [ph for ph in (1, 3, 5, 7) if ph in phases]
    for key, value in (
        ("detector-length", _num(p["detector_length"])),
        ("detector-length-leftTurnLane", _num(p["detector_length_left"])),
        ("total-cycle-length", "120"),
        ("ring1", ring_string(RING1, phases)),
        ("ring2", ring_string(RING2, phases)),
        ("barrierPhases", barrier_phases(BARRIER_GROUPS[0], phases)),
        ("barrier2Phases", barrier_phases(BARRIER_GROUPS[1], phases)),
        ("coordinate-mode", "false"),
        ("minRecall", ",".join(map(str, through_barrier1))),
        ("maxRecall", ""),
        ("fixForceOff", "false"),
        ("latchingDetectors", ",".join(map(str, lefts))),
        ("show-detectors", "false"),
        ("whetherOutputState", "false"),
    ):
        ET.SubElement(tl, "param", key=key, value=value)
    for num in sorted(phases):
        left = NEMA_PHASES[num][0] == "left"
        ET.SubElement(
            tl, "phase", name=str(num), duration="99",
            minDur=_num(p["left_min_green" if left else "through_min_green"]),
            maxDur=_num(p["left_max_green" if left else "through_max_green"]),
            vehext=_num(p["vehicle_extension"]), yellow=_num(p["yellow"]), red=_num(p["red"]),
            state=phases[num],
        )
    return tl


def build_controllers(net_path, signal_cfg: Mapping, signal_ids, params=None) -> dict:
    """signal id -> {"phases": {num: state}, "classes": {...}, "logic": Element}."""
    tls = parse_net(net_path)
    out = {}
    for sid in signal_ids:
        tl = tls[sid]
        classes = classify_links(tl["links"], signal_cfg[sid]["lane_sets"])
        phases = build_phases(classes, tl["states"])
        out[sid] = {"phases": phases, "classes": classes,
                    "logic": signal_logic(sid, phases, params or {})}
    return out


def write_additional(path, controllers: Mapping) -> Path:
    root = ET.Element("additional")
    for sid in controllers:
        root.append(controllers[sid]["logic"])
    ET.indent(root, space="    ")
    path = Path(path)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path
