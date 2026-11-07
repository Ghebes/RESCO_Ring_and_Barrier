#!/usr/bin/env python3
"""Run and evaluate SUMO's native NEMA controller on Cologne1."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parent
ENV_DIR = ROOT / "resco_benchmark" / "environments" / "cologne1"
NETWORK = ENV_DIR / "cologne1.net.xml"
ROUTES = ENV_DIR / "cologne1.rou.xml"
CONTROLLER = ENV_DIR / "cologne1_nema.add.xml"
DEFAULT_BEGIN = 25200
DEFAULT_END = 28800

PARAM_DEFAULTS = {
    "detector_length": 40.0,
    "detector_length_left": 20.0,
    "left_min_green": 3.0,
    "left_max_green": 12.0,
    "through_min_green": 5.0,
    "through_max_green": 80.0,
    "vehicle_extension": 1.0,
}


def _number(value: float) -> str:
    return str(int(value)) if float(value).is_integer() else str(value)


def write_controller(source: Path, destination: Path, params: dict[str, float]) -> None:
    """Write a parameterized copy of the checked-in controller definition."""
    tree = ET.parse(source)
    logic = tree.getroot().find("tlLogic")
    if logic is None:
        raise ValueError(f"No tlLogic in {source}")

    param_nodes = {node.get("key"): node for node in logic.findall("param")}
    param_nodes["detector-length"].set("value", _number(params["detector_length"]))
    param_nodes["detector-length-leftTurnLane"].set(
        "value", _number(params["detector_length_left"])
    )
    for phase in logic.findall("phase"):
        phase_number = int(phase.get("name", "0"))
        is_left = phase_number % 2 == 1
        phase.set(
            "minDur",
            _number(params["left_min_green" if is_left else "through_min_green"]),
        )
        phase.set(
            "maxDur",
            _number(params["left_max_green" if is_left else "through_max_green"]),
        )
        phase.set("vehext", _number(params["vehicle_extension"]))
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def parse_tripinfo(path: Path) -> dict[str, float | int]:
    trips = list(ET.parse(path).getroot().findall("tripinfo"))
    arrived = [trip for trip in trips if float(trip.get("arrival", "-1")) >= 0]
    unfinished = len(trips) - len(arrived)

    def values(name: str, records=trips) -> list[float]:
        return [float(trip.get(name, "0")) for trip in records]

    def mean(items: list[float]) -> float:
        return statistics.fmean(items) if items else math.nan

    # Match RESCO's publication metric: all emitted tripinfo records (including
    # unfinished trips), with insertion delay included in time loss.
    time_loss = [
        float(trip.get("timeLoss", "0")) + float(trip.get("departDelay", "0"))
        for trip in trips
    ]
    return {
        "arrived": len(arrived),
        "unfinished": unfinished,
        "mean_time_loss": mean(time_loss),
        "total_time_loss": sum(time_loss),
        "mean_duration": mean(values("duration")),
        "mean_waiting_time": mean(values("waitingTime")),
    }


def run_once(
    seed: int,
    params: dict[str, float] | None = None,
    begin: int = DEFAULT_BEGIN,
    end: int = DEFAULT_END,
    sumo_binary: str = "sumo",
    keep_outputs: Path | None = None,
) -> dict[str, float | int]:
    effective = dict(PARAM_DEFAULTS)
    if params:
        effective.update(params)
    if effective["left_min_green"] > effective["left_max_green"]:
        raise ValueError("left_min_green must not exceed left_max_green")
    if effective["through_min_green"] > effective["through_max_green"]:
        raise ValueError("through_min_green must not exceed through_max_green")

    with tempfile.TemporaryDirectory(prefix="cologne1_nema_") as temp_name:
        temp = Path(temp_name)
        additional = temp / "controller.add.xml"
        tripinfo = temp / "tripinfo.xml"
        write_controller(CONTROLLER, additional, effective)
        command = [
            sumo_binary,
            "--net-file", str(NETWORK),
            "--route-files", str(ROUTES),
            "--additional-files", str(additional),
            "--begin", str(begin),
            "--end", str(end),
            "--seed", str(seed),
            "--step-length", "1",
            "--time-to-teleport", "-1",
            "--tripinfo-output", str(tripinfo),
            "--tripinfo-output.write-unfinished", "true",
            "--tripinfo-output.write-undeparted", "true",
            "--no-step-log", "true",
        ]
        completed = subprocess.run(command, text=True, capture_output=True)
        if completed.returncode:
            raise RuntimeError(
                f"SUMO failed for seed {seed}\n{completed.stdout}\n{completed.stderr}"
            )
        result = parse_tripinfo(tripinfo)
        result["seed"] = seed
        if keep_outputs is not None:
            keep_outputs.mkdir(parents=True, exist_ok=True)
            (keep_outputs / f"tripinfo_seed_{seed}.xml").write_bytes(tripinfo.read_bytes())
        return result


def summarize(results: Iterable[dict[str, float | int]]) -> dict[str, object]:
    runs = list(results)
    metrics = (
        "arrived", "unfinished", "mean_time_loss", "total_time_loss",
        "mean_duration", "mean_waiting_time",
    )
    summary: dict[str, object] = {"runs": runs, "run_count": len(runs)}
    for metric in metrics:
        vals = [float(run[metric]) for run in runs]
        summary[metric] = {
            "mean": statistics.fmean(vals),
            "stdev": statistics.stdev(vals) if len(vals) > 1 else 0.0,
            "median": statistics.median(vals),
        }
    return summary


def parse_seeds(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="90210", help="Comma-separated SUMO seeds")
    parser.add_argument("--begin", type=int, default=DEFAULT_BEGIN)
    parser.add_argument("--end", type=int, default=DEFAULT_END)
    parser.add_argument("--sumo-binary", default="sumo")
    parser.add_argument("--output", type=Path, help="Write JSON results here")
    parser.add_argument("--keep-tripinfo", type=Path)
    for name, default in PARAM_DEFAULTS.items():
        parser.add_argument(f"--{name.replace('_', '-')}", type=float, default=default)
    args = parser.parse_args()
    params = {name: getattr(args, name) for name in PARAM_DEFAULTS}
    runs = [
        run_once(seed, params, args.begin, args.end, args.sumo_binary, args.keep_tripinfo)
        for seed in parse_seeds(args.seeds)
    ]
    output = {"controller": "SUMO NEMA actuated", "parameters": params, **summarize(runs)}
    encoded = json.dumps(output, indent=2, sort_keys=True, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
