#!/usr/bin/env python3
"""Tune Cologne1's native NEMA controller and evaluate on held-out seeds."""

from __future__ import annotations

import argparse
import itertools
import json
import random
from pathlib import Path

from run_cologne1_nema import PARAM_DEFAULTS, run_once, summarize


SEARCH_SPACE = {
    "detector_length": [10.0, 20.0, 30.0, 40.0],
    "detector_length_left": [5.0, 10.0, 15.0, 20.0],
    "left_min_green": [3.0, 5.0, 7.0],
    "left_max_green": [12.0, 20.0, 30.0, 40.0],
    "through_min_green": [5.0, 10.0, 15.0],
    "through_max_green": [30.0, 45.0, 60.0, 80.0],
    "vehicle_extension": [1.0, 2.0, 3.0, 4.0],
}


def candidates(trials: int, search_seed: int) -> list[dict[str, float]]:
    names = list(SEARCH_SPACE)
    combinations = [
        dict(zip(names, values))
        for values in itertools.product(*(SEARCH_SPACE[name] for name in names))
        if values[names.index("left_min_green")] <= values[names.index("left_max_green")]
        and values[names.index("through_min_green")] <= values[names.index("through_max_green")]
    ]
    rng = random.Random(search_seed)
    rng.shuffle(combinations)
    default = dict(PARAM_DEFAULTS)
    selected = [default]
    selected.extend(item for item in combinations if item != default)
    return selected[:trials]


def evaluate(params: dict[str, float], seeds: list[int], begin: int, end: int) -> dict:
    return summarize(run_once(seed, params, begin, end) for seed in seeds)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=40)
    parser.add_argument("--training-seeds", type=int, default=5)
    parser.add_argument("--testing-seeds", type=int, default=20)
    parser.add_argument("--seed-base", type=int, default=90210)
    parser.add_argument("--search-seed", type=int, default=20261029)
    parser.add_argument("--begin", type=int, default=25200)
    parser.add_argument("--end", type=int, default=28800)
    parser.add_argument("--output", type=Path, default=Path("cologne1_nema_optimized.json"))
    args = parser.parse_args()
    if min(args.trials, args.training_seeds, args.testing_seeds) < 1:
        parser.error("trials and seed counts must be positive")

    training_seeds = list(range(args.seed_base, args.seed_base + args.training_seeds))
    testing_seeds = list(
        range(args.seed_base + 10000, args.seed_base + 10000 + args.testing_seeds)
    )
    history = []
    for index, params in enumerate(candidates(args.trials, args.search_seed), 1):
        result = evaluate(params, training_seeds, args.begin, args.end)
        objective = result["mean_time_loss"]["mean"]
        history.append({"trial": index, "parameters": params, "objective": objective})
        print(f"trial {index}/{args.trials}: mean time loss={objective:.3f}s", flush=True)

    best = min(history, key=lambda item: item["objective"])
    held_out = evaluate(best["parameters"], testing_seeds, args.begin, args.end)
    report = {
        "controller": "SUMO NEMA actuated",
        "objective": "RESCO mean timeLoss including departDelay",
        "simulation": {"begin": args.begin, "end": args.end},
        "training_seeds": training_seeds,
        "testing_seeds": testing_seeds,
        "best_parameters": best["parameters"],
        "best_training_objective": best["objective"],
        "held_out_performance": held_out,
        "search_history": history,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {args.output}")
    print(
        "held-out mean time loss: "
        f"{held_out['mean_time_loss']['mean']:.3f}s "
        f"(SD {held_out['mean_time_loss']['stdev']:.3f}s)"
    )


if __name__ == "__main__":
    main()
