#!/usr/bin/env python3
"""Train a bounded balance policy offline from PendulumLab telemetry.

The script identifies a local linear dynamics model from logged transitions, then
uses cross-entropy policy search on that model. It never connects to hardware and
marks every generated policy as deployment-disabled.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import random
import statistics
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

STATE_NAMES = ("theta_rad", "theta_dot_rad_s", "cart_position_half_travel",
               "cart_velocity_half_travel_s")
FEATURE_NAMES = STATE_NAMES + ("rated_torque_fraction", "bias")
DEFAULT_WEIGHTS = (4.3, 0.15, -0.02, -0.01, 0.0)
LOWER_BOUNDS = (-12.0, -4.0, -2.0, -2.0, -0.08)
UPPER_BOUNDS = (12.0, 4.0, 2.0, 2.0, 0.08)


@dataclass(frozen=True)
class RewardWeights:
    upright: float = 4.0
    center: float = 2.0
    angular_speed: float = 0.5
    cart_speed: float = 0.5
    effort: float = 0.2
    action_change: float = 0.8
    failure: float = 100.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def policy_action(weights: tuple[float, ...] | list[float], state: list[float],
                  action_limit: float) -> float:
    return clamp(sum(weights[i] * state[i] for i in range(4)) + weights[4],
                 -action_limit, action_limit)


def reward(state: list[float], action: float, previous_action: float,
           weights: RewardWeights = RewardWeights()) -> float:
    theta, theta_dot, position, velocity = state
    penalty = (
        weights.upright * (theta / 0.05) ** 2
        + weights.center * (position / 0.50) ** 2
        + weights.angular_speed * (theta_dot / 1.0) ** 2
        + weights.cart_speed * (velocity / 1.0) ** 2
        + weights.effort * (action / 0.35) ** 2
        + weights.action_change * ((action - previous_action) / 0.10) ** 2
    )
    if abs(theta) >= 0.25 or abs(position) >= 0.80:
        penalty += weights.failure
    return 1.0 - penalty


def parse_message(message: str | None) -> dict[str, float]:
    values: dict[str, float] = {}
    if not message:
        return values
    for part in message.split(","):
        key, separator, raw = part.partition("=")
        if separator:
            try:
                values[key] = float(raw)
            except ValueError:
                pass
    return values


def load_runs(pattern: str) -> list[list[tuple[list[float], float]]]:
    runs: list[list[tuple[list[float], float]]] = []
    for filename in sorted(glob.glob(pattern)):
        samples: list[tuple[list[float], float]] = []
        with open(filename, newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                if row.get("component") != "BalanceTelemetry":
                    continue
                values = parse_message(row.get("message", ""))
                required = STATE_NAMES + ("rated_torque_fraction",)
                if not all(name in values and math.isfinite(values[name]) for name in required):
                    continue
                samples.append(([values[name] for name in STATE_NAMES],
                                values["rated_torque_fraction"]))
        if len(samples) >= 20:
            runs.append(samples)
    if not runs:
        raise RuntimeError(f"No usable BalanceTelemetry runs matched {pattern!r}")
    return runs


def solve(matrix: list[list[float]], vector: list[float]) -> list[float]:
    augmented = [row[:] + [vector[i]] for i, row in enumerate(matrix)]
    size = len(vector)
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise RuntimeError("Dynamics regression matrix is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [augmented[row][item] - factor * augmented[column][item]
                              for item in range(size + 1)]
    return [augmented[row][-1] for row in range(size)]


def fit_dynamics(runs: list[list[tuple[list[float], float]]], ridge: float = 1e-4):
    features: list[list[float]] = []
    targets: list[list[float]] = []
    for run in runs:
        for (state, action), (next_state, _) in zip(run, run[1:]):
            if (abs(state[0]) >= 0.25 or abs(next_state[0]) >= 0.25 or
                    abs(state[2]) >= 0.80 or abs(next_state[2]) >= 0.80 or
                    abs(next_state[0] - state[0]) >= 0.10 or
                    abs(next_state[2] - state[2]) >= 0.20 or
                    abs(action) > 1.0):
                continue
            features.append(state + [action, 1.0])
            targets.append([next_state[i] - state[i] for i in range(4)])
    if len(features) < 100:
        raise RuntimeError("Too few safe local transitions for dynamics identification")
    dimension = len(FEATURE_NAMES)
    gram = [[0.0] * dimension for _ in range(dimension)]
    for feature in features:
        for row in range(dimension):
            for column in range(dimension):
                gram[row][column] += feature[row] * feature[column]
    for index in range(dimension - 1):
        gram[index][index] += ridge
    coefficients: list[list[float]] = []
    for output in range(4):
        rhs = [sum(feature[index] * target[output]
                   for feature, target in zip(features, targets))
               for index in range(dimension)]
        coefficients.append(solve(gram, rhs))
    errors = [[] for _ in range(4)]
    for feature, target in zip(features, targets):
        for output in range(4):
            prediction = sum(coefficients[output][i] * feature[i]
                             for i in range(dimension))
            errors[output].append((prediction - target[output]) ** 2)
    rmse = [math.sqrt(statistics.fmean(values)) for values in errors]
    return coefficients, rmse, len(features)


def model_step(coefficients: list[list[float]], state: list[float], action: float):
    feature = state + [action, 1.0]
    return [state[output] + sum(coefficients[output][i] * feature[i]
                                for i in range(len(feature)))
            for output in range(4)]


def evaluate_policy(candidate: list[float], coefficients: list[list[float]],
                    initial_states: list[list[float]], horizon: int,
                    action_limit: float) -> float:
    episode_scores = []
    for initial in initial_states:
        state = initial[:]
        previous_action = 0.0
        total = 0.0
        for _ in range(horizon):
            action = policy_action(candidate, state, action_limit)
            total += reward(state, action, previous_action)
            state = model_step(coefficients, state, action)
            previous_action = action
            if abs(state[0]) >= 0.25 or abs(state[2]) >= 0.80:
                total -= 100.0
                break
        episode_scores.append(total)
    return statistics.fmean(episode_scores)


def train(coefficients: list[list[float]], initial_states: list[list[float]],
          seed: int, iterations: int, population: int, horizon: int,
          action_limit: float):
    randomizer = random.Random(seed)
    means = list(DEFAULT_WEIGHTS)
    deviations = [3.0, 1.0, 0.5, 0.5, 0.02]
    elite_count = max(4, population // 10)
    history = []
    for iteration in range(iterations):
        candidates = []
        for _ in range(population):
            candidate = [clamp(randomizer.gauss(means[i], deviations[i]),
                               LOWER_BOUNDS[i], UPPER_BOUNDS[i])
                         for i in range(5)]
            score = evaluate_policy(candidate, coefficients, initial_states,
                                    horizon, action_limit)
            candidates.append((score, candidate))
        candidates.sort(reverse=True, key=lambda item: item[0])
        elite = candidates[:elite_count]
        for index in range(5):
            values = [candidate[index] for _, candidate in elite]
            means[index] = statistics.fmean(values)
            deviations[index] = max(1e-4, statistics.pstdev(values) * 0.85)
        history.append({"iteration": iteration + 1,
                        "best_score": elite[0][0], "mean": means[:]})
    return means, history


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs", default="logs/phase1_*.csv")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--seed", type=int, default=20260902)
    parser.add_argument("--iterations", type=int, default=24)
    parser.add_argument("--population", type=int, default=96)
    parser.add_argument("--horizon", type=int, default=180)
    parser.add_argument("--action-limit", type=float, default=0.35)
    args = parser.parse_args()
    if not 0.0 < args.action_limit <= 0.50:
        parser.error("--action-limit must be in (0, 0.50]")

    runs = load_runs(args.logs)
    coefficients, rmse, transitions = fit_dynamics(runs)
    initial_states = [run[index][0] for run in runs
                      for index in range(0, len(run), max(1, len(run) // 8))]
    initial_states = [state for state in initial_states
                      if abs(state[0]) < 0.10 and abs(state[2]) < 0.60][:64]
    if len(initial_states) < 4:
        raise RuntimeError("Too few safe initial states for policy training")
    candidate, history = train(coefficients, initial_states, args.seed,
                               args.iterations, args.population, args.horizon,
                               args.action_limit)
    baseline_score = evaluate_policy(list(DEFAULT_WEIGHTS), coefficients,
                                     initial_states, args.horizon,
                                     args.action_limit)
    candidate_score = evaluate_policy(candidate, coefficients, initial_states,
                                      args.horizon, args.action_limit)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path("experiments") / f"rl_training_{timestamp}" / "policy_candidate.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    artifact = {
        "algorithm": "model_based_cross_entropy_policy_search",
        "deployment_allowed": False,
        "reason": "Offline learned policy requires independent simulation and supervised hardware validation.",
        "source_log_pattern": args.logs,
        "runs": len(runs),
        "transitions": transitions,
        "state": list(STATE_NAMES),
        "action": "bounded_rated_torque_fraction",
        "action_limit": args.action_limit,
        "policy": {"type": "bounded_linear", "weights": dict(zip(
            ("theta", "theta_dot", "cart_position", "cart_velocity", "bias"), candidate))},
        "reward": RewardWeights().__dict__,
        "safety_termination": {"absolute_theta_radians": 0.25,
                               "absolute_cart_half_travel": 0.80},
        "model_delta_state_coefficients": dict(zip(STATE_NAMES, coefficients)),
        "one_step_delta_rmse": dict(zip(STATE_NAMES, rmse)),
        "baseline_model_score": baseline_score,
        "candidate_model_score": candidate_score,
        "seed": args.seed,
        "training": {"iterations": args.iterations, "population": args.population,
                     "horizon_steps": args.horizon, "history": history},
    }
    output.write_text(json.dumps(artifact, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "runs": len(runs),
                      "transitions": transitions, "baseline_score": baseline_score,
                      "candidate_score": candidate_score,
                      "deployment_allowed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
