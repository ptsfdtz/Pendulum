#!/usr/bin/env python3
"""Interactive, bounded balance experiment manager for PendulumLab.

The operator authorizes every homing and balance run with Enter. The deterministic
C++ console owns hardware I/O and safety. This slow loop scores completed runs and
selects one bounded gain change at a time, with rollback to known-good parameters.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import shutil
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

from train_balance_policy import RewardWeights, parse_message, reward

PARAMETERS = ("kp", "kd", "kx", "kv", "ki")
CONTROLLER_REVISION = "run_id_isolated_telemetry_v2"
SEARCH_ALGORITHM = "safe_cem_episodic_policy_search_v12_run_id"
BOUNDS = {"kp": (2.0, 60.0), "kd": (2.0, 60.0), "kx": (0.005, 0.08),
          "kv": (0.08, 0.28), "ki": (0.0, 0.002)}
STEPS = {"kp": 0.20, "kd": 0.02, "kx": 0.005, "kv": 0.005,
         "ki": 0.0005}
INITIAL_STD = {"kp": 2.0, "kd": 3.0, "kx": 0.004,
               "kv": 0.012, "ki": 0.00005}
MIN_STD = {"kp": 0.5, "kd": 0.75, "kx": 0.002,
           "kv": 0.006, "ki": 0.00002}


@dataclass
class Metrics:
    score: float
    duration_seconds: float
    samples: int
    rms_angle_radians: float
    rms_cart_position: float
    rms_angular_speed: float
    rms_cart_speed: float
    rms_control: float
    rms_control_change: float
    termination: str
    success: bool


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def clamp(value: float, limits: tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def initial_parameters(config: dict) -> dict[str, float]:
    balance = config["balance_control"]
    return {
        "kp": float(balance["angle_gain_percent_at_maximum_angle"]),
        "kd": float(balance["angular_rate_gain_percent_at_maximum_rate"]),
        "kx": float(balance["cart_position_gain_rated_torque_per_half_travel"]),
        "kv": float(balance["cart_velocity_gain_rated_torque_per_half_travel_per_second"]),
        "ki": float(balance["cart_integral_gain_rated_torque_per_half_travel_second"]),
    }


def candidate_for(index: int, best: dict[str, float],
                  step_scale: float = 1.0) -> tuple[dict[str, float], str]:
    parameter = PARAMETERS[(index // 2) % len(PARAMETERS)]
    direction = 1.0 if index % 2 == 0 else -1.0
    candidate = best.copy()
    candidate[parameter] = clamp(
        best[parameter] + direction * STEPS[parameter] * step_scale,
                                 BOUNDS[parameter])
    return candidate, f"{parameter}{'+' if direction > 0 else '-'}"


def new_rl_state(center: dict[str, float], score: float | None = None) -> dict:
    center = center.copy()
    return {
        "algorithm": SEARCH_ALGORITHM,
        "mean": center.copy(),
        "std": INITIAL_STD.copy(),
        "best_parameters": center.copy(),
        "best_score": score,
        "episode": 0,
        "seed": 20260902,
        "replay": [],
    }


def sample_rl_candidate(state: dict) -> dict[str, float]:
    if int(state["episode"]) == 0 and not state.get("replay"):
        return {name: clamp(float(state["mean"][name]), BOUNDS[name])
                for name in PARAMETERS}
    randomizer = random.Random(int(state["seed"]) + int(state["episode"]))
    return {
        name: clamp(randomizer.gauss(float(state["mean"][name]),
                                    float(state["std"][name])), BOUNDS[name])
        for name in PARAMETERS
    }


def update_rl_state(state: dict, parameters: dict[str, float], metrics: Metrics,
                    source_run: str) -> dict:
    experience = {
        "episode": int(state["episode"]), "parameters": parameters.copy(),
        "reward": metrics.score, "duration_seconds": metrics.duration_seconds,
        "success": metrics.success, "termination": metrics.termination,
        "source_run": source_run,
    }
    replay = list(state.get("replay", []))
    replay.append(experience)
    replay = replay[-100:]
    state["replay"] = replay
    state["episode"] = int(state["episode"]) + 1
    if state.get("best_score") is None or metrics.score > float(state["best_score"]):
        state["best_score"] = metrics.score
        state["best_parameters"] = parameters.copy()

    window = replay[-20:]
    if len(window) >= 6:
        elite_count = max(3, len(window) // 4)
        elites = sorted(window, key=lambda item: float(item["reward"]),
                        reverse=True)[:elite_count]
        rewards = [float(item["reward"]) for item in elites]
        reward_floor = min(rewards)
        weights = [max(1e-6, reward - reward_floor + 1e-3)
                   for reward in rewards]
        weight_sum = sum(weights)
        for name in PARAMETERS:
            elite_mean = sum(weight * float(item["parameters"][name])
                             for weight, item in zip(weights, elites)) / weight_sum
            elite_variance = sum(
                weight * (float(item["parameters"][name]) - elite_mean) ** 2
                for weight, item in zip(weights, elites)) / weight_sum
            old_mean = float(state["mean"][name])
            old_std = float(state["std"][name])
            state["mean"][name] = clamp(
                0.65 * old_mean + 0.35 * elite_mean, BOUNDS[name])
            learned_std = max(MIN_STD[name], math.sqrt(elite_variance))
            state["std"][name] = min(
                (BOUNDS[name][1] - BOUNDS[name][0]) / 3.0,
                max(MIN_STD[name], 0.75 * old_std + 0.25 * learned_std))
    return state


def bootstrap_rl_state(root: Path, state: dict) -> dict:
    if state.get("replay"):
        return state
    records = []
    for directory in sorted(root.glob("run_*"), key=lambda path: path.stat().st_mtime):
        config_path = directory / "config.json"
        metrics_path = directory / "metrics.json"
        if not config_path.exists() or not metrics_path.exists():
            continue
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
            values = json.loads(metrics_path.read_text(encoding="utf-8"))
            if config.get("controller_revision") != CONTROLLER_REVISION:
                continue
            parameters = {name: float(config["parameters"].get(name, 0.0))
                          for name in PARAMETERS}
            metrics = Metrics(**{field: values[field]
                                 for field in Metrics.__dataclass_fields__})
            records.append((parameters, metrics, str(directory)))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
    for parameters, metrics, source in records[-100:]:
        state = update_rl_state(state, parameters, metrics, source)
    state["bootstrapped_experiences"] = min(100, len(records))
    return state


def score_samples(samples: list[dict[str, float]], termination: str,
                  timeout_seconds: float) -> Metrics:
    if not samples:
        return Metrics(-10000.0, 0.0, 0, math.inf, math.inf, math.inf,
                       math.inf, math.inf, math.inf, termination, False)
    previous_action = 0.0
    total = 0.0
    squared = {name: 0.0 for name in
               ("angle", "position", "angular_speed", "cart_speed", "control", "change")}
    for sample in samples:
        state = [sample["theta_rad"], sample["theta_dot_rad_s"],
                 sample["cart_position_half_travel"],
                 sample["cart_velocity_half_travel_s"]]
        action = sample["rated_torque_fraction"]
        total += reward(state, action, previous_action, RewardWeights())
        squared["angle"] += state[0] ** 2
        squared["angular_speed"] += state[1] ** 2
        squared["position"] += state[2] ** 2
        squared["cart_speed"] += state[3] ** 2
        squared["control"] += action ** 2
        squared["change"] += (action - previous_action) ** 2
        previous_action = action
    duration = sum(sample.get("sample_interval_us", 0.0) for sample in samples) * 10.0e-6
    success = termination == "timeout" and duration >= timeout_seconds * 0.8
    if not success:
        total -= 250.0
    count = len(samples)
    rms = lambda name: math.sqrt(squared[name] / count)
    return Metrics(total / count, duration, count, rms("angle"), rms("position"),
                   rms("angular_speed"), rms("cart_speed"), rms("control"),
                   rms("change"), termination, success)


def telemetry_rows(path: Path) -> list[dict[str, float]]:
    result = []
    if not path.exists():
        return result
    with path.open(newline="", encoding="utf-8-sig", errors="replace") as stream:
        for row in csv.DictReader(stream):
            if row.get("component") != "BalanceTelemetry":
                continue
            values = parse_message(row.get("message", ""))
            required = ("sample", "theta_rad", "theta_dot_rad_s",
                        "cart_position_half_travel", "cart_velocity_half_travel_s",
                        "rated_torque_fraction", "sample_interval_us")
            if all(key in values and math.isfinite(values[key]) for key in required):
                result.append(values)
    return result


def log_contains(path: Path, text: str) -> bool:
    return path.exists() and text in path.read_text(encoding="utf-8-sig", errors="replace")


def wait_for_log(log_directory: Path, started: float, timeout: float = 15.0) -> Path:
    deadline = time.time() + timeout
    while time.time() < deadline:
        candidates = [path for path in log_directory.glob("phase1_*.csv")
                      if path.stat().st_mtime >= started - 1.0]
        if candidates:
            return max(candidates, key=lambda path: path.stat().st_mtime)
        time.sleep(0.1)
    raise RuntimeError("Manual console did not create a telemetry log")


def send(process: subprocess.Popen, command: str) -> None:
    if process.poll() is not None or process.stdin is None:
        raise RuntimeError("Manual console exited unexpectedly")
    process.stdin.write(command + "\n")
    process.stdin.flush()


def wait_for_event(log_path: Path, marker: str, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if log_contains(log_path, marker):
            return True
        time.sleep(0.2)
    return False


def marker_count(path: Path, marker: str) -> int:
    if not path.exists():
        return 0
    return path.read_text(encoding="utf-8-sig", errors="replace").count(marker)


def last_line_containing(path: Path, marker: str) -> str:
    if not path.exists():
        return marker
    lines = [line.strip() for line in
             path.read_text(encoding="utf-8-sig", errors="replace").splitlines()
             if marker in line]
    return lines[-1] if lines else marker


def run_id_from_start_line(line: str) -> int:
    marker = "run_id="
    start = line.find(marker)
    if start < 0:
        raise RuntimeError("Accepted balance start did not report a run_id")
    token = line[start + len(marker):].split(",", 1)[0].strip().strip('"')
    try:
        run_id = int(token)
    except ValueError as error:
        raise RuntimeError(f"Invalid balance run_id in console output: {token}") from error
    if run_id <= 0:
        raise RuntimeError(f"Invalid non-positive balance run_id: {run_id}")
    return run_id


def wait_for_new_marker(path: Path, previous: dict[str, int],
                        timeout: float) -> str | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        for marker, old_count in previous.items():
            if marker_count(path, marker) > old_count:
                return marker
        time.sleep(0.2)
    return None


def operator_enter(prompt: str) -> None:
    input(prompt)


def monitor_experiment(process: subprocess.Popen, log_path: Path,
                       timeout_seconds: float,
                       run_id: int
                       ) -> tuple[str, list[dict[str, float]]]:
    started = time.time()
    last_row_count = 0
    last_progress = started
    print("实验运行中：按 Enter 可手动结束。")
    while True:
        rows = [row for row in telemetry_rows(log_path)
                if int(row.get("run_id", -1)) == run_id]
        if len(rows) != last_row_count:
            last_row_count = len(rows)
            last_progress = time.time()
        if rows:
            latest = rows[-1]
            if abs(latest["theta_rad"]) >= 0.25:
                send(process, "balance stop")
                return "pendulum_fall", rows
            if abs(latest["cart_position_half_travel"]) >= 0.80:
                send(process, "balance stop")
                return "cart_envelope", rows
        if process.poll() is not None:
            return "console_exit", rows
        if not rows and time.time() - started >= 2.0:
            return "start_rejected", rows
        if rows and time.time() - last_progress >= 1.0:
            return "controller_abort", rows
        if time.time() - started >= timeout_seconds:
            send(process, "balance stop")
            return "timeout", rows
        if sys.platform == "win32":
            import msvcrt
            if msvcrt.kbhit():
                character = msvcrt.getwch()
                if character == "\r":
                    send(process, "balance stop")
                    return "operator_enter", rows
        time.sleep(0.05)


def save_run(root: Path, run_number: int, parameters: dict[str, float],
             proposal: str, samples: list[dict[str, float]], metrics: Metrics) -> Path:
    directory = root / f"run_{run_number:04d}_{utc_stamp()}"
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "config.json").write_text(json.dumps(
        {"proposal": proposal, "parameters": parameters,
         "controller_revision": CONTROLLER_REVISION}, indent=2) + "\n")
    (directory / "metrics.json").write_text(json.dumps(asdict(metrics), indent=2) + "\n")
    if samples:
        with (directory / "telemetry.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=sorted(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
    return directory


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--console", type=Path,
                        default=Path("out/build/vs2022-x64/Release/pendulum_manual_console.exe"))
    parser.add_argument("--config", type=Path, default=Path("config/config.json"))
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--polarity", choices=("+", "-"), default="+")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not 3.0 <= args.duration <= 60.0:
        parser.error("--duration must be between 3 and 60 seconds")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    root = Path("experiments/balance_optimizer")
    root.mkdir(parents=True, exist_ok=True)
    known_good_path = root / "known_good.json"
    if known_good_path.exists():
        known_good = json.loads(known_good_path.read_text(encoding="utf-8"))
    else:
        known_good = {"parameters": initial_parameters(config), "score": None,
                      "confirmed_successes": 0, "deployment": "operator_supervised"}
        known_good_path.write_text(json.dumps(known_good, indent=2) + "\n")
    if known_good.get("controller_revision") != CONTROLLER_REVISION:
        known_good = {"parameters": initial_parameters(config), "score": None,
                      "confirmed_successes": 0,
                      "deployment": "operator_supervised",
                      "controller_revision": CONTROLLER_REVISION}
        known_good_path.write_text(json.dumps(known_good, indent=2) + "\n")
    search_state_path = root / "search_state.json"
    if search_state_path.exists():
        search_state = json.loads(search_state_path.read_text(encoding="utf-8"))
    else:
        search_state = new_rl_state(known_good["parameters"],
                                    known_good.get("score"))
        search_state_path.write_text(json.dumps(search_state, indent=2) + "\n")
    if search_state.get("algorithm") != SEARCH_ALGORITHM:
        search_state = new_rl_state(initial_parameters(config), None)
    search_state = bootstrap_rl_state(root, search_state)
    search_state_path.write_text(json.dumps(search_state, indent=2) + "\n")
    if args.dry_run:
        print(json.dumps({"known_good": known_good, "next_candidate":
                          sample_rl_candidate(search_state),
                          "search_state": search_state}, indent=2))
        return 0
    if not args.console.exists():
        raise RuntimeError(f"Console executable not found: {args.console}")

    console_output = root / f"console_{utc_stamp()}.txt"
    output_stream = console_output.open("w", encoding="utf-8")
    started = time.time()
    print("正在启动硬件控制台并等待编码器稳定采样...", flush=True)
    process = subprocess.Popen([str(args.console), "--config", str(args.config)],
                               stdin=subprocess.PIPE, stdout=output_stream,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
    try:
        log_path = wait_for_log(Path(config["logging"]["directory"]), started)
        print(f"遥测日志已创建：{log_path}", flush=True)
        if not wait_for_event(console_output, "pendulum> ", 30.0):
            raise RuntimeError("Manual console did not become ready")
        print("硬件控制台已就绪，当前 AO0=0、Servo OFF。", flush=True)
        operator_enter("按 Enter 授权本次 home center（滑台会移动）...")
        print("正在执行 home center...", flush=True)
        send(process, "home center")
        if not wait_for_event(console_output, "Homing complete:", 120.0):
            raise RuntimeError("Home center failed or timed out; outputs were not armed for learning")
        print("已加载当前最佳参数。请手动扶住摆杆接近竖直；启动瞬间的位置将作为本轮零点。")

        run_number = 1
        pending: dict | None = None
        while True:
            operator_enter(f"按 Enter 开始第 {run_number} 次实验；Ctrl+C 退出...")
            if pending is not None:
                parameters = pending["parameters"].copy()
                proposal = pending["proposal"] + "_confirmation"
            else:
                parameters = sample_rl_candidate(search_state)
                proposal = f"rl_episode_{int(search_state['episode']):04d}"
            print("本轮参数：" + ", ".join(
                f"{name}={parameters[name]:.6g}" for name in PARAMETERS) +
                f"，方案={proposal}", flush=True)
            for name in PARAMETERS:
                send(process, f"balance {name} {parameters[name]:.12g}")
            start_markers = {
                marker: marker_count(console_output, marker)
                for marker in ("Balance loop started", "Balance start rejected")
            }
            send(process, f"balance start {args.polarity}")
            start_result = wait_for_new_marker(
                console_output, start_markers, 3.0)
            if start_result == "Balance start rejected":
                print(last_line_containing(console_output,
                                           "Balance start rejected"), flush=True)
                print("本次未启动、未计入实验、未消耗学习回合；请调整摆杆后再次按 Enter。",
                      flush=True)
                continue
            if start_result != "Balance loop started":
                raise RuntimeError(
                    "Balance start produced neither an accepted nor rejected response")
            run_id = run_id_from_start_line(
                last_line_containing(console_output, "Balance loop started"))
            termination, samples = monitor_experiment(
                process, log_path, args.duration, run_id)
            metrics = score_samples(samples, termination, args.duration)
            run_path = save_run(root, run_number, parameters, proposal, samples, metrics)
            print(f"实验结束：{termination}，得分={metrics.score:.4f}，记录={run_path}")

            return_markers = {
                marker: marker_count(console_output, marker)
                for marker in ("Home return complete:", "Home return failed:",
                               "Home return rejected:")
            }
            print("正在使用本次会话已测得的中心计数返回中心...", flush=True)
            send(process, "home return")
            return_result = wait_for_new_marker(
                console_output, return_markers,
                float(config["home_center"]["center_timeout_seconds"]) + 5.0)
            if return_result != "Home return complete:":
                raise RuntimeError(
                    "Automatic return to the known center failed or timed out; "
                    "learning stopped with outputs safe")
            print("滑台已回到已知中心，AO0=0、Servo OFF。", flush=True)

            if metrics.samples > 0:
                search_state = update_rl_state(
                    search_state, parameters, metrics, str(run_path))
                print("本轮经验已加入回放；策略分布已根据高奖励回合更新。")
            else:
                search_state["episode"] = int(search_state["episode"]) + 1
                print("本轮没有有效遥测，仅跳过该回合。")
            search_state_path.write_text(json.dumps(search_state, indent=2) + "\n")

            best_score = known_good["score"]
            if best_score is None:
                if metrics.samples > 0:
                    known_good = {
                        "parameters": parameters,
                        "score": metrics.score,
                        "confirmed_successes": 1 if metrics.success else 0,
                        "baseline_success": metrics.success,
                        "source_run": str(run_path),
                        "deployment": "operator_supervised",
                        "controller_revision": CONTROLLER_REVISION
                    }
                    known_good_path.write_text(json.dumps(known_good, indent=2) + "\n")
                    print("有效遥测已记录为搜索基线；下一轮开始自动修改参数。")
                else:
                    print("本轮没有有效遥测，无法建立基线；参数保持不变。")
            else:
                improved = metrics.success and (
                    metrics.score > float(best_score) + 0.02 * abs(float(best_score)))
                if improved:
                    if pending is None:
                        pending = {"parameters": parameters, "proposal": proposal,
                                   "scores": [metrics.score]}
                    else:
                        pending["scores"].append(metrics.score)
                    confirmations = len(pending["scores"])
                    if confirmations >= 3:
                        median_score = sorted(pending["scores"])[1]
                        known_good = {"parameters": parameters, "score": median_score,
                                      "confirmed_successes": 3,
                                      "source_run": str(run_path),
                                      "deployment": "operator_supervised",
                                      "controller_revision": CONTROLLER_REVISION}
                        known_good_path.write_text(json.dumps(known_good, indent=2) + "\n")
                        pending = None
                        print("候选连续通过 3 次，已升级为当前最佳参数。")
                    else:
                        print(f"候选改善，但还需确认：{confirmations}/3。")
                else:
                    pending = None
                    print("本轮未满足长期最佳晋级条件；工作搜索状态已单独更新。")
            run_number += 1
    except KeyboardInterrupt:
        print("\n实验管理器退出。")
    finally:
        if process.poll() is None:
            try:
                send(process, "balance stop")
                send(process, "quit")
                process.wait(timeout=5)
            except Exception:
                process.terminate()
        output_stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
