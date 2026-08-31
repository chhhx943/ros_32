import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.pid.experiment import ExperimentRunner, Scenario
from tools.pid.model import PIDGains
from tools.pid.search import generate_candidates
from tools.pid.transport import PythonCanTransport, SimulationTransport


def _gains(data: Dict[str, Any]) -> PIDGains:
    return PIDGains(float(data["kp"]), float(data["ki"]), float(data["kd"]))


def _load_config(path: Path) -> Dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def _write_outputs(output_dir: Path, runner: ExperimentRunner) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "current_pid": runner.current_pid.as_dict(),
        "candidate_pid": runner.candidate_pid.as_dict() if runner.candidate_pid else None,
        "best_pid": runner.best_pid.as_dict(),
        "best_score": runner.best_score,
        "experiment_history": runner.experiment_history,
        "sample_count": len(runner.sample_history),
        "feedback_availability": {"current_a": False, "voltage_v": False},
    }
    with (output_dir / "report.json").open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(report, handle, indent=2, ensure_ascii=False)
    fields = ["time_s", "target", "actual", "error", "control_output", "kp", "ki", "kd",
              "current_a", "voltage_v", "safety_state", "safety_fault"]
    with (output_dir / "samples.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(sample.as_dict() for sample in runner.sample_history)


def main(argv: List[str] = None) -> int:
    parser = argparse.ArgumentParser(description="Safe PID search for the STM32 wheel controller")
    parser.add_argument("--transport", choices=("sim", "python-can"), default="sim")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("default_config.json"))
    parser.add_argument("--iterations", type=int, default=None)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output-dir", type=Path, default=Path("pid_autotune_results"))
    args = parser.parse_args(argv)
    config = _load_config(args.config)
    initial = _gains(config["initial_pid"])
    scenarios = [Scenario(item["name"], float(item["target"]), float(item["duration_s"]))
                 for item in config["scenarios"]]
    iterations = args.iterations if args.iterations is not None else int(config.get("iterations", 5))
    if iterations <= 0:
        parser.error("--iterations must be positive")

    if args.transport == "sim":
        transport = SimulationTransport()
    else:
        transport = PythonCanTransport(channel=args.channel, interface=args.interface)
    runner = ExperimentRunner(transport, initial)
    for candidate in generate_candidates(initial, config["bounds"], iterations, args.seed):
        print("Observe → Analyze → Hypothesis → Select Candidate", candidate.as_dict())
        result = runner.run_candidate(candidate, scenarios)
        print("Evaluate → Compare", result.as_dict())
        if result.aborted:
            print("Next Action: restored safe PID and stopped after", result.abort_reason)
        else:
            print("Next Action: continue bounded search")
    _write_outputs(args.output_dir, runner)
    print(json.dumps({"best_pid": runner.best_pid.as_dict(), "best_score": runner.best_score}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
