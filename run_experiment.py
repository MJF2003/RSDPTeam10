#!/usr/bin/env python3
import argparse
import re
from pathlib import Path

import yaml  # pip install pyyaml


def coerce_value(raw: str):
    """Heuristic type conversion: int -> float -> bool -> None -> str."""
    s = raw.strip()

    # bool
    if s.lower() in {"true", "false"}:
        return s.lower() == "true"

    # none-ish
    if s.lower() in {"none", "null"}:
        return None

    # int
    try:
        return int(s)
    except ValueError:
        pass

    # float
    try:
        return float(s)
    except ValueError:
        pass

    # fallback: string
    return s


def parse_params(tokens):
    """
    Parse a list of 'key=value' strings into a dict.
    Example: ['d=0.1', 'notes=foo'] -> {'d': 0.1, 'notes': 'foo'}
    """
    params = {}
    for t in tokens:
        if "=" not in t:
            raise ValueError(f"Parameter '{t}' is not in key=value form")
        key, val = t.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"Empty key in token '{t}'")
        params[key] = coerce_value(val)
    return params


def next_experiment_dir(root: Path) -> Path:
    """
    Under root, find expN dirs and return the next one (exp1, exp2, ...).
    """
    root.mkdir(exist_ok=True)

    exp_re = re.compile(r"^exp(\d+)$")
    max_idx = 0
    for child in root.iterdir():
        if child.is_dir():
            m = exp_re.match(child.name)
            if m:
                idx = int(m.group(1))
                if idx > max_idx:
                    max_idx = idx

    next_idx = max_idx + 1
    exp_dir = root / f"exp{next_idx}"
    exp_dir.mkdir()
    return exp_dir


def run_rosbag(exp_dir: Path, config: dict):
    """
    Placeholder for your actual experiment logic.
    You know how you want to call rosbag; wire it up here.
    """
    # Example stub: write an empty results.csv
    results_path = exp_dir / "results.csv"
    results_path.write_text("time,value\n")  # replace with real rosbag output

    # Example (pseudo-)command:
    # cmd = [
    #     "ros2", "bag", "record",
    #     "-o", str(results_path),
    #     "/your/topic/name",
    # ]
    # subprocess.run(cmd, check=True)


def main():
    parser = argparse.ArgumentParser(
        description="Run a lidar experiment and store config + results."
    )

    # First positional: experiment name (e.g. 'distances')
    parser.add_argument(
        "experiment",
        help="Experiment name (e.g. 'distances', 'angles', etc.)",
    )

    # Everything after that: free-form key=value params
    parser.add_argument(
        "params",
        nargs="*",
        help="Arbitrary key=value parameters (e.g. d=0.1 notes='test run')",
    )

    args = parser.parse_args()

    # Allow experiment to be either 'distances' or 'experiment_name=distances'
    raw_exp = args.experiment
    if "=" in raw_exp:
        k, v = raw_exp.split("=", 1)
        if k not in {"experiment", "experiment_name", "exp"}:
            raise SystemExit(
                f"First arg looks like key=value, but key '{k}' is unexpected. "
                "Use either 'distances' or 'experiment_name=distances'."
            )
        experiment_name = v
    else:
        experiment_name = raw_exp

    try:
        params = parse_params(args.params)
    except ValueError as e:
        raise SystemExit(str(e))

    # Build config dict
    config = {
        "experiment_name": experiment_name,
        **params,
    }

    # Make directories: <experiment_name>/expN/
    root = Path(experiment_name)
    exp_dir = next_experiment_dir(root)

    # Write config.yaml
    config_path = exp_dir / "config.yaml"
    with config_path.open("w") as f:
        yaml.safe_dump(config, f)

    print(f"Experiment: {experiment_name}")
    print(f"Params: {params}")
    print(f"Created directory: {exp_dir}")
    print(f"Wrote config: {config_path}")

    # Run rosbag or your experiment function
    run_rosbag(exp_dir, config)
    print(f"Wrote results to: {exp_dir / 'results.csv'}")


if __name__ == "__main__":
    main()
