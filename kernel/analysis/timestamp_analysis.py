import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

desktop = Path.home() / "Desktop"

logs = [
    desktop / "realtime_ratio.csv",
    desktop / "timestamp_loop_log.csv",
    desktop / "timestamp_project_airsim_loop_log.csv",
    desktop / "airsim_step_time.csv",
    desktop / "set_pose.csv",
    desktop / "pose_computation_time.csv",
    desktop / "determinism.csv",
    desktop / "wall_dt_camera.csv",
    desktop / "PA_dt_camera.csv"
]


def read_values(filepath):
    values = []

    with open(filepath, "r") as file:
        lines = file.readlines()

    for line in lines[1:]:
        line = line.strip()
        if line != "":
            values.append(float(line))

    return values


def calc_worst_error(dataset, expected_value):
    data_min = np.min(dataset)
    data_max = np.max(dataset)

    max_error = abs((data_max - expected_value) / expected_value) * 100
    min_error = abs((data_min - expected_value) / expected_value) * 100
    worst_error = max(max_error, min_error)

    return max_error, min_error, worst_error


def analyze_csv(
    filepath,
    title=None,
    ylabel="seconds",
    expected_value=None,
    expected_label=None
):
    if not filepath.exists():
        print(f"Skipping missing file: {filepath}")
        return

    dataset = read_values(filepath)

    if len(dataset) == 0:
        print(f"Skipping empty file: {filepath}")
        return

    samples = list(range(len(dataset)))

    data_min = np.min(dataset)
    data_max = np.max(dataset)
    data_mean = np.mean(dataset)
    data_std = np.std(dataset)
    data_var = np.var(dataset)

    if title is None:
        title = filepath.stem.replace("_", " ").title()

    stats_text = (
        f"min = {data_min:.6f}\n"
        f"max = {data_max:.6f}\n"
        f"mean = {data_mean:.6f}\n"
        f"std = {data_std:.6f}\n"
        f"variance = {data_var:.9f}"
    )

    if expected_value is not None:
        max_error, min_error, worst_error = calc_worst_error(dataset, expected_value)

        if expected_label is None:
            expected_label = str(expected_value)

        stats_text += (
            f"\nmax error from {expected_label} = {max_error:.2f}%"
            f"\nmin error from {expected_label} = {min_error:.2f}%"
            f"\nworst error from {expected_label} = {worst_error:.2f}%"
        )

    plt.figure()
    plt.plot(samples, dataset, label=filepath.name)
    plt.subplots_adjust(bottom=0.35)

    plt.figtext(
        0.98, 0.02,
        stats_text,
        ha="right",
        va="bottom"
    )

    plt.title(title)
    plt.xlabel("sample")
    plt.ylabel(ylabel)
    plt.legend()
    plt.show()


for log in logs:
    if log.name == "realtime_ratio.csv":
        analyze_csv(
            log,
            title="Realtime Ratio: sim_dt / wall_dt",
            ylabel="ratio",
            expected_value=1.0,
            expected_label="1.0"
        )

    elif log.name in ["timestamp_loop_log.csv", "airsim_step_time.csv"]:
        analyze_csv(
            log,
            expected_value=0.01,
            expected_label="10ms"
        )

    else:
        analyze_csv(log)