import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

desktop = Path.home() / "Desktop"

camera_log = desktop / "timestamp_camera_frame_publish_log.csv"
logs = [
    desktop / "timestamp_sim_execution_log.csv",
    desktop / "timestamp_target_drone_pose_log.csv",
    desktop / "timestamp_interceptor_pose_log.csv",
]

expected_fps = 60
expected_sim_step = 0.01


def read_timestamps(filepath):
    timestamps = []
    with open(filepath, "r") as file:
        lines = file.readlines()

        # skip header
        for line in lines[1:]:
            line = line.strip()
            if line != "":
                timestamps.append(float(line))

    return timestamps




# ---------------- DT PLOTS ----------------
for log in logs:
    timestamps = read_timestamps(log)

    dataset = []
    num_samples = []

    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i - 1]
        dataset.append(dt)
        num_samples.append(i)

    dt_min = np.min(dataset)
    dt_max = np.max(dataset)
    dt_mean = np.mean(dataset)
    var = np.var(dataset)

    err_from_dt_max = abs((dt_max - expected_sim_step) / expected_sim_step)
    err_from_dt_min = abs((dt_min - expected_sim_step) / expected_sim_step)
    largest_err = 100 * np.max([err_from_dt_max, err_from_dt_min])

    plt.plot(num_samples, dataset, label="dt")
    plt.subplots_adjust(bottom=0.3)  # makes extra room under x-axis
    plt.figtext(
        0.98, 0.02,
        f"min dt = {dt_min:.6f} s\n"
        f"max dt = {dt_max:.6f} s\n"
        f"mean dt = {dt_mean:.6f} s\n"
        f"variance = {var:.6f} s^2\n"
        f"error = {largest_err:.3f}%",
        ha="right",
        va="bottom"
    )

    plt.title(log.name)
    plt.xlabel("sample")
    plt.ylabel("dt (seconds)")
    plt.legend()
    plt.show()




# ---------------- CAMERA FPS ----------------
camera_timestamps = read_timestamps(camera_log)

camera_fps_dataset = []
num_samples_camera = []

for i in range(1, len(camera_timestamps)):
    dt_ns = camera_timestamps[i] - camera_timestamps[i - 1]
    fps = 1e9 / dt_ns
    camera_fps_dataset.append(fps)
    num_samples_camera.append(i)

fps_min = np.min(camera_fps_dataset)
fps_max = np.max(camera_fps_dataset)
fps_mean = np.mean(camera_fps_dataset)
camera_var = np.var(camera_fps_dataset)

camera_err_from_fps_max = abs((fps_max - expected_fps) / expected_fps)
camera_err_from_fps_min = abs((fps_min - expected_fps) / expected_fps)
camera_largest_err = 100 * np.max([camera_err_from_fps_max, camera_err_from_fps_min])

plt.plot(num_samples_camera, camera_fps_dataset, label="Camera FPS")
plt.subplots_adjust(bottom=0.3)  # makes extra room under x-axis

plt.figtext(
    0.98, 0.02,
    f"min fps = {fps_min:.3f}\n"
    f"max fps = {fps_max:.3f}\n"
    f"mean fps = {fps_mean:.3f}\n"
    f"variance = {camera_var:.3f}\n"
    f"error = {camera_largest_err:.3f}%",
    ha="right",
    va="bottom"
)
plt.title("FPS for each frame")
plt.xlabel("sample")
plt.ylabel("fps (1/s)")
plt.legend()
plt.show()