from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ours_csv = Path("multi_chomp_metrics_ours_t15_s42_r6_ours_coordinator_20260728_121423.csv").resolve()
orig_csv = Path("multi_chomp_metrics_original_t15_s42_r6_original_coordinator_20260727_124451.csv").resolve()
out_dir = Path("output")
out_dir.mkdir(exist_ok=True)

chart_path = out_dir / "distance_compare_baseline_2.png"
summary_path = out_dir / "distance_compare_summary_safety.csv"

SAFETY_DISTANCE = 0.4


def prepare_robot_series(csv_path, label):
    df = pd.read_csv(csv_path)
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df["allocation_cost"] = pd.to_numeric(df["allocation_cost"], errors="coerce")

    robots = df[(df["robot_id"] != "fleet") & (df["event"] == "MIN_DISTANCE")].copy()
    robots = robots.dropna(subset=["timestamp", "allocation_cost"])
    robots = robots.sort_values(["timestamp", "robot_id"])

    start_time = robots["timestamp"].min()
    robots["elapsed_sec"] = robots["timestamp"] - start_time
    robots["algorithm"] = label

    overall = (
        robots.groupby("timestamp", as_index=False)["allocation_cost"]
        .min()
        .rename(columns={"allocation_cost": "overall_min_distance"})
    )
    overall["elapsed_sec"] = overall["timestamp"] - start_time
    overall["algorithm"] = label
    return robots, overall


orig_robots, orig_overall = prepare_robot_series(orig_csv, "original")
ours_robots, ours_overall = prepare_robot_series(ours_csv, "ours")

summary = pd.concat([
    orig_overall[["algorithm", "elapsed_sec", "overall_min_distance"]],
    ours_overall[["algorithm", "elapsed_sec", "overall_min_distance"]]
], ignore_index=True)
summary.to_csv(summary_path, index=False)


ymin = min(orig_robots["allocation_cost"].min(), ours_robots["allocation_cost"].min(), SAFETY_DISTANCE)
ymax = max(orig_robots["allocation_cost"].max(), ours_robots["allocation_cost"].max(), SAFETY_DISTANCE)


fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=False, constrained_layout=True)

for robot_id, grp in orig_robots.groupby("robot_id"):
    axes[0].plot(grp["elapsed_sec"], grp["allocation_cost"], linewidth=1.1, label=robot_id)
axes[0].axhline(SAFETY_DISTANCE, color="red", linestyle="--", linewidth=1.5, zorder=5)
axes[0].set_title("Original: per-robot minimum distance")
axes[0].set_ylabel("Min dist")
axes[0].set_ylim(ymin, ymax)
axes[0].grid(True, alpha=0.3)

for robot_id, grp in ours_robots.groupby("robot_id"):
    axes[1].plot(grp["elapsed_sec"], grp["allocation_cost"], linewidth=1.1, label=robot_id)
axes[1].axhline(SAFETY_DISTANCE, color="red", linestyle="--", linewidth=1.5, zorder=5)
axes[1].set_title("Ours: per-robot minimum distance")
axes[1].set_ylabel("Min dist")
axes[1].set_ylim(ymin, ymax)
axes[1].grid(True, alpha=0.3)

axes[2].plot(
    orig_overall["elapsed_sec"],
    orig_overall["overall_min_distance"],
    linestyle=":",
    linewidth=2.2,
    label="original overall"
)
axes[2].plot(
    ours_overall["elapsed_sec"],
    ours_overall["overall_min_distance"],
    linestyle="-",
    linewidth=2.2,
    label="ours overall"
)
axes[2].axhline(SAFETY_DISTANCE, color="red", linestyle="--", linewidth=1.5, label="safety distance (= 0.4m)", zorder=5)
axes[2].set_title("Overall minimum distance comparison")
axes[2].set_xlabel("Elapsed sec")
axes[2].set_ylabel("Global min")
axes[2].grid(True, alpha=0.3)
axes[2].legend()

fig.suptitle("Minimum-distance comparison (elapsed sec)")
fig.savefig(chart_path, dpi=200, bbox_inches="tight")
plt.close(fig)