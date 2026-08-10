from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Latest task allocation logs — use only the files below.
files = {
    "Original r2": "task_allocation_log_original_t15_s12_r2_multi_chomp_scip_original_20260810_093657.csv",
    "Original r4": "task_allocation_log_original_t15_s12_r4_multi_chomp_scip_original_20260810_091818.csv",
    "Original r6": "task_allocation_log_original_t15_s12_r6_multi_chomp_scip_original_20260810_074712.csv",
    "Ours r2": "task_allocation_log_ours_t15_s12_r2_multi_chomp_scip_ours_20260810_093037.csv",
    "Ours r4": "task_allocation_log_ours_t15_s12_r4_multi_chomp_scip_ours_20260810_091201.csv",
    "Ours r6": "task_allocation_log_ours_t15_s12_r6_multi_chomp_scip_ours_20260810_073915.csv",
}

out_dir = Path("output")
out_dir.mkdir(exist_ok=True)
summary_rows = []
series = {}

for label, fname in files.items():
    df = pd.read_csv(fname)
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    start_time = df["timestamp"].min()
    completed = df[(df["event"] == "COMPLETED") & (df["task_id"].astype(str).str.startswith("task_"))].copy()
    completed = completed.sort_values("timestamp").reset_index(drop=True)
    completed["elapsed_sec"] = completed["timestamp"] - start_time
    completed["completed_tasks"] = range(1, len(completed) + 1)
    series[label] = completed[["elapsed_sec", "completed_tasks"]].copy()
    for _, row in completed.iterrows():
        summary_rows.append({
            "setup": label,
            "elapsed_sec": float(row["elapsed_sec"]),
            "completed_tasks": int(row["completed_tasks"]),
            "task_id": row["task_id"],
            "robot_id": row["robot_id"],
            "timestamp": float(row["timestamp"]),
        })

summary = pd.DataFrame(summary_rows)
summary.to_csv(out_dir / "t15_s12_completed_tasks_over_time.csv", index=False)

plt.figure(figsize=(12, 7))
styles = {
    "Original r2": {"linestyle": "--", "marker": "o"},
    "Original r4": {"linestyle": "--", "marker": "s"},
    "Original r6": {"linestyle": "--", "marker": "^"},
    "Ours r2": {"linestyle": "-", "marker": "o"},
    "Ours r4": {"linestyle": "-", "marker": "s"},
    "Ours r6": {"linestyle": "-", "marker": "^"},
}

for label in ["Original r2", "Original r4", "Original r6", "Ours r2", "Ours r4", "Ours r6"]:
    s = series[label]
    x = [0.0] + s["elapsed_sec"].tolist()
    y = [0] + s["completed_tasks"].tolist()
    plt.step(x, y, where="post", linewidth=2.0, label=label, linestyle=styles[label]["linestyle"])
    if len(s) > 0:
        plt.plot(s["elapsed_sec"], s["completed_tasks"], linestyle="None", marker=styles[label]["marker"], markersize=4)

plt.xlabel("Elapsed seconds")
plt.ylabel("Completed tasks")
plt.title("Completed tasks over time (t15 s12, r2/r4/r6)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(out_dir / "t15_s12_completed_tasks_over_time.png", dpi=200, bbox_inches="tight")
plt.close()
