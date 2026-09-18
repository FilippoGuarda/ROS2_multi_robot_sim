"""Task-level collision rate across ALL runs.

Addresses reviewer comments on collision rates over multiple trials, using the
`collision_flag` recorded per completed task in the task allocation logs.

Outputs (one graph per file):
    output/collision_rate.png
    output/collision_per_run.csv
    output/collision_summary.csv
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from logstats_common import (
    ALGO_COLOR,
    ALGO_LONG,
    ALGO_ORDER,
    ALGO_SHORT,
    OUT_DIR,
    alloc_runs,
    ci95,
    collect_completion_per_run,
    discover_runs,
    setup_order,
)

runs_all = discover_runs()
alloc = alloc_runs(runs_all)
per_run = collect_completion_per_run(alloc)
OUT_DIR.mkdir(exist_ok=True)
per_run[
    [
        "algo", "fleet", "scenario", "seed", "run_ts",
        "completed_total", "collision_tasks", "collision_rate_pct",
    ]
].to_csv(OUT_DIR / "collision_per_run.csv", index=False)

rows = []
for (algo, fleet, scenario), g in per_run.groupby(["algo", "fleet", "scenario"]):
    mean, half = ci95(g["collision_rate_pct"])
    rows.append(
        {
            "algo": algo,
            "fleet": fleet,
            "scenario": scenario,
            "n_runs": int(len(g)),
            "completed_total": int(g["completed_total"].sum()),
            "collision_tasks_total": int(g["collision_tasks"].sum()),
            "collision_rate_mean_pct": mean,
            "collision_rate_std_pct": g["collision_rate_pct"].std(ddof=1) if len(g) > 1 else float("nan"),
            "collision_rate_ci95_pct": half,
            "runs_with_collisions": int((g["collision_tasks"] > 0).sum()),
        }
    )
summary = pd.DataFrame(rows)
summary.to_csv(OUT_DIR / "collision_summary.csv", index=False)

setups = setup_order(alloc)
fig, ax = plt.subplots(figsize=(max(7.0, 1.8 * len(setups)), 6.0))
bar_w = 0.34
labels = []
for i, (fleet, scenario) in enumerate(setups):
    counts = []
    for j, algo in enumerate(ALGO_ORDER):
        g = per_run[
            (per_run["algo"] == algo)
            & (per_run["fleet"] == fleet)
            & (per_run["scenario"] == scenario)
        ]
        if g.empty:
            counts.append((ALGO_SHORT[algo], 0))
            continue
        mean, half = ci95(g["collision_rate_pct"])
        err = 0.0 if np.isnan(half) else half
        pos = i + (j - 0.5) * bar_w
        ax.bar(
            pos, mean, width=bar_w * 0.92, color=ALGO_COLOR[algo], alpha=0.85,
            yerr=err, capsize=3, error_kw=dict(ecolor="black", lw=1.2),
        )
        k = int(g["collision_tasks"].sum())
        n = int(g["completed_total"].sum())
        ax.annotate(
            f"{k} / {n}\ntasks w/ collisions",
            (pos, mean + err),
            textcoords="offset points", xytext=(0, 4),
            ha="center", va="bottom", fontsize=8,
        )
        counts.append((ALGO_SHORT[algo], int(len(g))))
    labels.append(f"{fleet} {scenario}\n(" + ", ".join(f"{a}: {k}" for a, k in counts if k > 0) + ")")

ax.axhline(0.0, color="black", linewidth=0.8)
ax.set_xticks(range(len(setups)))
ax.set_xticklabels(labels)
ax.set_ylabel("Completed tasks with collision flag [%]")
ax.set_title(
    "Task-level collision rate across all runs and seeds\n"
    "(bar: mean across runs; error bar: 95% CI; annotation: collisions / completed tasks)"
)
ax.grid(True, axis="y", alpha=0.3)
handles = [Patch(facecolor=ALGO_COLOR[a], alpha=0.85, label=ALGO_LONG[a]) for a in ALGO_ORDER]
ax.legend(handles=handles, loc="best")
fig.tight_layout()
out_png = OUT_DIR / "collision_rate.png"
fig.savefig(out_png, dpi=200, bbox_inches="tight")
plt.close(fig)
print(f"saved {out_png}")
