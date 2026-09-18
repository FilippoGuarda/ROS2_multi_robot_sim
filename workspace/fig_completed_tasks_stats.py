"""Task-completion statistics with std and 95% CI across runs.

Addresses reviewer comments on Table 4: report variability across runs
(standard deviation or confidence intervals) instead of averages only.

Outputs (one graph per file):
    output/completed_tasks_stats.png
    output/completed_tasks_per_run.csv
    output/completed_tasks_summary_stats.csv
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
    DEADLINE_SEC,
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
per_run.to_csv(OUT_DIR / "completed_tasks_per_run.csv", index=False)

rows = []
for (algo, fleet, scenario), g in per_run.groupby(["algo", "fleet", "scenario"]):
    m, half = ci95(g["completed_at_deadline"])
    mk, hk = ci95(g["makespan_sec"])
    rows.append(
        {
            "algo": algo,
            "fleet": fleet,
            "scenario": scenario,
            "n_runs": int(len(g)),
            "completed_mean": m,
            "completed_std": g["completed_at_deadline"].std(ddof=1) if len(g) > 1 else float("nan"),
            "completed_ci95": half,
            "completed_ci_lo": m - (half if not np.isnan(half) else 0.0),
            "completed_ci_hi": m + (half if not np.isnan(half) else 0.0),
            "completed_min": int(g["completed_at_deadline"].min()),
            "completed_max": int(g["completed_at_deadline"].max()),
            "collision_tasks_total": int(g["collision_tasks"].sum()),
            "makespan_mean_sec": mk,
            "makespan_ci95_sec": hk,
        }
    )
summary = pd.DataFrame(rows)
summary.to_csv(OUT_DIR / "completed_tasks_summary_stats.csv", index=False)

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
        m, half = ci95(g["completed_at_deadline"])
        err = 0.0 if np.isnan(half) else half
        sd = g["completed_at_deadline"].std(ddof=1)
        pos = i + (j - 0.5) * bar_w
        ax.bar(
            pos,
            m,
            width=bar_w * 0.92,
            color=ALGO_COLOR[algo],
            alpha=0.85,
            yerr=err,
            capsize=3,
            error_kw=dict(ecolor="black", lw=1.2),
        )
        txt = f"{m:.1f} \u00b1 {sd:.1f}" if pd.notna(sd) else f"{m:.1f}"
        ax.annotate(
            txt,
            (pos, m + err),
            textcoords="offset points",
            xytext=(0, 4),
            ha="center",
            va="bottom",
            fontsize=8,
        )
        counts.append((ALGO_SHORT[algo], int(len(g))))
    labels.append(f"{fleet} {scenario}\n(" + ", ".join(f"{a}: {k}" for a, k in counts if k > 0) + ")")

ax.set_xticks(range(len(setups)))
ax.set_xticklabels(labels)
ax.set_ylabel(f"Completed tasks at {DEADLINE_SEC:.0f} s deadline")
ax.set_title(
    "Task completion at deadline across runs\n"
    "(bar: mean across runs; error bar: 95% CI; annotation: mean \u00b1 std)"
)
ax.grid(True, axis="y", alpha=0.3)
handles = [
    Patch(facecolor=ALGO_COLOR[a], alpha=0.85, label=ALGO_LONG[a]) for a in ALGO_ORDER
]
ax.legend(handles=handles, loc="best")
fig.tight_layout()
out_png = OUT_DIR / "completed_tasks_stats.png"
fig.savefig(out_png, dpi=200, bbox_inches="tight")
plt.close(fig)
print(f"saved {out_png}")
