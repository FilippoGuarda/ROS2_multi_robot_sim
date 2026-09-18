"""Completed tasks over time with mean and 95% confidence band across seeds.

Addresses reviewer comments on variance / confidence information for the
task-completion results (previously a single representative run).

Outputs one PNG + one CSV per scenario (one graph per file):
    output/completed_tasks_over_time_ci_<scenario>.png
    output/completed_tasks_over_time_ci_<scenario>.csv
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import stats

from logstats_common import (
    ALGO_ORDER,
    ALGO_SHORT,
    DEADLINE_SEC,
    FLEET_COLOR,
    FLEET_ORDER,
    OUT_DIR,
    SCENARIO_ORDER,
    alloc_runs,
    completed_curve,
    completed_series,
    discover_runs,
)

runs = alloc_runs(discover_runs())
scenarios = [s for s in SCENARIO_ORDER if any(r.scenario == s for r in runs)]
scenarios += sorted({r.scenario for r in runs} - set(SCENARIO_ORDER))
OUT_DIR.mkdir(exist_ok=True)

for scenario in scenarios:
    series = {}
    for r in runs:
        if r.scenario != scenario:
            continue
        ec = completed_series(r)
        if ec is not None:
            series[(r.algo, r.fleet, r.run_ts)] = ec
    if not series:
        continue
    lasts = [float(el[-1]) for el, _ in series.values() if len(el)]
    t_max = max(lasts + [DEADLINE_SEC])
    grid = np.arange(0.0, t_max + 1.0, 1.0)

    fig, ax = plt.subplots(figsize=(10, 6))
    tidy = []
    fleets = [f for f in FLEET_ORDER if any(k[1] == f for k in series)]
    for fleet in fleets:
        for algo in ALGO_ORDER:
            curves = [
                completed_curve(el, c, grid)
                for (a, f, _), (el, c) in series.items()
                if a == algo and f == fleet
            ]
            if not curves:
                continue
            arr = np.vstack(curves)
            n = arr.shape[0]
            mean = arr.mean(axis=0)
            half = np.zeros_like(mean)
            if n >= 2:
                sd = arr.std(axis=0, ddof=1)
                half = stats.t.ppf(0.975, n - 1) * sd / np.sqrt(n)
            color = FLEET_COLOR.get(fleet, None)
            ls = "--" if algo == "multi_chomp" else "-"
            label = f"{ALGO_SHORT[algo]} {fleet} (n={n})"
            ax.fill_between(grid, mean - half, mean + half, color=color, alpha=0.15, linewidth=0)
            ax.plot(grid, mean, ls, color=color, linewidth=2.2, label=label)
            tidy.append(
                pd.DataFrame(
                    {
                        "curve": label,
                        "elapsed_sec": grid,
                        "mean": mean,
                        "ci_lo": mean - half,
                        "ci_hi": mean + half,
                        "n_runs": n,
                    }
                )
            )
    if not tidy:
        plt.close(fig)
        continue
    pd.concat(tidy, ignore_index=True).to_csv(
        OUT_DIR / f"completed_tasks_over_time_ci_{scenario}.csv", index=False
    )
    ax.axvline(DEADLINE_SEC, color="gray", linestyle=":", linewidth=1.5, label="deadline (300 s)")
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel("Completed tasks")
    ax.set_title(f"Completed tasks over time, {scenario} scenario (mean across runs, band: 95% CI)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=9)
    fig.tight_layout()
    out_png = OUT_DIR / f"completed_tasks_over_time_ci_{scenario}.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out_png}")
