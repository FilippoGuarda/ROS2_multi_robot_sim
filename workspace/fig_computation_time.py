"""Per-cycle CHOMP computation time for both CHOMP variants.

Addresses reviewer comments on:
  - quantitative computation times per optimization/update cycle to
    substantiate real-time performance
  - the fairness of the batch vs rolling comparison: cold-start full solves
    (CHOMP_TIME_FULL, 100-iteration budget) are reported separately from
    warm-start partial updates (CHOMP_TIME_PARTIAL, 10-iteration budget),
    together with the iteration counts actually executed.

Outputs one PNG per timing event (one graph per file):
    output/computation_time_full.png      (CHOMP_TIME_FULL)
    output/computation_time_partial.png   (CHOMP_TIME_PARTIAL)
    output/computation_time_per_run.csv
    output/computation_time_summary.csv
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
    CHOMP_EVENT_LABEL,
    CHOMP_TIME_EVENTS,
    OUT_DIR,
    chomp_time_samples,
    ci95,
    discover_runs,
    metrics_runs,
    setup_order,
)

runs_all = discover_runs()
metrics = metrics_runs(runs_all)
OUT_DIR.mkdir(exist_ok=True)

rows = []
for r in metrics:
    for event in CHOMP_TIME_EVENTS:
        s = chomp_time_samples(r, event)
        if s.empty:
            continue
        v = s["cycle_time"].to_numpy()
        rows.append(
            {
                "event": event,
                "algo": r.algo,
                "fleet": r.fleet,
                "scenario": r.scenario,
                "seed": r.seed,
                "run_ts": r.run_ts,
                "n_cycles": int(v.size),
                "cycle_time_mean": float(v.mean()),
                "cycle_time_median": float(np.median(v)),
                "cycle_time_p95": float(np.percentile(v, 95)),
                "cycle_time_max": float(v.max()),
                "iterations_budget_mean": float(s["iterations"].mean()),
                "iterations_executed_mean": float(s["iterations_executed"].mean()),
                "round_trip_mean": float(s["round_trip"].mean()),
                "total_compute_time": float(v.sum()),
            }
        )
per_run = pd.DataFrame(rows)
if per_run.empty:
    raise SystemExit("No CHOMP_TIME_* events found in the metrics logs.")
per_run.to_csv(OUT_DIR / "computation_time_per_run.csv", index=False)

rows = []
for (event, algo, fleet, scenario), g in per_run.groupby(
    ["event", "algo", "fleet", "scenario"]
):
    m, half = ci95(g["cycle_time_mean"])
    rows.append(
        {
            "event": event,
            "algo": algo,
            "fleet": fleet,
            "scenario": scenario,
            "n_runs": int(len(g)),
            "n_cycles_total": int(g["n_cycles"].sum()),
            "cycle_time_mean_of_run_means": m,
            "cycle_time_std_across_runs": g["cycle_time_mean"].std(ddof=1) if len(g) > 1 else float("nan"),
            "cycle_time_ci95": half,
            "cycle_time_median_pooled": float(g["cycle_time_median"].median()),
            "cycle_time_p95_pooled": float(g["cycle_time_p95"].max()),
            "cycle_time_max_pooled": float(g["cycle_time_max"].max()),
            "iterations_budget_mean": float(g["iterations_budget_mean"].mean()),
            "iterations_executed_mean": float(g["iterations_executed_mean"].mean()),
            "round_trip_mean": float(g["round_trip_mean"].mean()),
            "total_compute_time_mean": float(g["total_compute_time"].mean()),
        }
    )
summary = pd.DataFrame(rows)
summary.to_csv(OUT_DIR / "computation_time_summary.csv", index=False)

setups = setup_order(metrics)
for event in CHOMP_TIME_EVENTS:
    ev = per_run[per_run["event"] == event]
    if ev.empty:
        print(f"[skip] no '{event}' samples found")
        continue
    fig, ax = plt.subplots(figsize=(max(7.0, 1.8 * len(setups)), 6.0))
    box_w = 0.34
    labels = []
    for i, (fleet, scenario) in enumerate(setups):
        counts = []
        for j, algo in enumerate(ALGO_ORDER):
            vals = ev[
                (ev["algo"] == algo) & (ev["fleet"] == fleet) & (ev["scenario"] == scenario)
            ]["cycle_time_mean"].to_numpy()
            counts.append((ALGO_SHORT[algo], int(vals.size)))
            if vals.size == 0:
                continue
            pos = i + (j - 0.5) * box_w
            ax.boxplot(
                vals,
                positions=[pos],
                widths=box_w * 0.92,
                patch_artist=True,
                showfliers=False,
                boxprops=dict(facecolor=ALGO_COLOR[algo], alpha=0.55, edgecolor=ALGO_COLOR[algo]),
                medianprops=dict(color="black"),
                whiskerprops=dict(color=ALGO_COLOR[algo]),
                capprops=dict(color=ALGO_COLOR[algo]),
            )
            rng = np.random.default_rng(2000 + 37 * i + j)  # deterministic jitter
            jitter = rng.uniform(-0.05, 0.05, size=vals.size)
            ax.scatter(
                pos + jitter, vals, s=22, color=ALGO_COLOR[algo],
                edgecolor="black", linewidth=0.4, zorder=3,
            )
        labels.append(
            f"{fleet} {scenario}\n(" + ", ".join(f"{a}: {k}" for a, k in counts if k > 0) + ")"
        )
    ax.set_xticks(range(len(setups)))
    ax.set_xticklabels(labels)
    ax.set_ylabel("Computation time per cycle [s]")
    ax.set_title(
        f"Per-cycle computation time: {event} ({CHOMP_EVENT_LABEL[event]})\n"
        "(boxes: per-run mean cycle time across runs; points: individual runs)"
    )
    ax.grid(True, axis="y", alpha=0.3)
    handles = [
        Patch(facecolor=ALGO_COLOR[a], alpha=0.55, edgecolor=ALGO_COLOR[a], label=ALGO_LONG[a])
        for a in ALGO_ORDER
    ]
    ax.legend(handles=handles, loc="best")
    fig.tight_layout()
    tag = event.replace("CHOMP_TIME_", "").lower()
    out_png = OUT_DIR / f"computation_time_{tag}.png"
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"saved {out_png}")
