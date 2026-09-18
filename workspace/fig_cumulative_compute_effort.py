"""Cumulative CHOMP computational effort and iteration count over the mission.

Addresses reviewer comments on the fairness of the batch multi-CHOMP vs
rolling-CHOMP comparison: total computational effort (seconds) and total
executed iterations accumulated over the run, which reflects the different
cold-start / warm-start iteration budgets (batch: 100-iteration full solves;
rolling: 10-iteration warm-start updates plus occasional full re-solves).

Outputs two PNGs per scenario (one graph per file):
    output/cumulative_compute_effort_<scenario>.png
    output/cumulative_chomp_iterations_<scenario>.png
    output/compute_effort_per_run.csv
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
    CHOMP_TIME_EVENTS,
    DEADLINE_SEC,
    FLEET_COLOR,
    FLEET_ORDER,
    OUT_DIR,
    SCENARIO_ORDER,
    chomp_time_samples,
    discover_runs,
    metrics_runs,
)

runs_all = discover_runs()
metrics = metrics_runs(runs_all)
OUT_DIR.mkdir(exist_ok=True)

time_curves: dict[tuple[str, str, str], list] = {}
iter_curves: dict[tuple[str, str, str], list] = {}
max_elapsed: dict[str, float] = {}
rows = []
for r in metrics:
    samples = []
    for event in CHOMP_TIME_EVENTS:
        s = chomp_time_samples(r, event)
        if not s.empty:
            samples.append(s)
    if not samples:
        continue
    s = pd.concat(samples, ignore_index=True).sort_values("elapsed_sec")
    elapsed = s["elapsed_sec"].to_numpy()
    cum_time = np.cumsum(s["cycle_time"].to_numpy())
    iters = pd.to_numeric(s["iterations_executed"], errors="coerce").fillna(0).to_numpy()
    cum_iters = np.cumsum(iters)
    key = (r.scenario, r.algo, r.fleet)
    time_curves.setdefault(key, []).append((elapsed, cum_time))
    iter_curves.setdefault(key, []).append((elapsed, cum_iters))
    max_elapsed[r.scenario] = max(max_elapsed.get(r.scenario, 0.0), float(elapsed[-1]))
    rows.append(
        {
            "algo": r.algo,
            "fleet": r.fleet,
            "scenario": r.scenario,
            "seed": r.seed,
            "run_ts": r.run_ts,
            "n_cycles": int(len(s)),
            "total_compute_time": float(cum_time[-1]),
            "total_iterations_executed": float(cum_iters[-1]),
            "mean_cycle_time": float(s["cycle_time"].mean()),
        }
    )
per_run = pd.DataFrame(rows)
if per_run.empty:
    raise SystemExit("No CHOMP_TIME_* events found in the metrics logs.")
per_run.to_csv(OUT_DIR / "compute_effort_per_run.csv", index=False)

scenarios = [s for s in SCENARIO_ORDER if s in max_elapsed]
scenarios += sorted(set(max_elapsed) - set(SCENARIO_ORDER))


def _mean_band(curves, grid):
    arr = np.vstack([np.interp(grid, e, c, left=0.0) for e, c in curves])
    n = arr.shape[0]
    mean = arr.mean(axis=0)
    half = np.zeros_like(mean)
    if n >= 2:
        sd = arr.std(axis=0, ddof=1)
        half = stats.t.ppf(0.975, n - 1) * sd / np.sqrt(n)
    return mean, half, n


for scenario in scenarios:
    grid = np.arange(0.0, max_elapsed[scenario] + 1.0, 1.0)
    fleets = [f for f in FLEET_ORDER if (scenario, f) in {(k[0], k[2]) for k in time_curves}]
    for curves, ylab, tag in (
        (time_curves, "Cumulative computation time [s]", "cumulative_compute_effort"),
        (iter_curves, "Cumulative executed CHOMP iterations", "cumulative_chomp_iterations"),
    ):
        fig, ax = plt.subplots(figsize=(10, 6))
        for fleet in fleets:
            for algo in ALGO_ORDER:
                cs = curves.get((scenario, algo, fleet))
                if not cs:
                    continue
                mean, half, n = _mean_band(cs, grid)
                color = FLEET_COLOR.get(fleet, None)
                ls = "--" if algo == "multi_chomp" else "-"
                label = f"{ALGO_SHORT[algo]} {fleet} (n={n})"
                ax.fill_between(grid, mean - half, mean + half, color=color, alpha=0.15, linewidth=0)
                ax.plot(grid, mean, ls, color=color, linewidth=2.2, label=label)
        ax.axvline(DEADLINE_SEC, color="gray", linestyle=":", linewidth=1.5, label="deadline (300 s)")
        ax.set_xlabel("Elapsed time [s]")
        ax.set_ylabel(ylab)
        ax.set_title(
            f"{ylab.split('[')[0].strip()}, {scenario} scenario\n"
            "(mean across runs, band: 95% CI)"
        )
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=9)
        fig.tight_layout()
        out_png = OUT_DIR / f"{tag}_{scenario}.png"
        fig.savefig(out_png, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"saved {out_png}")
