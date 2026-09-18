"""Shared run-discovery and statistics utilities for the reviewer-response figures.

Log format (identical columns in both file kinds):

    timestamp,run_id,task_id,robot_id,event,status,allocation_cost,duration,path,collision_flag,message

Metrics logs (<algo>_metrics_*.csv) contain events:
    MIN_DISTANCE        per robot, distance [m] in `allocation_cost`
    MIN_DISTANCE_AVG    fleet-level average
    CHOMP_TIME_FULL      cold-start full solve, wall time [s] in `duration`;
                        iterations / iterations_executed / round_trip in `message`
    CHOMP_TIME_PARTIAL  warm-start partial update, same layout

Task allocation logs (task_allocation_log_*.csv) contain events:
    ASSIGNED, COMPLETED. COMPLETED rows carry the task execution time in
    `duration` and `collision_flag` (1 = collision occurred during execution).

Directory layout: logs/<algorithm>/<fleet>/<scenario>/*.csv with
<algorithm> in {multi_chomp, rolling_chomp}, <fleet> in {r2, r4, r6},
<scenario> in {dynamic, grouped, mixed, random}.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
LOGS_DIR = Path("logs")
OUT_DIR = Path("output")

SAFETY_DISTANCE = 0.4   # m, inter-robot safety threshold
DEADLINE_SEC = 300.0    # s, mission deadline for the completed-tasks metric

# How elapsed time is computed from the raw timestamps:
#   "file_start" : elapsed = timestamp - first event in the file
#                  (behavior of the original t15_s12_completed_tasks_over_time.py)
#   "mission"    : timestamps are already mission time (t = 0 at simulation start)
ELAPSED_ORIGIN = "file_start"

ALGO_ORDER = ["multi_chomp", "rolling_chomp"]
ALGO_LONG = {
    "multi_chomp": "Batch multi-CHOMP (baseline)",
    "rolling_chomp": "Rolling CHOMP (ours)",
}
ALGO_SHORT = {"multi_chomp": "original", "rolling_chomp": "ours"}
ALGO_COLOR = {"multi_chomp": "#E69F00", "rolling_chomp": "#0072B2"}

FLEET_ORDER = ["r2", "r4", "r6"]
FLEET_COLOR = {"r2": "#009E73", "r4": "#D55E00", "r6": "#CC79A7"}
SCENARIO_ORDER = ["dynamic", "grouped", "mixed", "random"]

METRICS_KINDS = ("multi_chomp_metrics", "rolling_chomp_metrics")
ALLOC_KIND = "task_allocation_log"

CHOMP_TIME_EVENTS = ("CHOMP_TIME_FULL", "CHOMP_TIME_PARTIAL")
CHOMP_EVENT_LABEL = {
    "CHOMP_TIME_FULL": "cold-start full solve",
    "CHOMP_TIME_PARTIAL": "warm-start partial update",
}

_FILENAME_RE = re.compile(
    r"^(?P<kind>multi_chomp_metrics|rolling_chomp_metrics|task_allocation_log)"
    r"_[a-z]+_t\d+_s(?P<seed>\d+)_.+_(?P<run_ts>\d{8}_\d{6})\.csv$"
)


@dataclass(frozen=True)
class Run:
    algo: str
    fleet: str
    scenario: str
    seed: int
    run_ts: str
    kind: str
    path: Path

    @property
    def setup(self) -> str:
        return f"{self.fleet} {self.scenario}"


def discover_runs(logs_dir: Path = LOGS_DIR) -> list[Run]:
    """Return every run CSV under logs_dir with metadata parsed from path + file name."""
    if not logs_dir.is_dir():
        raise SystemExit(f"Logs directory not found: {logs_dir.resolve()}")
    runs: list[Run] = []
    for algo_dir in sorted(logs_dir.iterdir()):
        if not algo_dir.is_dir():
            continue
        for fleet_dir in sorted(algo_dir.iterdir()):
            if not fleet_dir.is_dir():
                continue
            for scen_dir in sorted(fleet_dir.iterdir()):
                if not scen_dir.is_dir():
                    continue
                for f in sorted(scen_dir.glob("*.csv")):
                    m = _FILENAME_RE.match(f.name)
                    if m is None:
                        continue
                    runs.append(
                        Run(
                            algo=algo_dir.name,
                            fleet=fleet_dir.name,
                            scenario=scen_dir.name,
                            seed=int(m.group("seed")),
                            run_ts=m.group("run_ts"),
                            kind=m.group("kind"),
                            path=f,
                        )
                    )
    if not runs:
        raise SystemExit(f"No run CSVs found under {logs_dir.resolve()}")
    return runs


def metrics_runs(runs: list[Run]) -> list[Run]:
    return [r for r in runs if r.kind in METRICS_KINDS]


def alloc_runs(runs: list[Run]) -> list[Run]:
    return [r for r in runs if r.kind == ALLOC_KIND]


def setup_order(runs: list[Run]) -> list[tuple[str, str]]:
    """Ordered (fleet, scenario) pairs present in runs."""
    scenarios = [s for s in SCENARIO_ORDER if any(r.scenario == s for r in runs)]
    scenarios += sorted({r.scenario for r in runs} - set(SCENARIO_ORDER))
    fleets = [f for f in FLEET_ORDER if any(r.fleet == f for r in runs)]
    fleets += sorted({r.fleet for r in runs} - set(FLEET_ORDER))
    return [
        (f, s)
        for f in fleets
        for s in scenarios
        if any(r.fleet == f and r.scenario == s for r in runs)
    ]


def _elapsed(ts: np.ndarray, t0: float) -> np.ndarray:
    ts = np.asarray(ts, dtype=float)
    return ts if ELAPSED_ORIGIN == "mission" else ts - t0


def parse_message(message) -> dict:
    """Parse 'key=value;key=value;...' log messages into a dict."""
    if not isinstance(message, str):
        return {}
    out: dict[str, str] = {}
    for part in message.split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k.strip()] = v.strip()
    return out


# --------------------------------------------------------------------------- #
# Safety / inter-robot distance statistics
# --------------------------------------------------------------------------- #
def fleet_min_distance(run: Run) -> pd.DataFrame | None:
    """Fleet-level minimum inter-robot distance time series for one run."""
    df = pd.read_csv(run.path)
    if "event" not in df.columns or "timestamp" not in df.columns:
        return None
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    sub = df[(df["event"] == "MIN_DISTANCE") & (df["robot_id"].astype(str) != "fleet")]
    sub = sub.dropna(subset=["timestamp"])
    if sub.empty or "allocation_cost" not in sub.columns:
        return None
    sub = sub.copy()
    sub["allocation_cost"] = pd.to_numeric(sub["allocation_cost"], errors="coerce")
    sub = sub.dropna(subset=["allocation_cost"])
    if sub.empty:
        return None
    t0 = sub["timestamp"].min()
    grouped = sub.groupby("timestamp")["allocation_cost"].min().sort_index()
    return pd.DataFrame(
        {
            "elapsed_sec": _elapsed(grouped.index.to_numpy(), t0),
            "min_distance": grouped.to_numpy(),
        }
    )


def safety_stats(min_dist: pd.DataFrame) -> dict:
    d = min_dist["min_distance"].to_numpy()
    below = d < SAFETY_DISTANCE
    return {
        "run_min_distance": float(d.min()),
        "run_median_distance": float(np.median(d)),
        "run_mean_distance": float(d.mean()),
        "violation_rate_pct": float(100.0 * below.mean()),
        "violation_samples": int(below.sum()),
        "n_distance_samples": int(d.size),
    }


def collect_safety_per_run(runs: list[Run]) -> pd.DataFrame:
    """One row per run with distance/safety statistics."""
    rows = []
    for r in runs:
        s = fleet_min_distance(r)
        if s is None or s.empty:
            continue
        row = safety_stats(s)
        row.update(algo=r.algo, fleet=r.fleet, scenario=r.scenario, seed=r.seed, run_ts=r.run_ts)
        rows.append(row)
    if not rows:
        raise SystemExit("No MIN_DISTANCE events found in the metrics logs.")
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------- #
# Task-completion statistics (task allocation logs)
# --------------------------------------------------------------------------- #
def completed_series(run: Run) -> tuple[np.ndarray, np.ndarray] | None:
    """(elapsed_sec, cumulative completed task count) for one run."""
    df = pd.read_csv(run.path)
    if "event" not in df.columns or "timestamp" not in df.columns:
        return None
    df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
    df = df.dropna(subset=["timestamp"])
    if df.empty or "task_id" not in df.columns:
        return None
    t0 = df["timestamp"].min()
    comp = df[
        (df["event"] == "COMPLETED")
        & (df["task_id"].astype(str).str.startswith("task_"))
    ].sort_values("timestamp")
    elapsed = _elapsed(comp["timestamp"].to_numpy(), t0)
    counts = np.arange(1, len(comp) + 1)
    return elapsed, counts


def completed_curve(elapsed: np.ndarray, counts: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """Cumulative completed tasks evaluated on a common time grid."""
    if elapsed is None or len(elapsed) == 0:
        return np.zeros_like(grid, dtype=float)
    return np.interp(grid, elapsed, counts, left=0.0)


def collect_completion_per_run(runs: list[Run]) -> pd.DataFrame:
    """One row per run with task-completion, collision and duration statistics."""
    rows = []
    for r in runs:
        ec = completed_series(r)
        if ec is None:
            continue
        elapsed, counts = ec
        at_deadline = int(completed_curve(elapsed, counts, np.array([DEADLINE_SEC]))[0])
        df = pd.read_csv(r.path)
        df["timestamp"] = pd.to_numeric(df["timestamp"], errors="coerce")
        comp = df[(df["event"] == "COMPLETED") & (df["task_id"].astype(str).str.startswith("task_"))]
        flags = pd.to_numeric(comp.get("collision_flag"), errors="coerce").fillna(0).to_numpy()
        durations = pd.to_numeric(comp.get("duration"), errors="coerce")
        rows.append(
            {
                "algo": r.algo,
                "fleet": r.fleet,
                "scenario": r.scenario,
                "seed": r.seed,
                "run_ts": r.run_ts,
                "completed_at_deadline": at_deadline,
                "completed_total": int(len(counts)),
                "n_assigned": int((df["event"] == "ASSIGNED").sum()),
                "collision_tasks": int((flags > 0).sum()),
                "collision_rate_pct": float(100.0 * (flags > 0).mean()) if len(flags) else 0.0,
                "mean_task_duration_sec": float(durations.mean()) if durations.notna().any() else float("nan"),
                "makespan_sec": float(elapsed[-1]) if len(elapsed) else np.nan,
            }
        )
    if not rows:
        raise SystemExit("No COMPLETED task events found in the allocation logs.")
    return pd.DataFrame(rows)


# --------------------------------------------------------------------------- #
# Generic statistics
# --------------------------------------------------------------------------- #
def ci95(values) -> tuple[float, float]:
    """(mean, half-width of the two-sided 95% t confidence interval)."""
    a = np.asarray([v for v in values if pd.notna(v)], dtype=float)
    if a.size == 0:
        return np.nan, np.nan
    mean = float(a.mean())
    if a.size < 2:
        return mean, np.nan
    sd = float(a.std(ddof=1))
    if sd == 0.0:
        return mean, 0.0
    half = float(stats.t.ppf(0.975, a.size - 1) * sd / np.sqrt(a.size))
    return mean, half


# --------------------------------------------------------------------------- #
# CHOMP computation-time statistics (metrics logs)
# --------------------------------------------------------------------------- #
def chomp_time_samples(run: Run, event: str) -> pd.DataFrame:
    """Per-cycle CHOMP timing samples of one run for one timing event.

    Columns: elapsed_sec, cycle_time, iterations, iterations_executed,
    round_trip. `cycle_time` is the wall-clock solve time [s] (`duration`
    column); iteration counts come from the `message` field.
    """
    cols = ["elapsed_sec", "cycle_time", "iterations", "iterations_executed", "round_trip"]
    df = pd.read_csv(run.path)
    if "event" not in df.columns:
        return pd.DataFrame(columns=cols)
    sub = df[df["event"] == event]
    if sub.empty or "timestamp" not in sub.columns or "duration" not in sub.columns:
        return pd.DataFrame(columns=cols)
    ts_all = pd.to_numeric(df["timestamp"], errors="coerce")
    t0 = ts_all.min()
    ts = pd.to_numeric(sub["timestamp"], errors="coerce")
    dur = pd.to_numeric(sub["duration"], errors="coerce")
    ok = dur.notna() & ts.notna() & (dur > 0)
    sub = sub.loc[ok]
    if sub.empty:
        return pd.DataFrame(columns=cols)
    iters, iters_exec, rt = [], [], []
    for msg in sub["message"]:
        kv = parse_message(msg)
        iters.append(kv.get("iterations"))
        iters_exec.append(kv.get("iterations_executed"))
        rt.append(kv.get("round_trip"))
    return pd.DataFrame(
        {
            "elapsed_sec": _elapsed(ts[ok].to_numpy(), t0),
            "cycle_time": dur[ok].to_numpy(),
            "iterations": pd.to_numeric(pd.Series(iters, dtype=object), errors="coerce").to_numpy(),
            "iterations_executed": pd.to_numeric(pd.Series(iters_exec, dtype=object), errors="coerce").to_numpy(),
            "round_trip": pd.to_numeric(pd.Series(rt, dtype=object), errors="coerce").to_numpy(),
        }
    )
