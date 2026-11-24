"""Plot all saved gain candidates and reproduce the stairs-ascent clipping event."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.patches import Rectangle
import numpy as np

from sweep_assembled_gains import AssembledTracking, JOINTS, ROOT, archive_ticks


BASELINE = "hardware_baseline"


def save(fig, out, name):
    fig.savefig(out / f"{name}.png", dpi=180)
    fig.savefig(out / f"{name}.svg")
    plt.close(fig)


def plot_grid(out, candidates, values, combined, *, title, colorbar, name):
    fig, axes = plt.subplots(2, 3, figsize=(12, 7.8), layout="constrained")
    norm = Normalize(vmin=min(values.values()), vmax=max(values.values()))
    for row, joint in enumerate(JOINTS):
        grid = {
            tuple(float(candidate[f"{joint}_{gain}"]) for gain in ("kp", "ki", "kd")):
            candidate["candidate"]
            for candidate in candidates
            if candidate["candidate"].startswith(joint + "_") or candidate["candidate"] == BASELINE
        }
        p_values, i_values, d_values = [sorted({g[i] for g in grid}) for i in range(3)]
        for axis, ki in zip(axes[row], i_values):
            labels = [[grid[p, ki, d] for p in p_values] for d in d_values]
            cells = [[values[label] for label in line] for line in labels]
            colors = axis.imshow(cells, origin="lower", cmap="viridis_r", norm=norm)
            for y, line in enumerate(labels):
                for x, label in enumerate(line):
                    value = cells[y][x]
                    axis.text(x, y, f"{value:.3f}", ha="center", va="center",
                              color="#161923" if norm(value) < .52 else "white", fontsize=10)
                    if label == BASELINE:
                        axis.add_patch(Rectangle((x - .46, y - .46), .92, .92,
                                                 fill=False, edgecolor="#ff854f", linewidth=2.4))
            axis.set(
                title=f"{joint.capitalize()}  |  Ki = {ki:g} A/(deg·s)",
                xticks=range(len(p_values)), xticklabels=[f"{p:g}" for p in p_values],
                yticks=range(len(d_values)), yticklabels=[f"{d:g}" for d in d_values],
                xlabel="Kp (A/deg)", ylabel="Kd (A·s/deg)",
            )
    fig.colorbar(colors, ax=axes, label=colorbar, shrink=.78)
    fig.suptitle(f"{title}\n"
                 "Each panel varies one joint with the other at hardware gains. Orange outlines mark the shared baseline.\n"
                 f"Both joints changed together in one additional candidate. {combined}", fontsize=13)
    save(fig, out, name)


def replay_stairs(out, source, protocol, summary, trials, candidates):
    for path, key in ((source, "source_sha256"), (ROOT / "models/osl_v2_bench.xml", "model_sha256")):
        if hashlib.sha256(path.read_bytes()).hexdigest() != protocol[key]:
            raise ValueError(f"{path} differs from the input used for the saved sweep")
    label = "Stairs up"
    ticks = list(archive_ticks(source, protocol["archive_members"][label]))
    dt = protocol["physics_dt_s"]
    winner = summary["selected"]
    traces = {}
    for name in (BASELINE, winner):
        gains = {joint: tuple(float(candidates[name][f"{joint}_{gain}"]) for gain in ("kp", "ki", "kd"))
                 for joint in JOINTS}
        trace = []
        sim = AssembledTracking(ticks[0], feedback="actuator-difference", ankle_range="hardware",
                                physics_dt=dt, joint_gains=gains, on_step=trace.append)
        for tick in ticks:
            sim.accept(tick)
        knee = [row for row in trace if row["joint"] == "knee"]
        applied = max(abs(row["current_applied_A"]) for row in knee)
        requested = max(abs(row["current_requested_A"]) for row in knee)
        clipped = sum(row["current_clipped"] for row in knee)
        expected = trials[name, label, dt]["joints"]["knee"]
        if applied > 60 or not np.isclose(requested, expected["peak_requested_A"]) or clipped != expected["clipped_steps"]:
            raise ValueError(f"Current trace for {name} disagrees with the saved experiment")
        traces[name] = knee
        print(f"{name}: {requested:.3f} A requested, {applied:.3f} A applied, {clipped} clipped steps")
    with (out / "Stairs_Ascent_Knee_Traces.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["candidate", *traces[BASELINE][0]])
        writer.writeheader()
        for name, rows in traces.items():
            writer.writerows(dict(candidate=name, **row) for row in rows)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), layout="constrained")
    origin = ticks[0]["reference_time_s"]
    reference_times = [tick["reference_time_s"] - origin for tick in ticks]
    axes[0].step(reference_times, [tick["joints"]["knee"]["reference_deg"] for tick in ticks],
                 where="post", color="#222222", linestyle=":", label="Recorded target")
    axes[0].plot(reference_times, [tick["joints"]["knee"]["actual_deg"] for tick in ticks],
                 color="#66785f", linewidth=1, label="Hardware encoder")
    for name, legend, color in [(BASELINE, "Hardware gains in simulation", "#286599"),
                                (winner, "Lowest tuning error", "#b74c25")]:
        rows = traces[name]
        t = np.array([row["source_time_s"] - origin for row in rows])
        requested = np.abs([row["current_requested_A"] for row in rows])
        applied = np.abs([row["current_applied_A"] for row in rows])
        axes[0].plot(t, [row["sim_angle_deg"] for row in rows], color=color, label=legend)
        axes[1].plot(t, requested, color=color, label=legend)
        if name == winner:
            peak_t = t[requested.argmax()]
            axes[2].step(t, requested, where="pre", color=color, linestyle="--", label="Requested")
            axes[2].step(t, applied, where="pre", color="#286599", label="Applied")
            axes[2].set(xlim=(max(0, peak_t - .045), peak_t + .045), ylim=(0, max(requested) * 1.08))
    for axis in axes[1:]:
        axis.axhline(60, color="#333333", linestyle=":", label="60 A command cap")
    for axis, ylabel, subtitle in zip(
        axes, ("Knee angle (deg)", "Absolute knee current (A)", "Absolute knee current (A)"),
        ("Knee motion", "Requested current across the trial", "Clipping event with the lowest-error tuning gains"),
    ):
        axis.set(title=subtitle, xlabel="Elapsed reference time (s)", ylabel=ylabel)
        axis.grid(alpha=.22)
        axis.legend(loc="best", fontsize=8)
    fig.suptitle("Stairs ascent replay  |  200 Hz controller, 0.5 ms physics")
    save(fig, out, "Stairs_Ascent_Current_Clip")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", required=True, type=Path)
    parser.add_argument("--outdir", required=True, type=Path)
    parser.add_argument("--archive", type=Path,
                        default=Path.home() / "Downloads" / "Tracking Results & Codes & Dataset.zip")
    args = parser.parse_args()
    protocol = json.loads((args.sweep / "Protocol.json").read_text(encoding="utf-8"))
    summary = json.loads((args.sweep / "Summary.json").read_text(encoding="utf-8"))
    with (args.sweep / "Gain_Candidates.csv").open(encoding="utf-8", newline="") as stream:
        candidates = {row["candidate"]: row for row in csv.DictReader(stream)}
    with (args.sweep / "Trials.jsonl").open(encoding="utf-8") as stream:
        trials = {(row["candidate"], row["trial"], row["physics_dt_s"]): row for row in map(json.loads, stream)}
    scores = {name: float(row["score"]) for name, row in candidates.items()}
    development_peak = {
        name: max(joint["peak_requested_A"] for trial in protocol["development"]
                  for joint in trials[name, trial, protocol["physics_dt_s"]]["joints"].values())
        for name in candidates
    }
    args.outdir.mkdir(parents=True, exist_ok=False)
    extra = "combined_grid_winners"
    plot_grid(args.outdir, candidates.values(), scores, f"Score {scores[extra]:.3f}.",
              title="All 54 tested gain sets  |  Tuning error",
              colorbar="Normalized target RMSE score (lower is better)", name="All_Gain_Scores")
    plot_grid(args.outdir, candidates.values(), development_peak,
              f"Peak request {development_peak[extra]:.3f} A.",
              title="All 54 tested gain sets  |  Current demand during tuning",
              colorbar="Peak request across both joints and tuning runs (A)", name="All_Gain_Peak_Current")
    replay_stairs(args.outdir, args.archive, protocol, summary, trials, candidates)


if __name__ == "__main__":
    main()
