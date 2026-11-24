"""Rank the recorded gain grid against all six assembled replay trajectories."""

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
import hashlib
import json
from pathlib import Path

from sweep_assembled_gains import JOINTS, ROOT, RUNS, evaluate, init_worker, score


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--archive", type=Path,
                        default=Path.home() / "Downloads" / "Tracking Results & Codes & Dataset.zip")
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    protocol = json.loads((args.sweep / "Protocol.json").read_text(encoding="utf-8"))
    for path, key in ((args.archive, "source_sha256"), (ROOT / "models/osl_v2_bench.xml", "model_sha256")):
        if hashlib.sha256(path.read_bytes()).hexdigest() != protocol[key]:
            raise ValueError(f"{path} differs from the input used for the original sweep")
    with (args.sweep / "Gain_Candidates.csv").open(encoding="utf-8", newline="") as stream:
        gains = {
            row["candidate"]: {joint: tuple(float(row[f"{joint}_{gain}"]) for gain in ("kp", "ki", "kd"))
                               for joint in JOINTS}
            for row in csv.DictReader(stream)
        }
    dt = protocol["physics_dt_s"]
    evaluation = tuple(protocol["evaluation"])
    results = {name: {} for name in gains}
    with (args.sweep / "Trials.jsonl").open(encoding="utf-8") as stream:
        for row in map(json.loads, stream):
            if row["physics_dt_s"] == dt:
                results[row["candidate"]][row["trial"]] = row
    pending = {name: gains[name] for name, rows in results.items() if not all(trial in rows for trial in evaluation)}
    args.outdir.mkdir(parents=True, exist_ok=False)
    with (args.outdir / "Additional_Trials.jsonl").open("x", encoding="utf-8") as log, ProcessPoolExecutor(
        max_workers=args.workers, initializer=init_worker, initargs=(args.archive,)
    ) as pool:
        for name, rows in evaluate(pool, pending, evaluation, dt, log).items():
            results[name].update(rows)
        baseline = results["hardware_baseline"]
        if any(set(rows) != set(RUNS) for rows in results.values()) or not all(
            row["admissible"] for row in baseline.values()
        ):
            raise ValueError("Incomplete replay matrix or failed baseline")
        scores = {name: score(rows, baseline) for name, rows in results.items()}
        ranked = sorted((name for name in scores if scores[name] is not None), key=scores.get)
        winner = ranked[0]
        timing = evaluate(pool, {winner: gains[winner]}, tuple(RUNS), protocol["timing_check_dt_s"], log)[winner]
    ranking = [
        dict(candidate=name, score=scores[name], rejected_trials=[
            trial for trial, row in results[name].items() if not row["admissible"]
        ], gains=gains[name]) for name in ranked + [name for name in gains if name not in ranked]
    ]
    report = dict(original_sweep=str(args.sweep.resolve()), candidate_count=len(gains),
                  admissible_count=len(ranked), winner=winner,
                  objective="Mean target RMSE ratio to the hardware-gain baseline across both joints and six trials",
                  scope="Retrospective ranking on the six available references. No unseen validation set.",
                  admissibility=protocol["admissibility"], ranking=ranking,
                  baseline=baseline, winner_results=results[winner], winner_timing_check=timing)
    (args.outdir / "Ranking.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"{winner}: {gains[winner]}, score {scores[winner]:.6f}, {len(ranked)}/{len(gains)} passed")


if __name__ == "__main__":
    main()
