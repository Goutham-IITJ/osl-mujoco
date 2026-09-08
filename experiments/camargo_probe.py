#!/usr/bin/env python3
"""
camargo_probe.py -- acquire and VERIFY the Camargo et al. (2021) dataset.

Camargo J, Ramanathan A, Flanagan W, Young A (2021). "A comprehensive, open-source
dataset of lower limb biomechanics in multiple conditions of stairs, ramps, and
level-ground ambulation and transitions."  J Biomech 119:110320.
doi:10.1016/j.jbiomech.2021.110320

WHY THIS SCRIPT EXISTS
    The paper prints the *processing* of the dataset but not its table and column
    names.  Those must be read off the real files.  So this script DISCOVERS the
    structure instead of assuming it: it walks whatever you downloaded, prints the
    exact file paths and the exact column headers, and INFERS the units from the
    numerical ranges rather than trusting the paper.  Nothing here hard-codes a
    column name as fact -- the expected names are only used to rank candidates,
    and every guess is printed as a guess.

    It also does not average anything.  One subject, one condition, one stride.

STAGES
    --check          is the dataset reachable?  (resolves the three DOIs)
    --probe ROOT     walk the download, report structure/columns/units/strides
    (default: do both)

DEPENDENCIES  --  do NOT run this in osl-mujoco's .venv, which is deliberately
kept to mujoco + numpy only.  Make a separate analysis environment:
    py -3 -m venv .venv-analysis
    .venv-analysis\\Scripts\\python.exe -m pip install numpy scipy h5py requests
    (add  mat73  only if the probe reports v7.3 files that h5py cannot flatten)

USAGE
    .venv-analysis\\Scripts\\python.exe experiments\\camargo_probe.py --check
    .venv-analysis\\Scripts\\python.exe experiments\\camargo_probe.py \\
        --probe "D:\\datasets\\camargo2021" --subject AB19
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

# --------------------------------------------------------------- dataset location
DOIS = {
    "Part 1/3": "10.17632/fcgm3chfff.1",
    "Part 2/3": "10.17632/k9kvm5tn3f.1",   # printed across a line break as
                                          # 'k9kvm5t-' / 'n3f.1'; verify by resolving
    "Part 3/3": "10.17632/jj3r5f9pnf.1",
}
LAB_PAGE = "http://www.epic.gatech.edu/opensource-biomechanics-camargo-et-al/"
SUPPLEMENT = "https://doi.org/10.1016/j.jbiomech.2021.110320"

# Table 1 of the paper, for cross-checking whatever the dataset says about mass.
PAPER_TABLE1 = {
    "AB06": (20, "M", 1.80, 74.8), "AB07": (20, "M", 1.65, 55.3),
    "AB08": (21, "M", 1.74, 72.6), "AB09": (21, "F", 1.63, 63.5),
    "AB10": (22, "M", 1.75, 83.9), "AB11": (21, "M", 1.75, 77.1),
    "AB12": (24, "M", 1.74, 86.2), "AB13": (19, "M", 1.73, 59.0),
    "AB14": (22, "F", 1.52, 58.4), "AB15": (21, "M", 1.78, 96.2),
    "AB16": (20, "F", 1.65, 55.8), "AB17": (19, "M", 1.68, 61.2),
    "AB18": (19, "F", 1.80, 60.1), "AB19": (19, "M", 1.70, 68.0),
    "AB20": (21, "F", 1.71, 68.0), "AB21": (20, "F", 1.57, 58.1),
    "AB23": (20, "M", 1.80, 76.8), "AB24": (21, "F", 1.73, 72.6),
    "AB25": (20, "F", 1.63, 52.2), "AB27": (21, "M", 1.70, 68.0),
    "AB28": (33, "F", 1.69, 62.1), "AB30": (31, "M", 1.77, 77.0),
}
PREFERRED = ["AB19", "AB27", "AB20"]   # cohort-median anthropometry, in order

# Candidate names, used ONLY to rank discovered columns.  Not asserted anywhere.
CAND = {
    "knee_angle":  ["knee_angle_r", "knee_angle_r_moment", "knee_flexion_r",
                    "knee_angle", "knee"],
    "knee_moment": ["knee_angle_r_moment", "knee_moment_r", "knee_angle_r_mom",
                    "knee_flexion_r_moment"],
    "knee_power":  ["knee_angle_r_power", "knee_power_r", "knee_flexion_r_power"],
    "time":        ["Header", "header", "time", "Time", "t"],
    "gait_phase":  ["HeelStrike", "heelstrike", "gait_phase", "GaitPhase",
                    "percent_gait", "ToeOff"],
    "grf_v":       ["vy", "Fy", "FP_vy", "grf_v", "vz", "Fz"],
}


# ============================================================== stage 1: reachable
def check_availability(timeout: float = 20.0) -> dict:
    """Resolve each DOI and, if possible, list the files behind it.

    Prints exactly what came back.  A DOI that resolves is the evidence that the
    numerical dataset exists and is obtainable; the file listing is a bonus.
    """
    try:
        import requests
    except ImportError:
        print("requests is not installed -- cannot check automatically.")
        print("Open these by hand instead:")
        for k, v in DOIS.items():
            print(f"  {k}:  https://doi.org/{v}")
        print(f"  lab page:    {LAB_PAGE}")
        print(f"  supplement:  {SUPPLEMENT}")
        return {"checked": False}

    out = {"checked": True, "parts": {}}
    for label, doi in DOIS.items():
        rec = {"doi": doi}
        url = f"https://doi.org/{doi}"
        try:
            r = requests.get(url, timeout=timeout, allow_redirects=True)
            rec["status"] = r.status_code
            rec["resolved_to"] = r.url
            print(f"  {label}  {doi}")
            print(f"      HTTP {r.status_code} -> {r.url}")
            # Mendeley's public API can enumerate the files; endpoint shape is
            # UNVERIFIED here, so a failure is reported, not treated as fatal.
            ds_id = doi.split("/")[-1].split(".")[0]
            ver = doi.rsplit(".", 1)[-1]
            api = (f"https://data.mendeley.com/public-api/datasets/{ds_id}"
                   f"/files?folder_id=root&version={ver}")
            try:
                a = requests.get(api, timeout=timeout)
                if a.status_code == 200:
                    files = a.json()
                    rec["files"] = [
                        {"name": f.get("filename") or f.get("name"),
                         "size": (f.get("content_details") or {}).get("size")
                                 or f.get("size"),
                         "url": (f.get("content_details") or {}).get("download_url")}
                        for f in (files if isinstance(files, list) else [])
                    ]
                    tot = sum((f["size"] or 0) for f in rec["files"])
                    print(f"      {len(rec['files'])} file(s), "
                          f"{tot / 1e9:.2f} GB total")
                    for f in rec["files"][:12]:
                        sz = f["size"] or 0
                        print(f"        {f['name']}   {sz / 1e6:.1f} MB")
                else:
                    print(f"      file listing API returned {a.status_code} "
                          f"-- download through the browser instead")
            except Exception as e:
                print(f"      file listing unavailable ({type(e).__name__}) "
                      f"-- download through the browser instead")
        except Exception as e:
            rec["error"] = f"{type(e).__name__}: {e}"
            print(f"  {label}  {doi}  UNREACHABLE: {rec['error']}")
        out["parts"][label] = rec

    print(f"\n  lab page:    {LAB_PAGE}")
    print(f"  supplement:  {SUPPLEMENT}")
    print("\n  Download all three parts, unzip them into ONE root directory, then")
    print("  re-run with  --probe <that directory>")
    return out


# ================================================================ stage 2: loaders
def load_table(path: str):
    """Return (colnames, {name: 1-D array}, note) for one dataset file.

    Handles: MATLAB v7 (scipy), MATLAB v7.3 (h5py), and CSV.  MATLAB `table`
    objects serialise as opaque MCOS class data that scipy cannot decode -- that
    case is DETECTED and reported clearly rather than crashing, because it means
    a one-time MATLAB/Octave CSV export is required.
    """
    ext = os.path.splitext(path)[1].lower()

    if ext in (".csv", ".txt", ".tsv"):
        delim = "\t" if ext == ".tsv" else ","
        with open(path, "r", encoding="utf-8-sig") as fh:
            hdr = fh.readline().strip().split(delim)
        arr = np.genfromtxt(path, delimiter=delim, skip_header=1, dtype=float)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        cols = {h: arr[:, i] for i, h in enumerate(hdr) if i < arr.shape[1]}
        return hdr, cols, "csv"

    if ext != ".mat":
        return [], {}, f"unsupported extension {ext}"

    # --- try scipy (MATLAB <= v7)
    try:
        from scipy.io import loadmat
        from scipy.io.matlab._mio5_params import MatlabOpaque  # type: ignore
    except Exception:
        try:
            from scipy.io import loadmat
            MatlabOpaque = ()          # type: ignore
        except ImportError:
            loadmat = None
    if loadmat is not None:
        try:
            md = loadmat(path, squeeze_me=True, struct_as_record=False)
            opaque = [k for k, v in md.items()
                      if not k.startswith("__") and isinstance(v, MatlabOpaque)]
            if opaque:
                return [], {}, ("MATLAB table stored as an OPAQUE MCOS object "
                                f"({opaque}); scipy cannot decode it -- export to "
                                "CSV once from MATLAB/Octave, see --help")
            cols, names = {}, []
            for k, v in md.items():
                if k.startswith("__"):
                    continue
                # a struct with array fields, or a plain 2-D array + a names field
                if hasattr(v, "_fieldnames"):
                    for f in v._fieldnames:
                        a = np.atleast_1d(np.asarray(getattr(v, f)).squeeze())
                        if a.dtype.kind in "fiu" and a.ndim == 1:
                            cols[f] = a.astype(float)
                            names.append(f)
                elif isinstance(v, np.ndarray) and v.dtype.kind in "fiu" and v.ndim == 2:
                    for i in range(v.shape[1]):
                        nm = f"{k}_col{i}"
                        cols[nm] = v[:, i].astype(float)
                        names.append(nm)
            if cols:
                return names, cols, "scipy loadmat"
            return [], {}, f"loadmat gave no numeric columns; keys={list(md)}"
        except NotImplementedError:
            pass                       # v7.3 -> fall through to h5py
        except Exception as e:
            note_scipy = f"scipy failed: {type(e).__name__}: {e}"
        else:
            note_scipy = ""
    else:
        note_scipy = "scipy not installed"

    # --- try h5py (MATLAB v7.3 / HDF5)
    try:
        import h5py
        with h5py.File(path, "r") as f:
            cols, names = {}, []

            def visit(name, obj):
                if isinstance(obj, h5py.Dataset) and obj.dtype.kind in "fiu":
                    a = np.asarray(obj).squeeze()
                    if a.ndim == 1 and a.size > 1:
                        key = name.split("/")[-1]
                        cols[key] = a.astype(float)
                        names.append(key)

            f.visititems(visit)
        if cols:
            return names, cols, "h5py (MATLAB v7.3)"
        return [], {}, "h5py opened the file but found no 1-D numeric datasets"
    except ImportError:
        return [], {}, (note_scipy or "") + " ; h5py not installed"
    except Exception as e:
        return [], {}, (note_scipy or "") + f" ; h5py failed: {type(e).__name__}"


# ============================================================ stage 3: unit checks
def infer_unit(kind: str, a: np.ndarray) -> str:
    """Infer the unit of a signal from its numerical range.

    This is the check the paper cannot give us: the paper says angles are in
    degrees and moments in N.m/kg, but only the file can confirm what was stored.
    """
    a = a[np.isfinite(a)]
    if a.size == 0:
        return "empty"
    lo, hi = float(np.min(a)), float(np.max(a))
    pk = max(abs(lo), abs(hi))
    if kind == "angle":
        if 25.0 <= pk <= 150.0:
            u = "DEGREES"
        elif 0.4 <= pk <= 2.7:
            u = "RADIANS"
        else:
            u = f"UNCLEAR (peak {pk:.3f})"
        sign = ("flexion-POSITIVE" if hi > 20 and lo > -20 else
                "flexion-NEGATIVE (OpenSim gait2392 convention)" if lo < -20 and hi < 20
                else "sign UNCLEAR")
        return f"{u}, {sign}, range [{lo:.3f}, {hi:.3f}]"
    if kind == "moment":
        if pk <= 3.0:
            u = "N.m/kg (mass-normalised)"
        elif 8.0 <= pk <= 400.0:
            u = "N.m (absolute)"
        else:
            u = f"UNCLEAR (peak {pk:.3f})"
        return f"{u}, range [{lo:.3f}, {hi:.3f}]"
    if kind == "power":
        u = "W/kg (mass-normalised)" if pk <= 12.0 else "W (absolute)"
        return f"{u}, range [{lo:.3f}, {hi:.3f}]"
    if kind == "phase":
        if 0.0 <= lo and hi <= 1.01:
            u = "FRACTION 0-1"
        elif hi <= 101.0:
            u = "PERCENT 0-100"
        else:
            u = f"UNCLEAR (max {hi:.2f})"
        return f"{u}, range [{lo:.3f}, {hi:.3f}]"
    if kind == "time":
        d = np.diff(a)
        if d.size and np.all(d > 0):
            dt = float(np.median(d))
            return (f"SECONDS, dt={dt * 1000:.3f} ms -> {1 / dt:.1f} Hz, "
                    f"span {hi - lo:.3f} s, monotonic")
        return f"NON-MONOTONIC, range [{lo:.3f}, {hi:.3f}]"
    return f"range [{lo:.3f}, {hi:.3f}]"


def pick(names: list, kind: str) -> list:
    """Rank discovered column names against the candidate list. Guess, not fact."""
    lower = {n.lower(): n for n in names}
    hits = []
    for c in CAND[kind]:
        if c.lower() in lower:
            hits.append(lower[c.lower()])
    for n in names:
        nl = n.lower()
        if n in hits:
            continue
        if kind == "knee_angle" and "knee" in nl and "moment" not in nl and "power" not in nl:
            hits.append(n)
        elif kind == "knee_moment" and "knee" in nl and "moment" in nl:
            hits.append(n)
        elif kind == "knee_power" and "knee" in nl and "power" in nl:
            hits.append(n)
        elif kind == "gait_phase" and ("heel" in nl or "phase" in nl or "toeoff" in nl):
            hits.append(n)
    return hits


# ================================================================ stage 4: walking
def probe(root: str, want_subject: str) -> dict:
    rep = {"root": root, "subject_requested": want_subject}
    if not os.path.isdir(root):
        sys.exit(f"--probe path does not exist: {root}")

    # ---- find subject directories anywhere in the tree (depth-limited)
    subjects = {}
    for dirpath, dirnames, _ in os.walk(root):
        if dirpath[len(root):].count(os.sep) > 3:
            dirnames[:] = []
            continue
        for d in dirnames:
            if d.upper().startswith("AB") and d.upper() in PAPER_TABLE1:
                subjects.setdefault(d.upper(), os.path.join(dirpath, d))
    print(f"\n[1] SUBJECTS FOUND  ({len(subjects)} of the paper's 22)")
    print("    " + ", ".join(sorted(subjects)) or "    none")
    rep["subjects_found"] = sorted(subjects)
    if not subjects:
        sys.exit("no AB## subject directories found under --probe; check the unzip "
                 "location (all three parts must share one root)")

    order = [want_subject.upper()] + [s for s in PREFERRED if s != want_subject.upper()]
    chosen = next((s for s in order if s in subjects), sorted(subjects)[0])
    if chosen != want_subject.upper():
        print(f"    NOTE  {want_subject.upper()} not present; falling back to {chosen}")
    age, sex, ht, mass_paper = PAPER_TABLE1[chosen]
    print(f"\n[2] SUBJECT {chosen}  (paper Table 1: {age} y, {sex}, {ht} m, {mass_paper} kg)")
    rep.update(subject=chosen, mass_paper_kg=mass_paper, height_paper_m=ht)

    # ---- level-ground trials for that subject
    sroot = subjects[chosen]
    lg_dirs = []
    for dirpath, dirnames, _ in os.walk(sroot):
        for d in dirnames:
            if "levelground" in d.lower() or d.lower() in ("levelground", "level_ground"):
                lg_dirs.append(os.path.join(dirpath, d))
    print(f"\n[3] LEVEL-GROUND DIRECTORIES  ({len(lg_dirs)})")
    for d in lg_dirs:
        print(f"    {os.path.relpath(d, root)}")
        print(f"        sensors: {sorted(os.listdir(d))}")
    rep["levelground_dirs"] = [os.path.relpath(d, root) for d in lg_dirs]
    if not lg_dirs:
        print("    NONE -- this subject has no levelground mode; try another subject "
              "or fall back to the treadmill mode (see the design doc)")
        return rep

    lg = lg_dirs[0]
    sensors = {s: os.path.join(lg, s) for s in sorted(os.listdir(lg))
               if os.path.isdir(os.path.join(lg, s))}

    # ---- probe every sensor folder: exact files, exact columns, inferred units
    print(f"\n[4] FILES AND COLUMNS  (exact, as stored)")
    rep["sensors"] = {}
    KIND_OF = {"ik": "angle", "id": "moment", "jp": "power"}
    for sname, sdir in sensors.items():
        files = sorted(f for f in os.listdir(sdir)
                       if os.path.splitext(f)[1].lower() in (".mat", ".csv"))
        if not files:
            continue
        f0 = os.path.join(sdir, files[0])
        names, cols, note = load_table(f0)
        entry = {"dir": os.path.relpath(sdir, root), "n_files": len(files),
                 "example": files[0], "loader": note, "columns": names}
        print(f"\n    {sname}/   {len(files)} file(s)   e.g. {files[0]}")
        print(f"        loader: {note}")
        if not names:
            print(f"        !! no columns read -- see loader note above")
            rep["sensors"][sname] = entry
            continue
        print(f"        columns ({len(names)}): {names[:24]}"
              f"{' ...' if len(names) > 24 else ''}")
        # units of the interesting ones
        units = {}
        for c in pick(names, "time")[:1] or [n for n in names[:1]]:
            units[c] = infer_unit("time", cols[c])
        kind = KIND_OF.get(sname.lower(), None)
        for key, k in (("knee_angle", "angle"), ("knee_moment", "moment"),
                       ("knee_power", "power"), ("gait_phase", "phase")):
            for c in pick(names, key)[:2]:
                units[c] = infer_unit(k, cols[c])
        if units:
            print("        inferred units:")
            for c, u in units.items():
                print(f"          {c:<28} {u}")
        entry["inferred_units"] = units
        entry["n_samples"] = int(len(next(iter(cols.values()))))
        rep["sensors"][sname] = entry

    # ---- gait phase present?
    gp = None
    for sname, e in rep["sensors"].items():
        for c in e.get("columns", []):
            if c.lower() in ("heelstrike", "gait_phase", "gaitphase", "percent_gait"):
                gp = (sname, c)
                break
        if gp:
            break
    print(f"\n[5] GAIT PHASE 0-100%")
    if gp:
        print(f"    PRESENT as {gp[0]}/{gp[1]} -- use it directly; no heel-strike")
        print(f"    detector needed (the paper computed it by linear interpolation")
        print(f"    between heel strikes, heel strike = zero heel-marker velocity)")
    else:
        print("    NOT FOUND in the probed files. Reproduce the paper's procedure:")
        print("      heel strike = zero linear velocity of the heel marker (mocap),")
        print("      then linearly interpolate 0-100% between consecutive strikes.")
    rep["gait_phase"] = {"present": bool(gp), "location": gp}

    # ---- walking speed / condition labels
    print(f"\n[6] CONDITION / SPEED LABELS")
    cond = sensors.get("conditions") or sensors.get("condition")
    if cond:
        cf = sorted(os.listdir(cond))[:3]
        print(f"    conditions/ contains {len(os.listdir(cond))} file(s), e.g. {cf}")
        for f in cf:
            n, c, note = load_table(os.path.join(cond, f))
            print(f"      {f}: loader={note} columns={n[:12]}")
        print("    The paper reports self-selected speeds slow 0.88+/-0.19, "
              "normal 1.17+/-0.21, fast 1.45+/-0.27 m/s.")
        print("    Select the NORMAL-speed trials from these labels; if the label is")
        print("    categorical, cross-check against pelvis/treadmill speed if present.")
    else:
        print("    no conditions/ folder found in this mode directory")

    # ---- stride enumeration with moment coverage
    print(f"\n[7] STRIDE AVAILABILITY  (kinetics coverage is the binding constraint)")
    print("    The paper: GRF was recorded only during 'certain representative steps',")
    print("    and 'gait cycles without ground reaction force data were excluded from")
    print("    moment and power analyses'.  So filter to strides with a non-null knee")
    print("    moment before choosing one.")
    id_dir = sensors.get("id")
    ik_dir = sensors.get("ik")
    if id_dir and ik_dir:
        ik_files = sorted(f for f in os.listdir(ik_dir) if f.lower().endswith((".mat", ".csv")))
        id_files = sorted(f for f in os.listdir(id_dir) if f.lower().endswith((".mat", ".csv")))
        shared = sorted(set(ik_files) & set(id_files))
        print(f"    ik trials {len(ik_files)}, id trials {len(id_files)}, "
              f"same filename in both: {len(shared)}")
        rep["trials"] = {"ik": len(ik_files), "id": len(id_files),
                         "with_both": len(shared), "shared_examples": shared[:8]}
        for f in shared[:3]:
            n, c, _ = load_table(os.path.join(id_dir, f))
            mom = pick(n, "knee_moment")[:1]
            if mom:
                a = c[mom[0]]
                frac = float(np.mean(np.isfinite(a)))
                print(f"      {f}: {mom[0]} finite for {100 * frac:.1f}% of samples")
    else:
        print("    ik/ and/or id/ not found -- cannot assess moment coverage")

    return rep


# ======================================================================== main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="resolve the three DOIs")
    ap.add_argument("--probe", metavar="ROOT", default=None,
                    help="directory containing the unzipped dataset")
    ap.add_argument("--subject", default="AB19")
    ap.add_argument("--json", default=None, help="write the full report here")
    args = ap.parse_args()

    if not args.check and not args.probe:
        args.check = True

    print("=" * 96)
    print("CAMARGO ET AL. 2021 -- DATASET ACQUISITION AND VERIFICATION")
    print("One subject.  Level-ground, self-selected normal speed.  No averaging.")
    print("No figure digitisation: subject-level numerical data exist, and the")
    print("paper's Fig. 6 is a 22-subject average.")
    print("=" * 96)

    report = {}
    if args.check:
        print("\n[0] AVAILABILITY")
        report["availability"] = check_availability()
    if args.probe:
        report["probe"] = probe(args.probe, args.subject)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2, default=str)
        print(f"\nreport written to {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
