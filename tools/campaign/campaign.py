#!/usr/bin/env python3
"""Randomized correctness campaign driver (of-5rim, epic of-ipt).

Runs the repo's seeded randomized suites under a fresh OPENSOLID_CAMPAIGN_SEED,
collects failures, reconciles them against the open bead board, and files any
NEW failure as a bug bead carrying the seed and an exact repro command.

The suites stay byte-for-byte deterministic when the variable is unset (plain
`cargo test`, CI); the campaign is the only consumer of the remix hook.

Default mode is --dry-run: everything runs, reconciliation happens, and the
`bd create` invocations are printed instead of executed. Pass --file to file
for real (the intended cron mode, once the Mayor approves the wiring).

Exit status: 0 whenever the campaign itself ran to completion — found
failures are its product, not its error. Nonzero only for driver problems
(a suite that failed to compile, bd unreachable, bad arguments).
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import pathlib
import re
import secrets
import subprocess
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent

# (crate, test target, takes_seed). property_invariants runs on proptest,
# which draws fresh entropy every run on its own — it belongs to the campaign
# but ignores the seed variable.
SUITES = [
    ("opensolid-brep", "boolean_stress", True),
    ("opensolid-brep", "sweep_random", True),
    ("opensolid-frep", "ops_randomized", True),
    ("opensolid-frep", "property_invariants", False),
    ("opensolid-kernel", "convert_roundtrip_random", True),
    ("opensolid-kernel", "step_heal_random", True),
]

# A generated configuration the suite itself rejects (e.g. a non-transversal
# NURBS block pair) is a miss of the generator, not a kernel bug: report it,
# never file it.
GENERATOR_MISS_MARKERS = [
    "choose a different seed",
    "pick a different seed",
]

FAILED_LINE = re.compile(r"^test (\S+) \.\.\. FAILED$", re.M)
RESULT_LINE = re.compile(
    r"^test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored", re.M
)


def default_log_dir() -> pathlib.Path:
    state = os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state"))
    return pathlib.Path(state) / "opensolid-campaign"


def run_suite(crate: str, target: str, seed_hex: str, takes_seed: bool) -> dict:
    env = dict(os.environ)
    if takes_seed:
        env["OPENSOLID_CAMPAIGN_SEED"] = seed_hex
    else:
        env.pop("OPENSOLID_CAMPAIGN_SEED", None)
    cmd = ["cargo", "test", "-p", crate, "--test", target]
    start = time.monotonic()
    proc = subprocess.run(
        cmd, cwd=REPO_ROOT, env=env, capture_output=True, text=True
    )
    elapsed = time.monotonic() - start
    out = proc.stdout + "\n" + proc.stderr

    result = RESULT_LINE.search(proc.stdout)
    if result is None:
        # Never reached the harness: compile error or crash before any test.
        return {
            "crate": crate,
            "target": target,
            "seconds": round(elapsed, 1),
            "error": out[-4000:],
            "failures": [],
        }

    failures = []
    for name in FAILED_LINE.findall(proc.stdout):
        detail = extract_detail(proc.stdout, name)
        failures.append(
            {
                "test": name,
                "generator_miss": any(m in detail for m in GENERATOR_MISS_MARKERS),
                "detail": detail[-2000:],
            }
        )
    return {
        "crate": crate,
        "target": target,
        "seconds": round(elapsed, 1),
        "passed": int(result.group(2)),
        "failed": int(result.group(3)),
        "ignored": int(result.group(4)),
        "failures": failures,
    }


def extract_detail(stdout: str, test: str) -> str:
    """The `---- <test> stdout ----` block for a failed test."""
    marker = f"---- {test} stdout ----"
    start = stdout.find(marker)
    if start < 0:
        return ""
    end = stdout.find("\n---- ", start + len(marker))
    if end < 0:
        end = stdout.find("\nfailures:", start + len(marker))
    return stdout[start : end if end > 0 else None]


def bd(args: list[str]) -> str:
    proc = subprocess.run(
        ["bd", *args, "--no-pager"], capture_output=True, text=True, cwd=REPO_ROOT
    )
    if proc.returncode != 0:
        raise RuntimeError(f"bd {' '.join(args)} failed: {proc.stderr[-500:]}")
    return proc.stdout


def known_open_bead(target: str, test: str) -> str | None:
    """An open bead already covering this failing test, or None.

    Two probes: the campaign's own key (`<target>::<test>` in the
    description, which every bead this driver files carries) and the bare
    test name (catches hand-filed beads that quoted the test).
    """
    for needle in (f"{target}::{test}", test):
        listing = bd(
            ["list", "--status=open", "--flat", "-n", "0", "--desc-contains", needle]
        ).strip()
        for line in listing.splitlines():
            m = re.search(r"\b(of-[A-Za-z0-9.]+)\b", line)
            if m:
                return m.group(1)
    return None


def file_bead(
    crate: str, target: str, test: str, seed_hex: str, detail: str, dry_run: bool
) -> str:
    title = f"campaign: {target}::{test} fails under fresh seed"
    repro = (
        f"OPENSOLID_CAMPAIGN_SEED={seed_hex} "
        f"cargo test -p {crate} --test {target} {test}"
    )
    description = (
        f"Filed by the randomized correctness campaign (of-5rim engine, epic of-ipt).\n\n"
        f"Suite: {target}::{test}\n"
        f"Seed: {seed_hex}\n"
        f"Repro: {repro}\n\n"
        f"Failure output (tail):\n{detail}"
    )
    args = [
        "create",
        title,
        "--type=bug",
        "--priority=2",
        "-d",
        description,
    ]
    if dry_run:
        # Exercise the real filing path end-to-end; bd itself previews
        # instead of creating.
        args.append("--dry-run")
    out = bd(args)
    if dry_run:
        return "DRY-RUN (bd accepted)" if "[DRY RUN]" in out else f"DRY-RUN? {out[:80]}"
    m = re.search(r"(of-[A-Za-z0-9.]+)", out)
    return m.group(1) if m else out.strip()[:80]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run", action="store_true", default=True, help="default: do not file"
    )
    mode.add_argument(
        "--file", dest="dry_run", action="store_false", help="really file new beads"
    )
    ap.add_argument("--seed", help="hex seed (default: fresh 64-bit entropy)")
    ap.add_argument("--log-dir", type=pathlib.Path, default=default_log_dir())
    ap.add_argument(
        "--suites",
        nargs="*",
        help="restrict to these test targets (default: all)",
    )
    args = ap.parse_args()

    seed = int(args.seed.replace("0x", ""), 16) if args.seed else secrets.randbits(64)
    seed_hex = f"0x{seed:016X}"
    suites = [
        s for s in SUITES if not args.suites or s[1] in args.suites
    ]
    if not suites:
        print(f"no suite matches {args.suites}", file=sys.stderr)
        return 2

    stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
    print(f"campaign seed {seed_hex} — {len(suites)} suites, repo {REPO_ROOT}")

    report = {"seed": seed_hex, "started": stamp, "suites": [], "filed": [], "known": []}
    driver_error = False
    for crate, target, takes_seed in suites:
        print(f"  running {target} ...", flush=True)
        res = run_suite(crate, target, seed_hex, takes_seed)
        report["suites"].append(res)
        if "error" in res:
            driver_error = True
            print(f"    ERROR: suite did not run (see log)")
            continue
        misses = [f for f in res["failures"] if f["generator_miss"]]
        real = [f for f in res["failures"] if not f["generator_miss"]]
        print(
            f"    {res['passed']} passed, {res['failed']} failed"
            f" ({len(misses)} generator-miss), {res['ignored']} ignored,"
            f" {res['seconds']}s"
        )
        for f in real:
            bead = known_open_bead(target, f["test"])
            if bead:
                report["known"].append({"test": f"{target}::{f['test']}", "bead": bead})
                print(f"    KNOWN {f['test']} -> open bead {bead}, not filing")
            else:
                ref = file_bead(
                    crate, target, f["test"], seed_hex, f["detail"], args.dry_run
                )
                report["filed"].append({"test": f"{target}::{f['test']}", "bead": ref})
                print(f"    NEW   {f['test']} -> {ref}")

    total = sum(s["seconds"] for s in report["suites"])
    report["total_seconds"] = round(total, 1)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    log = args.log_dir / f"run-{stamp}-{seed_hex}.json"
    log.write_text(json.dumps(report, indent=2))
    print(
        f"campaign done: {total:.0f} total test-seconds,"
        f" {len(report['filed'])} new, {len(report['known'])} known -> {log}"
    )
    return 1 if driver_error else 0


if __name__ == "__main__":
    sys.exit(main())
