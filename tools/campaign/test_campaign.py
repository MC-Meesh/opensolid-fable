#!/usr/bin/env python3
"""Unit tests for the campaign driver (of-6662).

Run: python3 tools/campaign/test_campaign.py
"""

import importlib.util
import json
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

_HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("campaign", _HERE / "campaign.py")
campaign = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(campaign)


def _suite_result(crate, target, failures):
    return {
        "crate": crate,
        "target": target,
        "seconds": 1.0,
        "passed": 10,
        "failed": len(failures),
        "ignored": 0,
        "failures": [
            {"test": name, "generator_miss": False, "detail": f"panic in {name}"}
            for name in failures
        ],
    }


class BdInvocationTest(unittest.TestCase):
    def test_bd_does_not_pass_no_pager(self):
        """bd 1.1.2 rejects --no-pager; the wrapper must not send it (of-6662)."""
        with mock.patch.object(campaign.subprocess, "run") as run:
            run.return_value = mock.Mock(returncode=0, stdout="ok", stderr="")
            out = campaign.bd(["list", "--status=open"])
        self.assertEqual(out, "ok")
        cmd = run.call_args.args[0]
        self.assertEqual(cmd, ["bd", "list", "--status=open"])
        self.assertNotIn("--no-pager", cmd)

    def test_bd_failure_raises(self):
        with mock.patch.object(campaign.subprocess, "run") as run:
            run.return_value = mock.Mock(returncode=1, stdout="", stderr="boom")
            with self.assertRaises(RuntimeError):
                campaign.bd(["create", "x"])


class SpoolFindingTest(unittest.TestCase):
    def test_spool_appends_recoverable_jsonl(self):
        with tempfile.TemporaryDirectory() as tmp:
            log_dir = pathlib.Path(tmp) / "state"
            for test in ("t_one", "t_two"):
                spool = campaign.spool_finding(
                    log_dir,
                    "20260805T180000",
                    "0xDEADBEEF00000001",
                    "opensolid-brep",
                    "boolean_stress",
                    test,
                    "assertion failed: volume mismatch",
                    "bd create failed: unknown flag",
                )
            lines = spool.read_text().splitlines()
            self.assertEqual(len(lines), 2)
            entry = json.loads(lines[0])
            self.assertEqual(entry["test"], "t_one")
            self.assertEqual(entry["seed"], "0xDEADBEEF00000001")
            self.assertIn(
                "OPENSOLID_CAMPAIGN_SEED=0xDEADBEEF00000001 "
                "cargo test -p opensolid-brep --test boolean_stress t_one",
                entry["repro"],
            )
            self.assertIn("volume mismatch", entry["detail"])
            self.assertIn("unknown flag", entry["error"])


class FilingFailureIsNonFatalTest(unittest.TestCase):
    def test_bd_outage_spools_finding_and_runs_remaining_suites(self):
        """The Aug 4 failure mode: bd rejects the create, and previously the
        run aborted with 5 of 6 suites unrun and the finding lost."""
        results = {
            "boolean_stress": _suite_result(
                "opensolid-brep", "boolean_stress", ["t_bool"]
            ),
            "sweep_random": _suite_result("opensolid-brep", "sweep_random", []),
        }

        def fake_run_suite(crate, target, seed_hex, takes_seed):
            return results[target]

        with tempfile.TemporaryDirectory() as tmp:
            argv = [
                "campaign.py",
                "--file",
                "--seed",
                "0xDEAD",
                "--log-dir",
                tmp,
                "--suites",
                "boolean_stress",
                "sweep_random",
            ]
            with (
                mock.patch.object(campaign, "run_suite", side_effect=fake_run_suite) as rs,
                mock.patch.object(campaign, "known_open_bead", return_value=None),
                mock.patch.object(
                    campaign,
                    "file_bead",
                    side_effect=RuntimeError("bd create failed: unknown flag: --no-pager"),
                ),
                mock.patch.object(sys, "argv", argv),
            ):
                rc = campaign.main()

            # bd trouble is still a driver error (nonzero) ...
            self.assertEqual(rc, 1)
            # ... but every suite ran anyway,
            self.assertEqual(rs.call_count, 2)
            # the finding survived in the report and the spool,
            report = json.loads(
                next(pathlib.Path(tmp).glob("run-*.json")).read_text()
            )
            self.assertEqual(len(report["spooled"]), 1)
            self.assertEqual(report["spooled"][0]["test"], "boolean_stress::t_bool")
            spool = json.loads(
                next(pathlib.Path(tmp).glob("spool-*.jsonl")).read_text()
            )
            self.assertEqual(spool["test"], "t_bool")
            # and nothing was falsely reported as filed.
            self.assertEqual(report["filed"], [])


if __name__ == "__main__":
    unittest.main()
