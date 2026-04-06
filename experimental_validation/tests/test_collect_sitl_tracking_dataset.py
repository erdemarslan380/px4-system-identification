from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import experimental_validation.collect_sitl_tracking_dataset as collector
from experimental_validation.run_sitl_validation import DEFAULT_MODEL_SPECS, DEFAULT_VALIDATION_TRAJECTORIES


class CollectSitlTrackingDatasetTests(unittest.TestCase):
    def test_resolve_model_spec_arg_supports_custom_labels(self) -> None:
        built_in = collector.resolve_model_spec_arg(model_label=DEFAULT_MODEL_SPECS[0].label)
        self.assertEqual(built_in, DEFAULT_MODEL_SPECS[0])

        custom = collector.resolve_model_spec_arg(
            model_label="jmavsim_prior_sitl",
            gz_model="x500_ident_matrix_prior",
            display_name="jMAVSim prior SDF",
        )
        self.assertEqual(custom.label, "jmavsim_prior_sitl")
        self.assertEqual(custom.gz_model, "x500_ident_matrix_prior")
        self.assertEqual(custom.display_name, "jMAVSim prior SDF")

        with self.assertRaises(ValueError):
            collector.resolve_model_spec_arg(model_label="custom_only")

    def test_collect_tracking_dataset_salvages_latest_runtime_log_on_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            px4_root = root / "px4"
            px4_root.mkdir()
            out_root = root / "out"
            candidate_dir = root / "candidate"
            candidate_dir.mkdir()

            def _fake_run_validation_model(**kwargs):
                case_root = Path(kwargs["out_root"])
                model_spec = kwargs["model_spec"]
                run_rootfs = case_root / "runtime" / model_spec.label / "rootfs" / "tracking_logs"
                run_rootfs.mkdir(parents=True, exist_ok=True)
                (run_rootfs / "auto_latest.csv").write_text("timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n", encoding="utf-8")
                raise RuntimeError("gated after log creation")

            with mock.patch.object(collector, "run_validation_model", side_effect=_fake_run_validation_model):
                summary = collector.collect_tracking_dataset(
                    px4_root=px4_root,
                    out_root=out_root,
                    candidate_dir=candidate_dir,
                    model_spec=DEFAULT_MODEL_SPECS[0],
                    trajectories=(DEFAULT_VALIDATION_TRAJECTORIES[0],),
                )

            case = summary["cases"][0]
            self.assertFalse(case["ok"])
            self.assertIn("gated after log creation", case["error"])
            copied = Path(case["tracking_log"])
            self.assertTrue(copied.exists())
            self.assertEqual(copied.name, f"{DEFAULT_VALIDATION_TRAJECTORIES[0].name}.csv")
            manifest = json.loads((out_root / DEFAULT_MODEL_SPECS[0].label / "collection_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["cases"][0]["name"], DEFAULT_VALIDATION_TRAJECTORIES[0].name)

    def test_collect_tracking_dataset_prefers_saved_case_log_on_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            px4_root = root / "px4"
            px4_root.mkdir()
            out_root = root / "out"
            candidate_dir = root / "candidate"
            candidate_dir.mkdir()

            def _fake_run_validation_model(**kwargs):
                case_root = Path(kwargs["out_root"])
                model_spec = kwargs["model_spec"]
                saved = case_root / model_spec.label / "tracking_logs"
                saved.mkdir(parents=True, exist_ok=True)
                (saved / f"{DEFAULT_VALIDATION_TRAJECTORIES[0].name}.csv").write_text(
                    "timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n",
                    encoding="utf-8",
                )
                runtime = case_root / "runtime" / model_spec.label / "rootfs" / "tracking_logs"
                runtime.mkdir(parents=True, exist_ok=True)
                (runtime / "auto_latest.csv").write_text(
                    "timestamp_us,ref_x,ref_y,ref_z,pos_x,pos_y,pos_z,controller\n",
                    encoding="utf-8",
                )
                raise RuntimeError("gated after saved copy")

            with mock.patch.object(collector, "run_validation_model", side_effect=_fake_run_validation_model):
                summary = collector.collect_tracking_dataset(
                    px4_root=px4_root,
                    out_root=out_root,
                    candidate_dir=candidate_dir,
                    model_spec=DEFAULT_MODEL_SPECS[0],
                    trajectories=(DEFAULT_VALIDATION_TRAJECTORIES[0],),
                )

            case = summary["cases"][0]
            copied = Path(case["tracking_log"])
            self.assertTrue(copied.exists())
            self.assertEqual(copied.name, f"{DEFAULT_VALIDATION_TRAJECTORIES[0].name}.csv")


if __name__ == "__main__":
    unittest.main()
