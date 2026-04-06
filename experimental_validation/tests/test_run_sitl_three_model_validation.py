from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import experimental_validation.run_sitl_three_model_validation as matrix


class RunSitlThreeModelValidationTests(unittest.TestCase):
    def test_orchestrator_wires_three_model_outputs_into_figures_and_summary(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            px4_root = root / "px4"
            px4_root.mkdir()
            prior_candidate_dir = root / "prior_candidate"
            prior_candidate_dir.mkdir()

            collect_side_effect = [
                {"dataset_root": str((root / "stock_dataset" / "stock_sitl_placeholder").resolve())},
                {"dataset_root": str((root / "jmavsim_prior_dataset" / "jmavsim_prior_sitl").resolve())},
                {"dataset_root": str((root / "reidentified_dataset" / "reidentified_sitl").resolve())},
            ]

            with mock.patch.object(matrix, "_collect_or_reuse_dataset", side_effect=collect_side_effect) as collect_mock:
                with mock.patch.object(
                    matrix,
                    "prepare_identified_model",
                    side_effect=[
                        {
                            "model_dir": str((root / "prepared_prior_model").resolve()),
                            "base_model_dir": str((root / "prepared_prior_base").resolve()),
                        },
                        {
                            "model_dir": str((root / "prepared_reidentified_model").resolve()),
                            "base_model_dir": str((root / "prepared_reidentified_base").resolve()),
                        },
                    ],
                ) as prepare_mock:
                    with mock.patch.object(matrix, "_run_or_reuse_ident", return_value={"results": [1]}) as ident_mock:
                        with mock.patch.object(
                            matrix,
                            "_build_reidentified_candidate",
                            return_value={
                                "out_dir": str((root / "reidentified_candidate_from_sitl_ident").resolve()),
                                "primary_mode": "truth_assisted",
                            },
                        ) as candidate_mock:
                            with mock.patch.object(
                                matrix,
                                "build_comparison_figures",
                                return_value={"figures": {"group_1": {"png": "fake.png"}}},
                            ) as figures_mock:
                                summary = matrix.run_three_model_validation(
                                    px4_root=px4_root,
                                    out_root=root,
                                    prior_candidate_dir=prior_candidate_dir,
                                    force=False,
                                )

            self.assertEqual(collect_mock.call_count, 3)
            self.assertEqual(prepare_mock.call_count, 2)
            ident_mock.assert_called_once()
            candidate_mock.assert_called_once()
            figures_mock.assert_called_once()

            _, kwargs = figures_mock.call_args
            self.assertEqual(kwargs["stock_root"], root / "stock_dataset" / "stock_sitl_placeholder")
            self.assertEqual(kwargs["compare_root"], root / "jmavsim_prior_dataset" / "jmavsim_prior_sitl")
            self.assertEqual(kwargs["compare_root_2"], root / "reidentified_dataset" / "reidentified_sitl")
            self.assertEqual(kwargs["compare_label"], "jMAVSim prior SDF")
            self.assertEqual(kwargs["compare_label_2"], "Re-identified from SITL ident")

            summary_path = root / "three_model_validation_summary.json"
            self.assertTrue(summary_path.exists())
            loaded = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(loaded["stock_dataset_root"], str((root / "stock_dataset" / "stock_sitl_placeholder").resolve()))
            self.assertEqual(summary["summary_json"], str(summary_path.resolve()))


if __name__ == "__main__":
    unittest.main()
