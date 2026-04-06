from __future__ import annotations

import inspect
import unittest

import experimental_validation.run_sitl_ident_suite as ident_suite


class RunSitlIdentSuiteTests(unittest.TestCase):
    def test_cli_forwards_to_generic_ident_runner(self) -> None:
        source = inspect.getsource(ident_suite.main)
        self.assertIn("run_identification_with_assets(", source)
        self.assertIn('ap.add_argument("--source-model-dir"', source)
        self.assertIn('ap.add_argument("--model-name"', source)
        self.assertIn("headless=not args.visual", source)


if __name__ == "__main__":
    unittest.main()
