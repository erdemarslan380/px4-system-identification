from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from experimental_validation.publish_sitl_docs_assets import publish_docs_assets


class PublishSitlDocsAssetsTests(unittest.TestCase):
    def test_publish_docs_assets_copies_figures_review_and_parameters(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            docs_root = root / "docs"
            figures_root = root / "figures"
            review_root = root / "review"
            parameter_root = root / "parameters"

            figures_root.mkdir()
            review_root.mkdir()
            parameter_root.mkdir()

            (figures_root / "group_1.png").write_text("png", encoding="utf-8")
            (figures_root / "comparison_summary.json").write_text("{}", encoding="utf-8")
            (review_root / "index.html").write_text("<html></html>", encoding="utf-8")
            (review_root / "summary.json").write_text("{}", encoding="utf-8")
            (parameter_root / "parameter_summary.md").write_text("# Table\n", encoding="utf-8")
            (parameter_root / "parameter_summary.json").write_text("{}", encoding="utf-8")

            result = publish_docs_assets(
                section="stock",
                docs_root=docs_root,
                figures_root=figures_root,
                review_root=review_root,
                parameter_report_root=parameter_root,
            )

            self.assertTrue((docs_root / "stock" / "figures" / "group_1.png").exists())
            self.assertTrue((docs_root / "stock" / "review" / "index.html").exists())
            self.assertTrue((docs_root / "stock" / "parameters" / "parameter_summary.md").exists())
            self.assertTrue((docs_root / "stock" / "publish_manifest.json").exists())
            self.assertEqual(Path(result["review_index"]).name, "index.html")

    def test_publish_docs_assets_requires_at_least_one_source(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                publish_docs_assets(section="stock", docs_root=Path(tmp))


if __name__ == "__main__":
    unittest.main()
