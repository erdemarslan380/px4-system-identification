from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DOCS_ROOT = REPO_ROOT / "docs" / "sitl_validation"


def _copy_tree(src: Path, dst: Path) -> list[str]:
    if not src.exists():
        raise FileNotFoundError(src)
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    return sorted(
        str(path.relative_to(dst))
        for path in dst.rglob("*")
        if path.is_file()
    )


def publish_docs_assets(
    *,
    section: str,
    docs_root: Path = DEFAULT_DOCS_ROOT,
    figures_root: Path | None = None,
    review_root: Path | None = None,
    parameter_report_root: Path | None = None,
) -> dict[str, object]:
    if not section.strip():
        raise ValueError("section must not be empty")
    if figures_root is None and review_root is None and parameter_report_root is None:
        raise ValueError("at least one asset root must be provided")

    target_root = docs_root / section
    target_root.mkdir(parents=True, exist_ok=True)
    published: dict[str, object] = {
        "section": section,
        "docs_root": str(docs_root.resolve()),
        "target_root": str(target_root.resolve()),
    }

    if figures_root is not None:
        dst = target_root / "figures"
        published["figures_root"] = str(dst.resolve())
        published["figure_files"] = _copy_tree(figures_root, dst)

    if review_root is not None:
        dst = target_root / "review"
        published["review_root"] = str(dst.resolve())
        published["review_files"] = _copy_tree(review_root, dst)
        published["review_index"] = str((dst / "index.html").resolve())

    if parameter_report_root is not None:
        dst = target_root / "parameters"
        published["parameter_root"] = str(dst.resolve())
        published["parameter_files"] = _copy_tree(parameter_report_root, dst)

    manifest_path = target_root / "publish_manifest.json"
    manifest_path.write_text(json.dumps(published, indent=2), encoding="utf-8")
    published["manifest"] = str(manifest_path.resolve())
    return published


def main() -> int:
    parser = argparse.ArgumentParser(description="Copy generated SITL plots and HTML reviews into docs/sitl_validation.")
    parser.add_argument("--section", required=True, help="Subdirectory under docs/sitl_validation, e.g. stock or stock_vs_prior.")
    parser.add_argument("--docs-root", default=str(DEFAULT_DOCS_ROOT))
    parser.add_argument("--figures-root", default="")
    parser.add_argument("--review-root", default="")
    parser.add_argument("--parameter-report-root", default="")
    args = parser.parse_args()

    result = publish_docs_assets(
        section=args.section.strip(),
        docs_root=Path(args.docs_root).expanduser().resolve(),
        figures_root=Path(args.figures_root).expanduser().resolve() if args.figures_root else None,
        review_root=Path(args.review_root).expanduser().resolve() if args.review_root else None,
        parameter_report_root=Path(args.parameter_report_root).expanduser().resolve() if args.parameter_report_root else None,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
