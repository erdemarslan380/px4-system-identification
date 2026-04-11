#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.reference_models import (  # noqa: E402
    default_candidate_identified,
    default_jmavsim_prior_candidate,
    default_x500_reference,
)
from experimental_validation.twin_metrics import (  # noqa: E402
    build_blended_twin_score_from_values,
    flatten_identified_metrics,
    flatten_reference_metrics,
)


def _reference_payload(reference_name: str) -> tuple[dict[str, float], str]:
    if reference_name == "jmavsim_prior":
        return flatten_identified_metrics(default_jmavsim_prior_candidate()), "jmavsim_prior"
    if reference_name == "x500_reference":
        return flatten_reference_metrics(default_x500_reference()), "x500_reference"
    if reference_name == "x500_candidate":
        return flatten_identified_metrics(default_candidate_identified()), "x500_candidate"
    raise ValueError(f"unsupported reference model: {reference_name}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Compare identified multicopter parameters against a named reference model.")
    ap.add_argument("--identified-json", required=True, help="Path to identified_parameters.json.")
    ap.add_argument(
        "--reference-model",
        required=True,
        choices=("jmavsim_prior", "x500_reference", "x500_candidate"),
        help="Named local reference model.",
    )
    ap.add_argument("--out", default="", help="Optional JSON output path.")
    args = ap.parse_args()

    identified_json = Path(args.identified_json).expanduser().resolve()
    identified = json.loads(identified_json.read_text(encoding="utf-8"))
    candidate = flatten_identified_metrics(identified)
    reference, reference_label = _reference_payload(args.reference_model)
    report = build_blended_twin_score_from_values(candidate, reference)
    payload = {
        "identified_json": str(identified_json),
        "reference_model": reference_label,
        "score": report["score"],
        "normalized_penalty": report["normalized_penalty"],
        "family_scores": report["family_scores"],
        "comparable_metrics": report["comparable_metrics"],
    }

    if args.out:
        out_path = Path(args.out).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
