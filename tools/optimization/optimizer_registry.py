#!/usr/bin/env python3
"""Shared optimizer metadata for tuning plans, reports, and dashboard UI."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class OptimizerSpec:
    name: str
    display_name: str
    short_label: str
    color: str
    description: str
    global_method: str
    local_method: str


_LOCAL_METHOD = "Parallel central-difference gradient descent"

OPTIMIZERS: dict[str, OptimizerSpec] = {
    "bayes": OptimizerSpec(
        name="bayes",
        display_name="Bayesian + Central Diff",
        short_label="Bayes",
        color="#ef476f",
        description="Asynchronous GP + expected improvement global search with central-difference local refinement.",
        global_method="Asynchronous parallel Bayesian optimization (GP + EI, minimization)",
        local_method=_LOCAL_METHOD,
    ),
    "random": OptimizerSpec(
        name="random",
        display_name="Random + Central Diff",
        short_label="Random",
        color="#118ab2",
        description="Asynchronous uniform random global exploration with central-difference local refinement.",
        global_method="Asynchronous parallel random search",
        local_method=_LOCAL_METHOD,
    ),
    "anneal": OptimizerSpec(
        name="anneal",
        display_name="Annealed + Central Diff",
        short_label="Anneal",
        color="#06d6a0",
        description="Asynchronous annealed neighborhood search around the incumbent with central-difference local refinement.",
        global_method="Asynchronous parallel annealed neighborhood search",
        local_method=_LOCAL_METHOD,
    ),
}


def normalize_optimizer_name(value: str | None) -> str:
    raw = str(value or "bayes").strip().lower()
    return raw if raw in OPTIMIZERS else "bayes"


def get_optimizer_spec(value: str | None) -> OptimizerSpec:
    return OPTIMIZERS[normalize_optimizer_name(value)]


def list_optimizer_names() -> tuple[str, ...]:
    return tuple(OPTIMIZERS.keys())
