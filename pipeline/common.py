"""Shared helpers: repository paths and configuration loading."""
from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"
RESULTS_DIR = ROOT / "results"
FIGURES_DIR = ROOT / "paper" / "figures"
TABLES_DIR = ROOT / "paper" / "tables"


def load_yaml(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_sequences() -> dict:
    return load_yaml(CONFIG_DIR / "sequences.yaml")


def load_systems(include_extras: bool = True) -> list[dict]:
    """All registered systems from team.yaml (entries without a slug are skipped)."""
    team = load_yaml(ROOT / "team.yaml")
    entries = list(team.get("members", []))
    if include_extras:
        entries += team.get("extras", []) or []
    return [e for e in entries if e.get("slug")]


def load_system_config(slug: str) -> dict:
    return load_yaml(CONFIG_DIR / "systems" / f"{slug}.yaml")


def run_dir(slug: str, sequence: str, run_idx: int) -> Path:
    return RESULTS_DIR / slug / sequence / f"run_{run_idx:02d}"
