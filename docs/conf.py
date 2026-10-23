"""Sphinx configuration for the HISPEC tracking camera instrument documentation."""

import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
REPO_URL = "https://github.com/CaltechOpticalObservatories/hispec-tracking-camera-instrument"

project = "hispec-tracking-camera-instrument"
author = "Caltech Optical Observatories"
copyright = "Caltech Optical Observatories"

with (REPO_ROOT / "pyproject.toml").open("rb") as pyproject:
    release = tomllib.load(pyproject)["project"]["version"]
version = release

extensions = [
    "myst_parser",
    "sphinx.ext.extlinks",
    "sphinx_copybutton",
    "sphinx_design",
]

exclude_patterns = ["_build"]

# {source}`config/hispecatc.cfg` links a file here, {core}`configuration/` a page of the core docs
extlinks = {
    "source": (f"{REPO_URL}/blob/main/%s", "%s"),
    "core": ("https://caltechopticalobservatories.github.io/camera-interface/%s", None),
}

myst_enable_extensions = ["colon_fence", "deflist"]
myst_heading_anchors = 3

html_theme = "furo"
html_title = f"HISPEC tracking camera {release}"
html_theme_options = {
    "source_repository": f"{REPO_URL}/",
    "source_branch": "main",
    "source_directory": "docs/",
}
