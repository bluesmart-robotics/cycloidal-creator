# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

cycloidgen sizes a cycloidal drive, checks it (Hertz contact, stiffness, bearings, fatigue, thermal)
and exports DXF, SVG, STEP, STL, 3MF, a BOM, JSON and a PDF report, from a PySide6 window or the
CLI. The README holds the equations, the checks and the model's limits: read it, do not re-derive.
CONTRIBUTING.md (layout, style) and RELEASING.md (version policy, release steps) are current.

## Commands

```powershell
py -3.12 -m venv .venv; .venv\Scripts\python -m pip install -e ".[dev]"   # brings ruff too
.venv\Scripts\python -m pytest -q               # ~1400 tests, ~19 min; iterate on one file
.venv\Scripts\python -m ruff check .            # the lint gate; rules in pyproject.toml
.venv\Scripts\python -m cycloidgen              # the GUI (run.bat: the same, via pythonw)
.venv\Scripts\python -m cycloidgen --ratio 29 --list-outputs
.venv\Scripts\python -m cycloidgen --ratio 29 --only drawings,data --out <dir>  # no CAD kernel
.venv\Scripts\python docs\make_figures.py       # README figures (matplotlib, runs anywhere)
.venv\Scripts\python tools\make_screenshots.py  # README screenshots: a real window, Windows only
```

Run the whole suite and ruff before every commit. CI (`tests.yml`) adds Linux and macOS, Python
3.10–3.14, an NSIS compile against a stub, and a headless export plus a design search.

**Never run `ruff format`.** The code is hand-aligned (95 columns, continuation lines under their
openers, tables laid out as tables); the formatter would rewrite dozens of files. Match the file.

## Rules that have already cost real time

**Never change the profile equations without the envelope test.** The psi form needs a LEADING
MINUS: `psi(t) = -atan2(sin(N*t), R/(E*Np) - cos(N*t))`. The plus-sign version circulates widely
and looks perfect on screen, but interferes with the ring pins by about a millimetre: the drive
seizes. Only the envelope property (every profile point exactly `Rr` from the pin-centre locus)
catches it, and `tests/test_profile.py` pins it and the trap. Check a formula from any outside
source, another agent included, against an independent property of the geometry, never against a
second statement of the same formula.

**A new number gets a second way of computing it**, in a test. And a proven part does not prove
the assembly: the profile and kinematics were once exact to 0.0000 µm while every multi-disc
drive was unbuildable (the output pins could not enter the second disc's holes). Check the seams.

**A change that moves a computed number is a changelog event.** The version lives only in
`cycloidgen/__init__.py`, a one-line literal that setuptools and NSIS read as text, and
`tests/test_version.py` fails without its `## x.y.z` in CHANGELOG.md, whose **Numbers** section
says which quantity moved, on which design, by how much (RELEASING.md).

- Qt lives in `ui/` only; `core/`, `viz/`, `analysis/`, `design/`, `export/`, `report/` never
  import it, and importing `cycloidgen.analysis` must not even load matplotlib.
- `export/manifest.py` declares every output file once; the writer, the Outputs tab,
  `--list-outputs` and the README table all read it. Add a file there first.
- Every `python` block in `docs/api.md` is executed by `tests/test_api_docs.py`.
- A test that touches QSettings isolates them first (`isolated_settings` in
  `tests/test_workspace.py`): Qt ignores `setDefaultFormat` on Windows, so without
  `CYCLOIDGEN_SETTINGS` the test rewrites the developer's real preferences.
- The `PySide6<6.12` (Python < 3.12) and `casadi<3.8` (Windows) ceilings in `pyproject.toml`
  are measured native crashes; the comments there say when each may be lifted.

## Verifying a UI change

Offscreen Qt has no fonts (labels render as tofu) and the 3D viewport is a native GL surface, so
judge a UI change from a real window. Launch a second instance with `CYCLOIDGEN_SETTINGS` at a
scratch `.ini` seeded with `[General]` / `update_check=off`: an empty file is a first run, which
pops the update-consent dialog, and a modal left open makes later scripted clicks crash. Capture
it with Win32 `PrintWindow(hwnd, hdc, 2)` as `tools/make_screenshots.py` does (a foreground switch
plus screen grab photographs whatever is on top) and close it afterwards. For VTK picking, click
with `QTest.mouseClick(view._widget, ...)`; calling `LeftButtonPressEvent` directly crashes.

## Git and releases

- Commit straight to `main`: no feature branches, no pull requests. Answer the PR template's
  question (does this move a number somebody already built from?) in the commit message. Push
  only when asked. `main` rejects force-pushes, so never amend or rewrite a pushed commit.
- Subjects are one plain sentence about the behaviour ("A motor shaft too thick to bore for stops
  the export"); the body says why. No `Co-Authored-By` trailer or other AI attribution.
- `ROADMAP.md` is private and gitignored: never commit or force-add it, and never quote it in
  code, the changelog, the README, commit messages or issues.
- Release: bump, changelog section, ruff + pytest, tag `vX.Y.Z`, push `--follow-tags`;
  `release.yml` does the rest. **Known gap:** the GitHub release and PyPI wait only on the
  Windows job, which asks the frozen build for `--version` and `--list-outputs` but never runs an
  export. In that job a gate command gets a step of its own: pwsh carries on past a failing
  command, so a second line in the same step decides its exit code
  (`test_the_release_lint_can_fail_the_release`). Before tagging a release that adds an import or
  dependency path, run `python -m PyInstaller cycloidgen.spec --distpath <tmp>
  --workpath <tmp>`, then `<tmp>\cycloidgen\cycloidgen-cli.exe --ratio 21 --only drawings,data --out <dir>`.
- A Linux-only CI crash (bare exit code, a different test each run) is reproduced in Docker, not
  by re-pushing: `python:3.X-bookworm` + the apt list from `tests.yml` + `gdb`, `pip install -e
  ".[dev]"`, the CI env vars, `gdb -batch -ex run -ex 'bt 25' -ex py-bt --args python -m pytest
  ...`, fed a tar of `git ls-files -co --exclude-standard` (`tar --force-local` in Git Bash). A
  crash that moves between runs is refcount or heap damage; that is how the PySide6 pin was found.
