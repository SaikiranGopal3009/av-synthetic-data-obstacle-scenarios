# av-synthetic-data-obstacle-scenarios
Map-grounded synthetic data generation pipeline for AV obstacle perception.

## Project Description
This repository implements an end-to-end prototype for generating validated synthetic perception data generation simulating road obstacles. 
Instead of placing assets in arbitrary world coordinates, scenarios are authored in lane-relative Frenet coordinates `(lane_index, s, d)`, converted into 3D world poses, written to OpenUSD (`.usda`) stages with custom `sdg:*` semantic attributes, projected into 2D camera bounding boxes via a pinhole camera model, checked against geometric/label sanity rules, and exported in seeded batches with GIF previews.

## Problem Statement
Synthetic datasets used in simulation for training and evaluation of autonomous driving perception stack often break in silent ways: 
- obstacles spawn off-road or out of lane bounds, 
- 3D scene graphs drift out of sync with 2D ground-truth labels,
- random sweeps lack deterministic reproducibility. 

At the tooling level, heavy simulation engines can also be slow to iterate on, tightly coupled to proprietary scene formats or rendering pipelines, and awkward to test automatically in CI without GPU or full-engine overhead. 
This pipeline focuses on keeping the core map, OpenUSD contract, projection math, and validation checks lightweight, deterministic, and fast to test before scaling up to heavier rendering or full simulation engines.

## Layout
```
configs/
  workzone_demo.yaml   # road, obstacle, camera, and output settings
src/
  cli.py               # CLI entry point (single sample or seeded sweep)
  map_loader.py        # RoadSpec: Frenet (lane, s, d) to world (x, y, z)
  usd_builder.py       # OpenUSD (.usda) stage authoring with sdg:* attributes
  renderer.py          # top-down PNG and pinhole camera 2D bbox PNG
  validator.py         # geometry, camera, and 2D bounding-box checks
  sweep.py             # seeded batch runner, manifest.json, and GIF export
tests/
  test_smoke.py        # end-to-end smoke tests
```

## Setup

```
python3 -m venv .venv
source .venv/bin/activate
python -m pip install pyyaml numpy matplotlib pillow pytest
```

## Usage

Generate a single scenario:
```
python -m src.cli --config configs/workzone_demo.yaml
```

Writes `scenario_000.usda`, `top_down_000.png`, and `camera_000.png` to `outputs/samples/` and prints the projected 2D bounding box in pixels.

Run a seeded multi-sample sweep:
```
python -m src.cli --config configs/workzone_demo.yaml --num-samples 8 --seed 42
```
Generates `N` validated samples along with `manifest.json`, `sweep_camera.gif`, and `sweep_top_down.gif`.

## Tests
```
python -m pytest -q
```

## Planned Features

- Local Y-axis pitch/tilt variations around the nominal obstacle location in OpenUSD transforms, 3D-to-2D bounding-box projection, and sweep manifests
- Ground-contact height adjustment when pitching obstacles so tilted corners do not clip into the road surface
- Curved road centerline support in `RoadSpec` beyond straight multi-lane corridors
- Multi-obstacle scenarios with inter-object collision and occlusion checks in `validator.py`
