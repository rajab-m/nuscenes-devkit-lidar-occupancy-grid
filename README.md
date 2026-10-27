# LiDAR Occupancy Grid Mapping

A Python project for generating 2D occupancy grids from LiDAR data in the [nuScenes dataset](https://www.nuscenes.org/nuscenes).

## Features

- Load LiDAR point clouds using the nuScenes devkit.
- Transform points into world coordinates.
- Filter point clouds using height and distance thresholds.
- Build occupancy grids using log-odds updates.
- Use Bresenham's algorithm for ray tracing.
- Visualize the resulting maps with Matplotlib.

## Requirements

```bash
pip install nuscenes-devkit numpy matplotlib pyquaternion bresenham
```

## Dataset Setup

Download the nuScenes dataset and set the dataset path in your configuration:

```python
from nuscenes.nuscenes import NuScenes

nusc = NuScenes(
    version="v1.0-mini",
    dataroot="/path/to/nuscenes",
    verbose=True
)
```

## Usage

Configure the scene and processing mode in the main script:

```python
SCENE_NUMBER = 1
SAMPLE_NUMBER = 0
MODE = "scene"  # "scene" or "sample"
```

Run the Notebook:

```bash
occupancy_grid_lidar.ipynp
```

The occupancy grid is displayed using Matplotlib.

## Notes

The grid resolution, filtering thresholds, and sensor model probabilities can be adjusted in the code. Ground removal currently uses simple threshold filtering and may require improvement for more accurate maps.

## References

- [nuScenes Devkit](https://github.com/nutonomy/nuscenes-devkit)
- [Bresenham's Line Algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)


![occupancy](https://user-images.githubusercontent.com/56690379/160300891-6601bf59-2c05-455a-85ad-ad795d212084.jpg)
