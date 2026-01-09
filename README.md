# PRISM: Color-Stratified Point Cloud Sampling

**PRISM** is a novel color-guided stratified sampling method for RGB-LiDAR point clouds. Unlike conventional downsampling methods (Random, Voxel Grid, Farthest Point) that focus solely on spatial distribution, PRISM leverages photometric information to preserve texture-rich regions while efficiently reducing redundancy in visually homogeneous areas.

This repository contains the official implementation of the paper **"PRISM: Color-Stratified Point Cloud Sampling"**.

## Key Features

- **Color-Guided Sampling**: Allocates sampling density proportional to chromatic diversity.
- **Redundancy Suppression**: Aggressively downsamples spatially large but photometrically simple regions (e.g., walls, roads).
- **Feature Preservation**: Retains detailed structures like signage, vegetation, and architectural ornaments.
- **Predictable Output**: Controls output size via a single intuitive capacity parameter ($k$).

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/SUNY-MEC-MEIC-Lab/PRISM.git
   cd PRISM
   ```

2. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

## Usage

The core implementation is provided in `PRISM.py`.

### Basic Command

```bash
python PRISM.py --input path/to/input.ply --output path/to/output.ply --k 10
```

### Arguments

| Argument | Flag | Default | Description |
|----------|------|---------|-------------|
| **Input** | `-i`, `--input` | Required | Path to input `.ply` file or directory containing `.ply` files. |
| **Output** | `-o`, `--output` | Required | Path to output file or directory. |
| **Bin Capacity** | `-k` | `1` | Maximum number of points retained per unique color bin. Higher $k$ keeps more points. |
| **Quantization** | `-q`, `--quantization` | `1.0` | Color quantization step size. Larger values merge similar colors (more compression). |
| **No Chromaticity** | `--no_chromaticity` | `False` | If set, uses raw RGB values for binning. Default uses Chromaticity (normalized RGB) to be robust to illumination changes. |
