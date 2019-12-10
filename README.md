# Neural Point-Based Graphics
This repository provides evaluation dataset used in [Neural Point-Based Graphics](https://arxiv.org/pdf/1906.08240.pdf). We provide training data, test trajectories and our results.

# Dataset

## ScanNet
Users must agree to the terms of use to download any part of the ScanNet dataset. We provide a script to build point clouds from the raw data.

- Go to https://github.com/ScanNet/ScanNet to obtain ScanNet dataset.
- Download scene0000_00 and scene0024_00
- Export color, depth, poses and intrinsics from *.sens files with this code https://github.com/ScanNet/ScanNet/tree/master/SensReader/python
- Build point clouds:

Install Python requirements to build point clouds:

```bash
virtualenv --python=python3.6 venv
source venv/bin/activate
pip install -r requirements.txt
```

Run scripts on scene folders where you extracted the data:

```bash
python build_pointcloud.py --input /path/to/scene0000_00
python build_pointcloud.py --input /path/to/scene0024_00
```

Script requires the following folder structure:
```
scene
- pose
- color
- depth
- intrinsic
```

It will output paths to created \*.ply files.

## Photogrammetry
Download Plant and Shoe scenes from [here](https://yadi.sk/d/TBCt28HKwi46yg). These scenes where built using Samsung S10 camera and Agisoft Metashape.

Folder structure:
```
scene
- color - N RGB frames
- pointcloud.ply - point cloud
- mesh.ply - mesh
- proj_matrix.txt - OpenGL projection matrix
- view_matrix.txt - N stacked 4x4 train view matrices
```

# Results
Download results from [here](https://yadi.sk/d/IdoFue9FTsGcUQ).

Folder structure:
```
scene
- test_traj.txt - 4x4 test view matrices
- test_traj.mp4 - our results*
```

\* We use "Ours-full" method from the paper

# Citation
```
@misc{aliev2019neural,
    title={Neural Point-Based Graphics},
    author={Kara-Ali Aliev and Dmitry Ulyanov and Victor Lempitsky},
    year={2019},
    eprint={1906.08240},
    archivePrefix={arXiv},
    primaryClass={cs.CV}
}
```

# License
The data is released under the CC BY-SA 3.0 license, and the code is released under the MIT license.
