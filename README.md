# Introduction
This repo aims to render depth images with the help of open source 3D creation tool [Blender](https://www.blender.org/), and then generate point clouds from them. Codes are polished and parallelization is imposed to boost up scanning, which another Blender-based virtual scan project, [Blensor](https://www.blensor.org/), lacks. Codes are modified from https://github.com/wentaoyuan/it-net. The main difference is the supported Blender version and the dataset to be scanned. This script is compatible with v2.8x and there is no need to install Blender OFF Addon, since Blender has it integrated but not documented.

# Usage
## Dependencies
- Install [Blender](https://blender.org/download) v2.8x.
- Install all the dependencies listed in traj2pcd.py. This step can be skip if there is no need for point clouds.
## Depth images capturing
- Modify the intrinsics file according to your preference.
- `python parallel_scan.py --dataset_dir [dataset directory] --intrinsics [intrinsics file] --output_dir [images directory]  --num_process [paralleled process count]` Note that default camera trajectory consists of six views, staring at the model omnidirectionally, and covers the whole shape in most cases.
## Depth images to point clouds
- `python depth2pcd.py --intrinsics [intrinsics file] --input_dir [images directory] --output_dir [output directory] --num_scans [scan count]` Note that num_scans should be specified if you change the capturing trajectory, otherwise leave it as it is.

