import Imath
import OpenEXR
import argparse
import array
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import h5py
from PIL import Image

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)

def read_exr(exr_path, width, height):
    file = OpenEXR.InputFile(exr_path)
    depth_arr = array.array('f', file.channel('R', Imath.PixelType(Imath.PixelType.FLOAT)))
    depth = np.array(depth_arr).reshape((height, width))
    return depth

def depth2pcd(depth, intrinsics, pose=None):
    # Camera coordinate system in Blender is x: right, y: up, z: inwards
    inv_K = np.linalg.inv(intrinsics)
    inv_K[2, 2] = -1
    depth = np.flipud(depth)
    y, x = np.where(depth > 0)
    points = np.dot(inv_K, np.stack([x, y, np.ones_like(x)] * depth[y, x], 0))
    if pose is not None:
        # print("pose:", pose)
        points = np.dot(pose, np.concatenate([points, np.ones((1, points.shape[1]))], 0))[:3, :]
    return points.T

def colored_depth2pcd(depth, intrinsics, pose=None, colors=None):
    # Camera coordinate system in Blender is x: right, y: up, z: inwards
    points = depth2pcd(depth, intrinsics, pose)

    depth = np.flipud(depth)
    y, x = np.where(depth > 0)
    if colors is not None:
        colors = np.flipud(colors)
        points = np.concatenate([points, colors[y, x, :3]], axis=1)
    else:
        print("Color not provided. Returned points lost 3 channels.")
    return points

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--intrinsics', type=str, default='intrinsics.txt')
    parser.add_argument('--input_dir', type=str, default='images')
    parser.add_argument('--output_dir', type=str, default='results')
    parser.add_argument('--num_scans', type=int, default=6, help='number of scans')
    parser.add_argument('--has_color', action='store_true', default=False)
    args = parser.parse_args()

    args.output_dir = os.path.join(ROOT_DIR, args.output_dir)

    input_dir = os.path.join(ROOT_DIR, args.input_dir)
    category_list = [i for i in os.listdir(input_dir)
                     if os.path.isdir(os.path.join(input_dir, i))]

    intrinsics = np.loadtxt(args.intrinsics)
    width = int(intrinsics[0, 2] * 2)
    height = int(intrinsics[1, 2] * 2)
    num_scans = args.num_scans  # Fixed for six views

    # Preset pose setting
    preset_poses = []  # Fixed for six views
    dist_to_origin = 1
    rot_axis_list = ['Y', 'X', 'X']
    rot_angle_list = [[np.pi / 2.0, -np.pi / 2.0], [-np.pi / 2.0, np.pi / 2.0], [0, np.pi]]
    for i in range(int(num_scans / 2)):
        rot_axis = rot_axis_list[i]
        for j in range(2):
            dist_factor = 1 if j == 0 else -1

            rotation = R.from_euler(rot_axis, rot_angle_list[i][j]).as_dcm()
            translation = [0, 0, 0]
            translation[i] = dist_to_origin * dist_factor
            translation = np.transpose([translation])
            pose = np.concatenate((rotation, translation), axis=1)  # matrix (R|t)
            pose = np.concatenate((pose, [[0, 0, 0, 1.0]]),
                                       axis=0)  # Add extra row to amplify extrinsic format
            preset_poses.append(pose)

    for category in category_list:
        category_dir = os.path.join(ROOT_DIR, input_dir, category)
        model_list = [i for i in os.listdir(category_dir)
                           if os.path.isdir(os.path.join(category_dir, i))]
        for model_id in model_list:
            in_dir = os.path.join(args.input_dir, category, model_id)
            out_dir = os.path.join(args.output_dir, category)
            # num_scans = np.loadtxt(os.path.join(in_dir, 'num_scans.txt'), dtype=np.int)
            os.makedirs(out_dir, exist_ok=True)

            model_not_scanned = False
            points = []
            for i in range(num_scans):
                exr_path = os.path.join(in_dir, '%d.exr' % i)
                if not os.path.exists(exr_path): # Miss the scanning file, ignore this model & record model id
                    model_not_scanned = True
                    with open(os.path.join(ROOT_DIR, 'missing_scan.txt'), 'a') as missing_scan:
                        missing_scan.writelines(in_dir)
                    break

                # Set uncaught depths to zeros
                depth = read_exr(exr_path, width, height)
                depth[np.isinf(depth)] = 0

                # Calculate point coordinates and optional colors
                pose = preset_poses[i]
                if not args.has_color:
                    if i == 0:
                        points = depth2pcd(depth, intrinsics, pose)
                    else:
                        points = np.concatenate([points, depth2pcd(depth, intrinsics, pose)], axis=0)
                else:  # Add additional three channels if color is required
                    color_path = os.path.join(in_dir, '%d.png' % i)
                    if not os.path.exists(color_path) and not model_not_scanned:  # Miss the scanning file, ignore this model & record model id
                        model_not_scanned = True
                        with open(os.path.join(ROOT_DIR, 'missing_scan.txt'), 'a') as missing_scan:
                            missing_scan.writelines(in_dir)
                        break
                    colors = Image.open(color_path)
                    if i == 0:
                        points = colored_depth2pcd(depth, intrinsics, pose, colors)
                    else:
                        points = np.concatenate([points, colored_depth2pcd(depth, intrinsics, pose, colors)], axis=0)
            if model_not_scanned:
                continue
            np.random.shuffle(points)
            # points = points[:2048, :]
            if points.shape[0] >= 16384:
                points = points[:16384, :]
                points = points + np.random.rand(points.shape[0], points.shape[1]) * 0.005
                with h5py.File(os.path.join(out_dir, model_id + '.h5'), 'w') as f:
                    f.create_dataset(name="data", data=np.array(points).astype(np.float32))
            else:
                print(points.shape[0])
                with open('failed.txt', 'a') as f:
                    f.writelines(model_id+'\n')
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(points[:, :3])
            # pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6] / 255.0)
            # o3d.io.write_point_cloud(os.path.join(out_dir, model_id + '.pcd'), pcd)
            # o3d.io.write_point_cloud(os.path.join(out_dir, model_id + '.pcd'), pcd)
            # np.savetxt(os.path.join(out_dir, 'test.txt'), pose, '%.20f')
