import glob
import open3d as o3d
from PIL import Image
import numpy as np
import os
import tqdm
import cv2
import json
import pprint
import pandas as pd

from plyio import write_ply

join = os.path.join
basename = os.path.basename



def read_color_scannet(fn):
    return np.array(Image.open(fn))

def read_depth_scannet(fn):
    return np.array(Image.open(fn), dtype='u2')

def save_color_scannet(color, filename):
    color=color*255
    Image.fromarray(color.astype(np.uint8)).save(filename)

def save_depth_scannet(depth, filename, scale=1000):
    out=depth*scale
    out[out<-1]=0
    Image.fromarray(out.astype(np.uint32)).save(filename)

def get_poses_scannet(n_frames):
    poses=[]
    for frame_id in range(n_frames):
        poses.append(np.loadtxt(join(args.input, 'pose', f'{frame_id}.txt')))
    return poses

def get_frame_scannet(frame_id):
    depth=read_depth_scannet(join(args.input, 'depth', f'{frame_id}.png'))
    color=read_color_scannet(join(args.input, 'color', f'{frame_id}.jpg'))
    return color,depth,poses_scannet[frame_id]

def get_intrinsics_scannet():
    K=np.loadtxt(join(args.input, 'intrinsic', f'intrinsic_depth.txt'))
    intr_path=join('/tmp', f'{args.input.__hash__()}.json')
    json.dump({
        "width": 640,
        "height": 480,
        "intrinsic_matrix": list(K[:3,:3].T.flatten())
    }, open(intr_path,'w'))
    print(json.load(open(intr_path)))
    return o3d.read_pinhole_camera_intrinsic(intr_path)



def to_rgbd(color, depth, depth_scale, depth_trunc):
    if color.shape[:2] != depth.shape[:2]:
        color = cv2.resize(color, (depth.shape[1], depth.shape[0]))
    rgbd = o3d.create_rgbd_image_from_color_and_depth(o3d.Image(color), o3d.Image(depth),
                                                      convert_rgb_to_intensity=False, depth_scale=depth_scale, depth_trunc=depth_trunc)
    return rgbd


def write_as_ply(path, pcd):
    xyz = np.array(pcd.points, np.float32)
    normals = np.array(pcd.normals, np.float32)
    colors = np.array(pcd.colors, np.float32)

    if np.array(pcd.colors).dtype == np.uint8:
        colors /= 255.

    df = {
        'x': xyz[:,0],
        'y': xyz[:,1],
        'z': xyz[:,2],
        'nx': normals[:,0],
        'ny': normals[:,1],
        'nz': normals[:,2],
        'red': colors[:,0],
        'green': colors[:,1],
        'blue': colors[:,2],
    }

    df = pd.DataFrame(df, columns=[
        'x','y','z',
        'nx','ny','nz',
        'red','green','blue'])

    write_ply(path, points=df, as_text=False)


def jitter(pcd, mag):
    pcd = o3d.PointCloud(pcd)
    pts = np.array(pcd.points)
    pts_j = pts + np.random.rand(*pts.shape) * mag - 0.5 * mag
    pcd.points = o3d.Vector3dVector(pts_j)
    return pcd


if __name__ == '__main__':
    import argparse
    parser =  argparse.ArgumentParser()
    parser.add_argument('--input', type=str, required=True)
    parser.add_argument('--take-each', type=int, default=1)
    parser.add_argument('--voxel-size', type=float, default=0.01)
    parser.add_argument('--no-jitter', action='store_true')
    args = parser.parse_args()

    args.jitter = not args.no_jitter
    print(args.jitter)

    
    pprint.pprint(args)

    n_frames = len(os.listdir(join(args.input, 'color')))
    poses_scannet = get_poses_scannet(n_frames)
    get_frame = get_frame_scannet
    depth_scale = 1000.
    depth_trunc = 3
    intr = get_intrinsics_scannet()

    pcd_combined = o3d.PointCloud()
    poses = []
    print('building point cloud...')
    for i in tqdm.tqdm(range(n_frames)):
        pose = poses_scannet[i]

        if not np.isfinite(pose).all():
            print('skip ', i)
            continue

        poses.append(pose)

        if i % args.take_each != 0:
            continue

        color, depth, _ = get_frame(i)
        rgbd = to_rgbd(color, depth, depth_scale, depth_trunc)
        pcd = o3d.create_point_cloud_from_rgbd_image(rgbd, intr)

        pcd.transform(pose)
        pcd_combined += pcd


    print('COMBINED')
    print(pcd_combined)

    print('downsampling...')
    pcd_combined = o3d.voxel_down_sample(pcd_combined, args.voxel_size)
    print(pcd_combined)

    print('calculating normals...')
    o3d.estimate_normals(pcd_combined)

    if args.jitter:
        # apply small jitter to points to destroy grid pattern, which *probably* could lead to model overfitting
        pcd_combined = jitter(pcd_combined, args.voxel_size)

    os.makedirs(join(args.input, 'out'), exist_ok=True)

    get_name = lambda ext: 'downsample_te_{}_vs_{}{}.{}'.format(args.take_each, args.voxel_size, '_jit' if args.jitter else '', ext)

    name_pcd = join(args.input, 'out', get_name('pcd'))
    print(name_pcd)
    o3d.write_point_cloud(name_pcd, pcd_combined)

    name_ply = join(args.input, 'out', get_name('ply'))
    print(name_ply)
    write_as_ply(name_ply, pcd_combined)

    o3d.draw_geometries([pcd_combined])