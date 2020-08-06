import bpy
import addon_utils
from mathutils import Matrix
import numpy as np
import os
import sys
import time

def setup_blender(width, height, focal_length):
    # camera
    camera = bpy.data.objects['Camera']
    camera.data.sensor_height = camera.data.sensor_width
    camera.data.angle = np.arctan(width / 2 / focal_length) * 2
    # render layer
    scene = bpy.context.scene
    # scene.render.alpha_mode = 'TRANSPARENT'
    scene.render.image_settings.color_depth = '16'
    scene.render.image_settings.use_zbuffer = True
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100

    # compositor nodes
    scene.use_nodes = True
    tree = scene.node_tree
    for n in tree.nodes:
        tree.nodes.remove(n)
    rl = tree.nodes.new('CompositorNodeRLayers')
    output = tree.nodes.new('CompositorNodeOutputFile')
    output.format.file_format = 'OPEN_EXR'
    tree.links.new(rl.outputs['Depth'], output.inputs[0])

    # remove default cube
    bpy.data.objects['Cube'].select_set(state=True)
    bpy.ops.object.delete()

    return scene, camera, output

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)

if __name__ == '__main__':
    model_dir = sys.argv[-5]
    category = sys.argv[-4]
    model_id = sys.argv[-3]
    intrinsics_file = sys.argv[-2]
    output_dir = sys.argv[-1]

    model_path = os.path.join(model_dir, 'model.obj')
    if os.path.isfile(model_path):
        start = time.time()

        # Output directory settings
        output_dir = os.path.join(ROOT_DIR, output_dir)
        output_dir = os.path.join(output_dir, category, model_id)
        os.makedirs(output_dir, exist_ok=True)

        # Redirect output to log file
        log_path = os.path.join(output_dir, 'blender_render.log')
        open(log_path, 'a').close()
        old = os.dup(1)
        os.close(1)
        os.open(log_path, os.O_WRONLY)

        # Camera settings
        intrinsics = np.loadtxt(intrinsics_file)
        focal = intrinsics[0, 0]
        width = int(intrinsics[0, 2] * 2)
        height = int(intrinsics[1, 2] * 2)
        scene, camera, output = setup_blender(width, height, focal)

        # Render output settings
        output.base_path = output_dir

        # Import obj files

        bpy.ops.import_scene.obj(filepath=model_path, axis_forward='Y', axis_up='Z')

        # Depth map output settings
        num_scans = 6
        np.savetxt(os.path.join(output_dir, 'num_scans.txt'), np.array([num_scans]), '%d')
        output.file_slots[0].path = os.path.join('#.exr')

        # Pose cameras & Scanning
        dist_to_origin = 1
        rot_axis_list = ['Y', 'X', 'X']
        rot_angle_list = [[np.pi / 2.0, -np.pi / 2.0], [-np.pi / 2.0, np.pi / 2.0], [0, np.pi]]
        for i in range(int(num_scans / 2)):
            rot_axis = rot_axis_list[i]
            for j in range(2):
                dist_factor = 1 if j == 0 else -1

                rot = Matrix.Rotation(rot_angle_list[i][j], 4, rot_axis)
                translation = [0, 0, 0]
                translation[i] = dist_to_origin * dist_factor
                translation = Matrix.Translation(translation)
                camera.matrix_world = Matrix(np.matmul(translation, rot))
                np.savetxt(os.path.join(output_dir, '%d.txt' % (i * 2 + j)),
                           np.array(camera.matrix_world), '%.20f')
                scene.render.filepath = os.path.join(output_dir, str(i * 2 + j) + '.png')
                scene.frame_set(i * 2 + j)
                bpy.ops.render.render(write_still=True)

        # Clean up object
        bpy.ops.object.delete()
        # show time
        os.close(1)
        os.dup(old)
        os.close(old)
        print('{1} done, time={0:.4f} sec'.format(time.time() - start, model_id))
    else:
        with open(os.path.join(ROOT_DIR, 'failed.txt'), 'a') as f_exp:
            f_exp.writelines(model_path)
