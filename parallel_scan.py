import argparse
import os
import sys
import subprocess
from functools import partial
from multiprocessing.dummy import Pool

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)

blender_command = 'blender'

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', type=str, default='shapenet')
    parser.add_argument('--intrinsics', type=str, default='intrinsics.txt')
    parser.add_argument('--output_dir', type=str, default='images_min')
    parser.add_argument('--num_process', type=int, default=4, help='number of processes')
    parser.add_argument('--absolute_path', action='store_true', default=False)

    args = parser.parse_args()

    if not args.absolute_path:
        args.output_dir = os.path.join(ROOT_DIR, args.output_dir)
        dataset_dir = os.path.join(ROOT_DIR, args.dataset_dir)

    category_dir_list = [os.path.join(dataset_dir, i) for i in os.listdir(dataset_dir)
                         if os.path.isdir(os.path.join(dataset_dir, i))]
    category_list = [i for i in os.listdir(dataset_dir)
                         if os.path.isdir(os.path.join(dataset_dir, i))]

    model_dir_list = []
    model_list = []
    commands = []
    for category, category_dir in zip(category_list, category_dir_list):
        model_dir_list = [os.path.join(category_dir, i) for i in os.listdir(category_dir)
                          if os.path.isdir(os.path.join(category_dir, i))]
        model_list = [i for i in os.listdir(category_dir)
                           if os.path.isdir(os.path.join(category_dir, i))]
        commands.extend([[blender_command, '-b', '-P', 'scan_single.py',
                          model_dir, category, model, args.intrinsics, args.output_dir]
                         for model_dir, model in zip(model_dir_list, model_list)])

    pool = Pool(args.num_process)
    print('=== Rendering %d models on %d workers...' % (len(commands), args.num_process))
    for idx, completed in enumerate(pool.imap(partial(subprocess.run), commands)):
        print('Finished (%d / %d)' % (idx + 1, len(commands)))