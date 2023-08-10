import os
from src.maze_blimps import *

maze_files_dir = os.path.join(DIR, 'evolution', 'maze_files')
if not os.path.exists(maze_files_dir):
    os.makedirs(maze_files_dir)

def find_n_and_next_maze_file_name(file_dir=maze_files_dir):
    files = [f for f in os.listdir(file_dir) if f.startswith('maze_')]
    if not files:
        return None, 'maze_0.txt'
    else:
        number = max([int(f[5:-4]) for f in files])
        return 'maze_' + str(number) + '.txt', 'maze_' + str(number + 1) + '.txt'


def load_and_use_maze(maze, file_dir=maze_files_dir):
    name, _ = find_n_and_next_maze_file_name(file_dir=file_dir)
    if name is None:
        return False
    else:
        old_path=os.path.join(file_dir, name)
        new_name='used_'+name
        load_maze_with(maze, filee=old_path)
        os.rename(old_path,os.path.join(file_dir,new_name))
        return True


def find_maze_name_and_save(maze, file_dir=maze_files_dir):
    _, name = find_n_and_next_maze_file_name(file_dir=file_dir)
    save_maze_to(maze, os.path.join(file_dir, name))