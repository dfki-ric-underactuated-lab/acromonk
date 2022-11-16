# function to create log folder for training

import os
import datetime
import shutil
from utils.paths_handling import project_root_dir


def make_log_folder():
    time_now = datetime.datetime.today()
    year = time_now.year
    month = time_now.month
    day = time_now.day
    hour = time_now.hour
    minute = time_now.minute
    second = time_now.second

    save_folder = os.path.join(project_root_dir, 'data', 'trained_agents',
                               f'{year}_{month}_{day}_{hour}_{minute}_{second}')
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
        os.makedirs(os.path.join(save_folder, 'parameters'))
    else:
        raise NameError('Folder for saving model already exists!')

    # copy the current parameter settings for reproducibility
    parameter_files_list = os.listdir(os.path.join(project_root_dir, 'parameters'))
    for param_file in parameter_files_list:
        file_src = os.path.join(project_root_dir, 'parameters', param_file)
        file_dest = os.path.join(save_folder, 'parameters', param_file)
        shutil.copy(file_src, file_dest)

    return save_folder

