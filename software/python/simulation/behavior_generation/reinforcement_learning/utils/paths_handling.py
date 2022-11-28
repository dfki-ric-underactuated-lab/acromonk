import os

project_root_dir = os.path.join(os.path.sep, *os.path.dirname(os.path.abspath(__file__)).split(os.path.sep)[:5])
rl_root_dir = os.path.join(os.path.sep, *os.path.dirname(os.path.abspath(__file__)).split(os.path.sep)[:-1])
