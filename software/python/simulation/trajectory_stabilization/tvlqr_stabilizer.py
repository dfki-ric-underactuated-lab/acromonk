import sys

sys.path.append("../../utilities/")
from utils import (
    drake_visualizer,
    generate_path,
    create_acromonk_plant,
    np,
    save_data,
    save_dict,
    load_desired_trajectory
)
plant, context, scene_graph, builder = create_acromonk_plant()
x0, u0, _, _ =  load_desired_trajectory("BF")