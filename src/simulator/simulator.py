import json
import time
from pathlib import Path

import mujoco.viewer

from simulator.models.robots.robot_2026.robot2026 import Robot2026
from simulator.models.robots.robot_parameters import RobotParameters
from simulator.models.robots.robot_specs import RobotSpecs
from simulator.models.scenes.scene_parameters import SceneParameters

from .models.scenes.scene_2027 import SceneModel

robot_parameters_file_path = "assets/simulator/models_parameters/robot2026.json"
robot_specs_file_path = "assets/simulator/components_specs/robot2026.json"
robot_frame_mesh_path = "assets/simulator/3D_models/robots/frames/frame2026.stl"
scene_parameters_file_path = "assets/simulator/scenes_parameters/scene2026.json"
textures_folder_path = Path("assets/simulator/textures")

if __name__ == "__main__":
    with open(robot_parameters_file_path) as f:
        robot_parameters = RobotParameters(**json.load(f))

    with open(robot_specs_file_path) as f:
        robot_specs = RobotSpecs(**json.load(f))

    with open(scene_parameters_file_path) as f:
        scene_parameters = SceneParameters(**json.load(f))

    sim_spec = mujoco.MjSpec()

    scene = SceneModel(sim_spec, scene_parameters, robot_frame_mesh_path, textures_folder_path)
    robot = Robot2026(sim_spec, robot_parameters, robot_specs)

    scene.add_robot(robot, (0, 0, 0))

    model, data = scene.compile()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            data.ctrl[0] = -0.5
            data.ctrl[1] = 0.5
            data.ctrl[2] = 0.5
            data.ctrl[3] = -0.5

            # noinspection argument-list
            mujoco.mj_step(model, data)
            viewer.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)
