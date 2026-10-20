import json
import time

import mujoco.viewer

from simulator.models.robots.robot_2026.robot2026 import Robot2026
from simulator.models.robots.robot_parameters import RobotParameters

from .models.scenes.scene_2027 import SceneModel

robot_parameters_file_path = "assets/simulator/models_parameters/robot2026.json"
robot_frame_mesh_path = "assets/simulator/3D_models/robots/frames/frame2026.stl"

if __name__ == "__main__":
    with open(robot_parameters_file_path) as f:
        robot_parameters = RobotParameters(**json.load(f))

    scene = SceneModel(robot_frame_mesh_path)
    robot = Robot2026(robot_parameters)

    scene.add_robot(robot, (0, 0, 0))

    model, data = scene.compile()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # noinspection argument-list
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
