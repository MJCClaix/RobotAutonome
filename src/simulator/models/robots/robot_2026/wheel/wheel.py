from math import cos, degrees, pi, radians, sin, sqrt

import mujoco

from simulator.models.robots.robot_2026.wheel.roller import Roller
from simulator.models.robots.robot_parameters import RobotMecanumWheelParameters
from simulator.models.robots.robot_part import RobotPart


class Wheel(RobotPart):
    def __init__(self, parameters: RobotMecanumWheelParameters):
        self.parameters = parameters
        
        k_cyl = sin(radians(180) / self.parameters.num_rollers) / sqrt(1 + cos(radians(180) / self.parameters.num_rollers) ** 2)
        m_contact = sqrt(2) * sin(radians(360) / self.parameters.num_rollers) / (3 + cos(radians(360) / self.parameters.num_rollers))
        k_ratio = sqrt((1 + k_cyl) ** 2 + (m_contact ** 2) / 2)
        self.hub_diameter = self.parameters.diameter / k_ratio
        self.roll_diameter = self.parameters.diameter - self.hub_diameter

        self.roller = Roller(self.parameters, self.roll_diameter)

    def attach(self, parent_body: mujoco.MjsBody, name: str, position: tuple[float, float, float], side: int):
        wheel_body: mujoco.MjsBody = parent_body.add_body(name=name, pos=position)
        wheel_body.add_joint(name=f"{name}_axe", type=mujoco.mjtJoint.mjJNT_HINGE, axis=(0, 1, 0))

        hub_body: mujoco.MjsBody = wheel_body.add_body(name=f"{name}_hub", pos=(0, 0, 0))
        hub_body.add_geom(
            name=f"{name}_hub_geom",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=(self.hub_diameter / 2, self.parameters.thickness / 2),
            rgba=(0.2, 0.2, 0.2, 1),
            mass=0.001,
            zaxis=[0, 1, 0],
        )

        for i in range(self.parameters.num_rollers):
            # angle du roller autour de l'axe de la roue (axe y)
            theta = 2 * pi / self.parameters.num_rollers * i
            radius = self.hub_diameter / 2
            self.roller.attach(
                wheel_body,
                name=f"{name}_roller{i + 1}",
                position=(radius * cos(theta), 0, -radius * sin(theta)),
                euler=(0, degrees(theta), 0),
                side=side
            )

        return wheel_body