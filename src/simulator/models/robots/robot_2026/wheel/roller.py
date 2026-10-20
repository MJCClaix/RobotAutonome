from math import radians, sqrt

import mujoco

from simulator.models.robots.robot_parameters import RobotMecanumWheelParameters
from simulator.models.robots.robot_part import RobotPart


class Roller(RobotPart):
    def __init__(self, parameters: RobotMecanumWheelParameters, diameter: float):
        self.parameters = parameters
        self.diameter = diameter
        self.lenght = sqrt(2 * self.parameters.thickness ** 2)

    def attach(self, parent_body: mujoco.MjsBody, name: str, position: tuple[float, float, float], euler: tuple[float, float, float], side: int) -> mujoco.MjsBody:
        roller_body: mujoco.MjsBody = parent_body.add_body(name=name, pos=position, euler=euler)
        roller_body.add_geom(
            name=f"{name}_geom",
            type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
            size=(self.diameter / 2, self.lenght / 2, self.diameter / 2),
            euler=(45 * side, 0, 0),
            mass=0.001,
        )
        roller_body.add_joint(name=f"{name}_axe", type=mujoco.mjtJoint.mjJNT_HINGE, axis=(0, 1, side))
        return roller_body
