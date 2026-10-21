import mujoco

from simulator.models.robots.robot_2026.wheel.wheel import Wheel
from simulator.models.robots.robot_model import RobotModelBase
from simulator.models.robots.robot_parameters import (
    RobotMecanumRollingBaseParameters,
    RobotParameters,
)
from simulator.models.robots.robot_specs import RobotSpecs


class Robot2026(RobotModelBase):
    def __init__(self, sim_spec: mujoco.MjSpec, parameters: RobotParameters, robot_specs: RobotSpecs):
        self.parameters = parameters
        self.sim_spec = sim_spec
        if not isinstance(self.parameters.rolling_base, RobotMecanumRollingBaseParameters):
            raise TypeError("Mecanum model is required")
        self.wheel = Wheel(self.sim_spec, self.parameters.rolling_base.wheel, robot_specs)

    def assemble(self, parent_body: mujoco.MjsBody, start_position: tuple[float, float, float]):
        robot_body: mujoco.MjsBody = parent_body.add_body(name="robot", pos=start_position)
        robot_body.add_freejoint(name="robot_freejoin")

        frame_body: mujoco.MjsBody = robot_body.add_body(name="frame", pos=(0, 0, 0))
        frame_body.add_geom(
            name="frame_geom",
            type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname="robot_frame_mesh",
            contype=0,
            conaffinity=0,
            mass=self.parameters.mass - 4 * (self.parameters.rolling_base.wheel.num_rollers + 1) * 0.001,
        )

        # noinspection unresolved-references
        wheel_poses = [
            (
                self.parameters.rolling_base.width / 2 - self.parameters.rolling_base.wheel.diameter / 2,
                self.parameters.rolling_base.length / 2,
                1
            ),
            (
                -self.parameters.rolling_base.width / 2 + self.parameters.rolling_base.wheel.diameter / 2,
                self.parameters.rolling_base.length / 2,
                -1
            ),
            (
                self.parameters.rolling_base.width / 2 - self.parameters.rolling_base.wheel.diameter / 2,
                -self.parameters.rolling_base.length / 2,
                -1
            ),
            (
                -self.parameters.rolling_base.width / 2 + self.parameters.rolling_base.wheel.diameter / 2,
                -self.parameters.rolling_base.length / 2,
                1
            ),
        ]

        for i, pos in enumerate(wheel_poses):
            self.wheel.attach(
                robot_body,
                name=f"wheel{i+1}",
                position=(pos[0], pos[1], self.parameters.rolling_base.wheel.diameter / 2),
                side=pos[2]
            )
        return robot_body