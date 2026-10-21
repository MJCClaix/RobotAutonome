import random
from math import cos, degrees, pi, radians, sin, sqrt

import mujoco

from simulator.models.robots.robot_2026.wheel.roller import Roller
from simulator.models.robots.robot_parameters import RobotMecanumWheelParameters
from simulator.models.robots.robot_part import RobotPart
from simulator.models.robots.robot_specs import RobotSpecs


class Wheel(RobotPart):
    def __init__(self, sim_spec: mujoco.MjSpec, parameters: RobotMecanumWheelParameters, robot_specs: RobotSpecs):
        self.sim_spec = sim_spec
        self.parameters = parameters
        self.robot_specs = robot_specs
        
        k_cyl = sin(radians(180) / self.parameters.num_rollers) / sqrt(1 + cos(radians(180) / self.parameters.num_rollers) ** 2)
        m_contact = sqrt(2) * sin(radians(360) / self.parameters.num_rollers) / (3 + cos(radians(360) / self.parameters.num_rollers))
        k_ratio = sqrt((1 + k_cyl) ** 2 + (m_contact ** 2) / 2)
        self.hub_diameter = self.parameters.diameter / k_ratio
        self.roll_diameter = self.parameters.diameter - self.hub_diameter

        self.max_torque = random.gauss(self.robot_specs.motors.locking_torque, self.robot_specs.motors.locking_torque * self.robot_specs.motors.torque_uncertainty)
        self.max_speed = random.gauss(self.robot_specs.motors.no_load_speed, self.robot_specs.motors.no_load_speed * self.robot_specs.motors.speed_uncertainty)
        self.motor_damping = self.max_torque / self.max_speed
        self.rotor_inertia = self.robot_specs.motors.base_rotor_inertia * self.robot_specs.motors.reduction_ratio ** 2
        self.dead_zone = self.max_torque * self.robot_specs.motors.dead_zone

        self.roller = Roller(self.parameters, self.robot_specs.wheels, self.roll_diameter)

    def attach(self, parent_body: mujoco.MjsBody, name: str, position: tuple[float, float, float], side: int):
        wheel_body: mujoco.MjsBody = parent_body.add_body(name=name, pos=position)
        axe: mujoco.MjsJoint = wheel_body.add_joint(
            name=f"{name}_axe",
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=(0, 1, 0),
            damping=self.motor_damping,
            armature=self.rotor_inertia,
            frictionloss=self.dead_zone,
        )
        self.sim_spec.add_actuator(
            name=f"{name}_cmd",
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            target=axe.name,
            gaintype=mujoco.mjtGain.mjGAIN_FIXED,
            dyntype=mujoco.mjtDyn.mjDYN_NONE,
            ctrlrange=[-1.0, 1.0],
            gear=(self.max_torque, 0, 0, 0, 0, 0),
        )

        hub_body: mujoco.MjsBody = wheel_body.add_body(name=f"{name}_hub", pos=(0, 0, 0))
        hub_body.add_geom(
            name=f"{name}_hub_geom",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=(self.hub_diameter / 2, self.parameters.thickness / 2),
            rgba=(0.2, 0.2, 0.2, 1),
            mass=0.001,
            zaxis=[0, 1, 0],
            contype=0,
            conaffinity=0,
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

        num_rollers = self.parameters.num_rollers
        for i in range(num_rollers):
            j = (i + 1) % num_rollers  # le dernier rouleau est voisin du premier
            self.sim_spec.add_exclude(
                name=f"{name}_roller{i + 1}_roller{j + 1}_excl",
                bodyname1=f"{name}_roller{i + 1}",
                bodyname2=f"{name}_roller{j + 1}",
            )

        return wheel_body