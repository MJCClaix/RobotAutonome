from math import pi

from pydantic import BaseModel, field_validator


class RobotMotorSpecs(BaseModel):
    locking_torque: float
    no_load_speed: float
    reduction_ratio: float
    base_rotor_inertia: float
    dead_zone: float
    torque_uncertainty: float
    speed_uncertainty: float

    @field_validator("locking_torque", mode="before")
    @classmethod
    def locking_pair_converter(cls, value: float):
        return value * 0.0980665

    @field_validator("no_load_speed", mode="before")
    @classmethod
    def no_load_speed_converter(cls, value: float):
        return value * 2 * pi / 60


class RobotWheelSpecs(BaseModel):
    nominal_friction: float
    nominal_friction_uncertainty: float


class RobotSpecs(BaseModel):
    motors: RobotMotorSpecs
    wheels: RobotWheelSpecs
