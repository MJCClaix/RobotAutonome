from typing import Annotated, Literal

from pydantic import BaseModel, Field, field_validator


class RobotMecanumWheelParameters(BaseModel):
    num_rollers: int
    diameter: float
    thickness: float

    @field_validator("diameter", "thickness", mode="before")
    @classmethod
    def convert_fields(cls, value: int):
        return value / 1000


class RobotHolonomWheelParameters(BaseModel):
    num_rollers: int
    diameter: float
    thickness: float

    @field_validator("diameter", "thickness", mode="before")
    @classmethod
    def convert_fields(cls, value: int):
        return value / 1000


class RobotMecanumRollingBaseParameters(BaseModel):
    type: Literal["mecanum"]
    length: float
    width: float
    wheel: RobotMecanumWheelParameters

    @field_validator("length", "width", mode="before")
    @classmethod
    def convert_fields(cls, value: int):
        return value / 1000


class RobotHolonomRollingBaseParameters(BaseModel):
    type: Literal["holonom"]
    diameter: float
    wheel: RobotHolonomWheelParameters

    @field_validator("diameter", mode="before")
    @classmethod
    def convert_fields(cls, value: int):
        return value / 1000


class RobotParameters(BaseModel):
    height: float
    mass: float
    rolling_base: Annotated[RobotMecanumRollingBaseParameters | RobotHolonomRollingBaseParameters, Field(discriminator="type")]

    @field_validator("height", "mass", mode="before")
    @classmethod
    def convert_fields(cls, value: int):
        return value / 1000
