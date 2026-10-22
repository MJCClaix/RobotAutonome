from pydantic import BaseModel, field_validator


class SceneBordersParameters(BaseModel):
    height: float
    thickness: float

    @field_validator("height", "thickness", mode="before")
    @classmethod
    def validate(cls, value: float):
        return value / 1000


class SceneParameters(BaseModel):
    width: float
    height: float
    borders: SceneBordersParameters

    @field_validator("width", "height", mode="before")
    @classmethod
    def validate(cls, value: float):
        return value / 1000