from abc import ABC, abstractmethod

import mujoco


class RobotModelBase(ABC):
    @abstractmethod
    def assemble(self, parent_body: mujoco.MjsBody, start_position: tuple[float, float, float]):
        pass