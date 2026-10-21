import mujoco
import mujoco.viewer

from simulator.models.robots.robot_model import RobotModelBase


class SceneModel:
    def __init__(self, sim_spec: mujoco.MjSpec, robot_frame_mesh_path: str):
        # C'est la scène qui possède l'objet MjSpec racine
        self.sim_spec = sim_spec
        self.sim_spec.option.gravity = [0, 0, -9.81]
        self.sim_spec.option.timestep = 0.002

        # Décor : Sol et Lumière
        self.sim_spec.worldbody.add_light(pos=[0, 0, 4], dir=[0, 0, -1])
        self.sim_spec.worldbody.add_geom(
            name="sol",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[3, 3, 0.1],
            rgba=[0.85, 0.85, 0.85, 1],
        )

        self.sim_spec.add_mesh(name="robot_frame_mesh", file=robot_frame_mesh_path)

    def add_robot(self, robot: RobotModelBase, position: tuple[float, float, float]):
        """Injecte le robot dans le monde de la scène"""
        robot.assemble(self.sim_spec.worldbody, start_position=position)

    def compile(self):
        """Transforme l'arbre complet en modèle physique exécutable"""
        model = self.sim_spec.compile(None)
        data = mujoco.MjData(model)
        return model, data
