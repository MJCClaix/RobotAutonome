from pathlib import Path

import mujoco
import mujoco.viewer

from simulator.models.robots.robot_model import RobotModelBase
from simulator.models.scenes.scene_parameters import SceneParameters


class SceneModel:
    def __init__(self, sim_spec: mujoco.MjSpec, scene_parameters: SceneParameters, robot_frame_mesh_path: str, textures_folder_path: Path) -> None:
        # C'est la scène qui possède l'objet MjSpec racine
        self.sim_spec = sim_spec
        self.scene_parameters = scene_parameters
        self.textures_folder_path = Path(textures_folder_path)

        self.sim_spec.option.gravity = [0, 0, -9.81]
        self.sim_spec.option.timestep = 0.002

        self.sim_spec.add_texture(
            name="wood_texture",
            type=mujoco.mjtTexture.mjTEXTURE_CUBE,
            file=str(textures_folder_path / "wood.png"),
        )
        self.sim_spec.add_texture(
            name="table_texture",
            type=mujoco.mjtTexture.mjTEXTURE_2D,
            file=str(textures_folder_path / "2027" / "table.png"),
        )
        self.sim_spec.add_texture(
            name="aruco_marker_texture",
            type=mujoco.mjtTexture.mjTEXTURE_2D,
            file=str(textures_folder_path / "aruco_marker.png"),
        )

        self.sim_spec.add_material(
            name="wood_material",
            textures=["", "wood_texture"],
            texuniform=True,
        )
        self.sim_spec.add_material(
            name="table_material",
            textures=["", "table_texture"],
        )
        self.sim_spec.add_material(
            name="aruco_marker_material",
            textures=["", "aruco_marker_texture"],
        )

        # Décor : Sol et Lumière
        self.sim_spec.worldbody.add_light(pos=[0, 0, 4], dir=[0, 0, -1])
        self.sim_spec.worldbody.add_geom(
            name="sol",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[self.scene_parameters.width / 2, self.scene_parameters.height / 2, 0.1],
            material="table_material",
        )
        self.sim_spec.worldbody.add_geom(
            name="sol_border1",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[self.scene_parameters.width / 2 + self.scene_parameters.borders.thickness , self.scene_parameters.borders.thickness / 2, self.scene_parameters.borders.height / 2],
            rgba=[0.85, 0.85, 0.85, 1],
            pos=[0, self.scene_parameters.height / 2 + self.scene_parameters.borders.thickness / 2, self.scene_parameters.borders.height / 2],
            material="wood_material",
        )
        self.sim_spec.worldbody.add_geom(
            name="sol_border2",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[self.scene_parameters.borders.thickness / 2, self.scene_parameters.height / 2, self.scene_parameters.borders.height / 2],
            rgba=[0.85, 0.85, 0.85, 1],
            pos=[self.scene_parameters.width / 2 + self.scene_parameters.borders.thickness / 2, 0, self.scene_parameters.borders.height / 2],
            material="wood_material",
        )
        self.sim_spec.worldbody.add_geom(
            name="sol_border3",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[self.scene_parameters.width / 2 + self.scene_parameters.borders.thickness, self.scene_parameters.borders.thickness / 2, self.scene_parameters.borders.height / 2],
            rgba=[0.85, 0.85, 0.85, 1],
            pos=[0, -self.scene_parameters.height / 2 - self.scene_parameters.borders.thickness / 2, self.scene_parameters.borders.height / 2],
            material="wood_material",
        )
        self.sim_spec.worldbody.add_geom(
            name="sol_border4",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[self.scene_parameters.borders.thickness / 2,self.scene_parameters.height / 2,self.scene_parameters.borders.height / 2],
            rgba=[0.85, 0.85, 0.85, 1],
            pos=[-self.scene_parameters.width / 2 - self.scene_parameters.borders.thickness / 2, 0, self.scene_parameters.borders.height / 2],
            material="wood_material",
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
