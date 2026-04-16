"""Utilities to load example Hiwonder 5-DOF manipulator."""

import coal
import numpy as np
import os
import pinocchio

from ..core.utils import set_collisions
from .utils import get_example_models_folder


def load_models():
    """
    Gets the example Hiwonder 5-DOF model.

    Returns
    -------
        tuple[`pinocchio.Model`]
            A 3-tuple containing the model, collision geometry model, and visual geometry model.
    """
    models_folder = get_example_models_folder()
    package_dir = os.path.join(models_folder, "hiwonder_description")
    urdf_filename = os.path.join(package_dir, "hiwonder.urdf")

    return pinocchio.buildModelsFromUrdf(urdf_filename, package_dirs=models_folder)


def add_object_collisions(model, collision_model, visual_model, obstacle_list=None):
    """
    Adds obstacles and collisions to the Hiwonder 5-DOF manipulator collision model.

    Parameters
    ----------
        model : `pinocchio.Model`
            The robot model.
        collision_model : `pinocchio.Model`
            The collision geometry model.
        visual_model : `pinocchio.Model`
            The visual geometry model.
    """
    if obstacle_list is None:
        obstacle_list = [
            {"type": "cylinder", "center": [-0.2, 0.15, 0.2], "radius": 0.05, "height": 0.4},
            {"type": "box", "center": [0.2, -0.2, 0.2], "size": [0.1, 0.1, 0.4]},
        ]

    for i, obs in enumerate(obstacle_list):
        if obs["type"] == "cylinder":
            obstacle = pinocchio.GeometryObject(
                f"obstacle_{i+1}",
                0,
                # pinocchio.SE3(np.eye(3), np.array([0.35, 0.3, 0.0])),
                pinocchio.SE3(np.eye(3), np.array(obs["center"])),
                coal.Cylinder(obs["radius"], obs["height"]),
            )
            obstacle.meshColor = np.array([0.0, 1.0, 0.0, 0.5])
            visual_model.addGeometryObject(obstacle)
            collision_model.addGeometryObject(obstacle)

        elif obs["type"] == "box":
            obstacle = pinocchio.GeometryObject(
                f"obstacle_{i+1}",
                0,
                pinocchio.SE3(np.eye(3), np.array(obs["center"])),
                coal.Box(obs["size"][0], obs["size"][1], obs["size"][2]),
            )
            obstacle.meshColor = np.array([1.0, 0.0, 0.0, 0.5])
            visual_model.addGeometryObject(obstacle)
            collision_model.addGeometryObject(obstacle)


    # Define the active collision pairs between the robot and obstacle links.
    collision_names = [
        cobj.name for cobj in collision_model.geometryObjects if "link" in cobj.name
    ]
    
    obstacle_names = [f"obstacle_{i+1}" for i in range(len(obstacle_list))]
    for obstacle_name in obstacle_names:
        for collision_name in collision_names:
            set_collisions(model, collision_model, obstacle_name, collision_name, True)

