
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DCore import Qt3DCore
from PySide6.QtGui import QColor, QVector3D, QCursor

import math
import pybullet

from collision_group import CollisionGroup
from global_config import grid_size, root_entity
from viewport_manager import Layer


class Player:

    mass = 10
    body = None

    entity_map = None
    mesh_map = None
    transform_map = None
    material_map = None

    latest_pos_move_vector = None

    def __init__(self):
        self.mass = grid_size
        self.body = pybullet.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_CAPSULE,
                radius=grid_size * 0.1,
                height=grid_size * 0.5,
            ),
            basePosition=[
                grid_size * 0.5,
                grid_size * 0.5,
                grid_size * 0.5,
            ],
        )
        pybullet.changeDynamics(
            self.body, -1,
            angularDamping=0.0,
            linearDamping=1.0,
            localInertiaDiagonal=[1e9, 1e9, 0],
        )

        pybullet.setCollisionFilterGroupMask(
            self.body, -1, CollisionGroup.get_group("player"), CollisionGroup.get_mask("player"))

        self.mesh_map = Qt3DExtras.QConeMesh()
        self.mesh_map.setBottomRadius(grid_size * 0.3)
        self.mesh_map.setTopRadius(0)
        self.mesh_map.setLength(grid_size * 0.5)

        self.transform_map = Qt3DCore.QTransform()

        self.material_map = Qt3DExtras.QPhongMaterial()
        self.material_map.setAmbient(QColor(255, 255, 255))
        self.material_map.setDiffuse(QColor(255, 255, 255))
        self.material_map.setSpecular(QColor(255, 255, 255))
        self.material_map.setShininess(0)

        self.entity_map = Qt3DCore.QEntity(root_entity)
        self.entity_map.addComponent(self.mesh_map)
        self.entity_map.addComponent(self.transform_map)
        self.entity_map.addComponent(self.material_map)

        self.entity_map.addComponent(Layer().get("ui"))

    def update(self, controller):
        self.latest_pos_move_vector = controller.cal_pos_move_vector()
        if self.latest_pos_move_vector.length() > 0.01:
            force = [
                self.latest_pos_move_vector.x() * self.mass,
                self.latest_pos_move_vector.z() * self.mass,
                self.latest_pos_move_vector.y() / 100 * self.mass,
            ]
            pybullet.applyExternalForce(self.body, -1, force, [0, 0, 0], pybullet.LINK_FRAME)

    def update_map(self, view_vector):
        player_pos, player_ori = pybullet.getBasePositionAndOrientation(self.body)
        player_linear_vel, player_angular_vel = pybullet.getBaseVelocity(self.body)

        self.transform_map.setTranslation(QVector3D(
            player_pos[0],
            player_pos[2],
            player_pos[1],
        ))
        self.transform_map.setRotationY(
            -90 + math.degrees(math.atan2(view_vector.x(), view_vector.z()))
        )
