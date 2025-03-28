
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QColor

import pybullet

from collision_group import CollisionGroup
from global_config import grid_size, maze_size


class Ground:

    entity = None
    mesh = None
    transform = None
    material = None

    body = None

    def __init__(self, root_entity):

        self.entity = Qt3DCore.QEntity(root_entity)

        self.mesh = Qt3DExtras.QPlaneMesh()
        self.mesh.setWidth(grid_size * maze_size * 4)
        self.mesh.setHeight(grid_size * maze_size * 4)

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(QVector3D(0, 0, 0))

        self.material = Qt3DExtras.QPhongMaterial(root_entity)
        # self.material.setAmbient(QColor(0, 127, 0))
        self.material.setDiffuse(QColor(0, 31, 0))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.transform)
        self.entity.addComponent(self.material)

        self.body = pybullet.loadURDF("plane.urdf")
        pybullet.changeDynamics(self.body, -1, restitution=0.9)
        pybullet.setCollisionFilterGroupMask(
            self.body, -1, CollisionGroup.get_group("env"), CollisionGroup.get_mask("env"))
