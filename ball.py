
from PySide6.QtGui import QVector3D, QColor
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras

import pybullet

from collision_group import CollisionGroup
from global_config import grid_size


class Ball:

    root_entity = None

    ball_list = list()

    entity = None
    mesh = None
    material = None
    transform = None
    body = None

    def __init__(self, root_entity, position, vector):
        self.root_entity = root_entity

        self.mesh = Qt3DExtras.QSphereMesh(self.root_entity)
        self.mesh.setRadius(grid_size * 0.02)

        self.material = Qt3DExtras.QPhongMaterial(self.root_entity)
        self.material.setAmbient(QColor(255, 255, 255))
        self.material.setDiffuse(QColor(255, 255, 255))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)

        self.entity = Qt3DCore.QEntity(self.root_entity)

        self.body = pybullet.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE,
                radius=grid_size * 0.02,
            ),
            basePosition=[position.x(), position.z(), position.y()],
        )
        pybullet.changeDynamics(self.body, -1, restitution=0.9)
        pybullet.resetBaseVelocity(self.body, linearVelocity=[
            vector.x(),
            vector.z(),
            vector.y(),
        ])
        pybullet.setCollisionFilterGroupMask(
            self.body, -1, CollisionGroup.get_group("ball"), CollisionGroup.get_mask("ball"))

        pos, _ = pybullet.getBasePositionAndOrientation(self.body)

        self.transform = Qt3DCore.QTransform(self.root_entity)
        self.transform.setTranslation(QVector3D(
            pos[0],
            pos[2],
            pos[1],
        ))

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.material)
        self.entity.addComponent(self.transform)

    def update(self):
        pos, _ = pybullet.getBasePositionAndOrientation(self.body)
        self.transform.setTranslation(QVector3D(pos[0], pos[2], pos[1]))


class BallList:
    _list = list()

    body_list = list()

    def __init__(self, root_entity):
        self.root_entity = root_entity

    def create_ball(self, position, vector):
        self._list.append(Ball(self.root_entity, position, vector))
        self.body_list.append(self._list[-1].body)

        if len(self._list) > 10:
            pybullet.removeBody(self._list[0].body)
            self._list[0].entity.setParent(None)
            self._list[0].entity.deleteLater()
            self._list.pop(0)

    def update(self):
        for i in self._list:
            i.update()
