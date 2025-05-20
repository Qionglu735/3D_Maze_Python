
from PySide6.QtGui import QVector3D, QColor
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras

import pybullet
import random

from collision_group import CollisionGroup
from global_config import grid_size
from viewport_manager import Layer


class Ball:

    root_entity = None

    ball_list = list()

    entity = None
    mesh = None
    material = None
    transform = None
    body = None
    # body_type = "rigid"
    body_type = "sticky"
    fix_position = False

    last_pos = None
    last_ori = None

    def __init__(self, root_entity, position, vector):
        self.root_entity = root_entity

        self.mesh = Qt3DExtras.QSphereMesh()
        self.mesh.setRadius(grid_size * 0.02)

        self.material = Qt3DExtras.QPhongMaterial()
        color_value = [
            2 ** 6 - 1,
            2 ** 6.2 - 1,
            2 ** 6.3 - 1,
            2 ** 6.5 - 1,
            2 ** 6.7 - 1,
            2 ** 7 - 1,
        ]
        color = QColor(random.choice(color_value), random.choice(color_value), random.choice(color_value))
        self.material.setAmbient(color)
        self.material.setDiffuse(color)
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
        self.entity.addComponent(Layer().get("scene"))

    def update(self):
        if self.fix_position:
            pybullet.resetBasePositionAndOrientation(self.body, self.last_pos, self.last_ori)
            pybullet.resetBaseVelocity(self.body, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

        self.last_pos, self.last_ori = pybullet.getBasePositionAndOrientation(self.body)
        self.transform.setTranslation(QVector3D(self.last_pos[0], self.last_pos[2], self.last_pos[1]))


class BallList:
    _list = list()

    body_list = list()

    def __init__(self, root_entity):
        self.root_entity = root_entity

    def create_ball(self, position, vector, body_type="rigid"):
        self._list.append(Ball(self.root_entity, position, vector))
        self._list[-1].body_type = body_type
        self.body_list.append(self._list[-1].body)

        if len(self._list) > 50:
            pybullet.removeBody(self._list[0].body)
            self.body_list.remove(self._list[0].body)
            self._list[0].entity.setParent(None)
            self._list[0].entity.deleteLater()
            self._list.pop(0)

    def find_by_body(self, body_id):
        try:
            index = self.body_list.index(body_id)
            return self._list[index]
        except ValueError:
            return None

    def update(self):
        for i in self._list:
            i.update()
