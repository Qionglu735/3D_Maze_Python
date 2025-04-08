
from PySide6.QtGui import QVector3D, QColor
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras

import pybullet
import random

from collision_group import CollisionGroup
from global_config import grid_size, root_entity
from viewport_manager import Layer


class Ingredient:

    entity = None
    mesh = None
    material = None
    transform = None
    body = None

    def __init__(self, position, vector):
        self.root_entity = root_entity

        self.mesh = Qt3DExtras.QSphereMesh()
        self.mesh.setRadius(grid_size * 0.2)

        self.material = Qt3DExtras.QPhongMaterial()
        color_list = [0, 255]
        color = QColor(random.choice(color_list), random.choice(color_list), random.choice(color_list))
        self.material.setAmbient(color)
        self.material.setDiffuse(color)
        self.material.setSpecular(color)
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
        self.entity.addComponent(Layer().get("alchemy"))

    def update(self):
        pos, _ = pybullet.getBasePositionAndOrientation(self.body)
        self.transform.setTranslation(QVector3D(pos[0], pos[2], pos[1]))


class IngredientList:
    _list = list()

    body_list = list()

    def __init__(self):
        pass

    def create_ingredient(self, position, vector):
        self._list.append(Ingredient(position, vector))
        self.body_list.append(self._list[-1].body)

        if len(self._list) > 20:
            pybullet.removeBody(self._list[0].body)
            self._list[0].entity.setParent(None)
            self._list[0].entity.deleteLater()
            self._list.pop(0)

    def update(self):
        for i in self._list:
            i.update()
