
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QColor, QQuaternion

import math
import pybullet

from collision_group import CollisionGroup
from global_config import grid_size, fps


class Target:
    entity = None
    mesh = None
    transform = None
    material = None

    def __init__(self, root_entity, position, rotation):
        self.body = pybullet.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_CYLINDER,
                radius=grid_size * 0.2,
                height=grid_size * 0.01,
            ),
            basePosition=[position.x(), position.z(), position.y()],
            baseOrientation=pybullet.getQuaternionFromEuler([x / 180 * math.pi for x in [
                rotation.x(), rotation.z(), rotation.y(),
            ]]),
        )
        pybullet.changeDynamics(self.body, -1, restitution=0.9)
        pybullet.setCollisionFilterGroupMask(
            self.body, -1, CollisionGroup.get_group("target"), CollisionGroup.get_mask("target"))

        pos, ori = pybullet.getBasePositionAndOrientation(self.body)
        ori_euler = [x / math.pi * 180 for x in pybullet.getEulerFromQuaternion(ori)]
        # print(ori, ori_euler)

        self.mesh = Qt3DExtras.QCylinderMesh(root_entity)
        self.mesh.setRadius(grid_size * 0.2)
        self.mesh.setLength(grid_size * 0.01)
        self.mesh.setRings(16)
        self.mesh.setSlices(360)

        self.transform = Qt3DCore.QTransform(root_entity)
        self.transform.setTranslation(QVector3D(pos[0], pos[2], pos[1]))
        self.transform.setRotation(QQuaternion.fromEulerAngles(QVector3D(ori_euler[0], ori_euler[2], ori_euler[1])))
        # print(self.transform.rotation(), self.transform.rotation().toEulerAngles())

        self.material = Qt3DExtras.QPhongMaterial(root_entity)
        self.material.setAmbient(QColor(127, 127, 63))
        self.material.setDiffuse(QColor(0, 0, 0))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)

        self.entity = Qt3DCore.QEntity(root_entity)
        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.transform)
        self.entity.addComponent(self.material)

    def update(self):
        pos, ori = pybullet.getBasePositionAndOrientation(self.body)
        ori_euler = list(pybullet.getEulerFromQuaternion(ori))
        ori_euler_degree = [x / math.pi * 180 for x in ori_euler]
        self.transform.setTranslation(QVector3D(pos[0], pos[2], pos[1]))
        self.transform.setRotation(QQuaternion.fromEulerAngles(QVector3D(
            ori_euler_degree[0], ori_euler_degree[2], ori_euler_degree[1],
        )))
        rotate_degree = 90 / fps
        ori_euler[2] += rotate_degree / 180 * math.pi
        pybullet.resetBasePositionAndOrientation(self.body, pos, pybullet.getQuaternionFromEuler(ori_euler))
        pybullet.resetBaseVelocity(self.body, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
