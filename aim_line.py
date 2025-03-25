
from PySide6.QtCore import Property, QObject, QPropertyAnimation, Signal, QPoint, Qt, QTimer, QEvent
from PySide6.QtGui import QGuiApplication, QMatrix4x4, QQuaternion, QVector3D, QColor, QSurfaceFormat, QCursor, \
    QVector2D
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender

import pybullet

from collision_group import CollisionGroup


class AimLine:
    root_entity = None
    size = 10

    sim_body = None
    pos_list = list()
    sim_length = 3000

    dot_list = list()
    dot_feq = 2

    showing = False

    def __init__(self, root_entity, size, fps):
        self.root_entity = root_entity
        self.size = size
        self.sim_length = fps * 10

        self.sim_body = pybullet.createMultiBody(
            baseMass=10,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE,
                radius=self.size * 0.02,
            ),
        )
        pybullet.changeDynamics(self.sim_body, -1, restitution=0.9)

        pybullet.setCollisionFilterGroupMask(
            self.sim_body, -1, CollisionGroup.get_group("aim_line"), CollisionGroup.get_mask("aim_line"))

        # for i in range(self.sim_length // self.dot_feq):
        #     self.dot_list.append(Dot(self.root_entity, self.size, QVector3D(0, 0, 0)))

    def update(self):
        if len(self.pos_list) < self.sim_length:
            pos, _ = pybullet.getBasePositionAndOrientation(self.sim_body)
            self.pos_list.append(QVector3D(pos[0], pos[2], pos[1]))
            self.dot_update()

    def set_pos(self, position, vector):
        pybullet.resetBasePositionAndOrientation(self.sim_body, [
            position.x(),
            position.z(),
            position.y(),
        ], [0, 0, 0, 1])
        pybullet.resetBaseVelocity(self.sim_body, linearVelocity=[
            vector.x() * 25,
            vector.z() * 25,
            vector.y() * 25,
        ])
        self.pos_list.clear()

    def dot_update(self):
        for i, pos in enumerate(self.pos_list):
            if i % self.dot_feq == 0:
                if len(self.dot_list) > i // self.dot_feq:
                    self.dot_list[i // self.dot_feq].transform.setTranslation(pos)
                else:
                    self.dot_list.append(Dot(self.root_entity, self.size, pos))

                self.dot_list[i // self.dot_feq].pos_set = True
                if self.showing:
                    self.dot_list[i // self.dot_feq].entity.setEnabled(True)

        for i, dot in enumerate(self.dot_list):
            if i * self.dot_feq >= len(self.pos_list):
                dot.pos_set = False
                if self.showing:
                    dot.entity.setEnabled(False)

    def show(self):
        for dot in self.dot_list:
            if dot.pos_set:
                dot.entity.setEnabled(True)
            else:
                dot.entity.setEnabled(False)

    def set_show(self):
        # print(len(self.pos_list), len(self.dot_list), len([x for x in self.dot_list if x.pos_set]))
        self.showing = True
        self.show()

    def hide(self):
        for dot in self.dot_list:
            dot.entity.setEnabled(False)

    def set_hide(self):
        self.showing = False
        self.hide()


class Dot:

    root_entity = None
    size = None

    entity = None
    mesh = None
    material = None
    transform = None

    pos_set = False

    def __init__(self, root_entity, size, position):
        self.root_entity = root_entity
        self.size = size

        self.mesh = Qt3DExtras.QSphereMesh(self.root_entity)
        self.mesh.setRadius(self.size * 0.02)

        self.material = Qt3DExtras.QPhongAlphaMaterial(self.root_entity)
        self.material.setAmbient(QColor(255, 255, 255))
        self.material.setDiffuse(QColor(255, 255, 255))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)
        self.material.setAlpha(0.7)

        self.transform = Qt3DCore.QTransform(self.root_entity)
        self.transform.setTranslation(position)

        self.entity = Qt3DCore.QEntity(self.root_entity)
        self.entity.setEnabled(False)

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.material)
        self.entity.addComponent(self.transform)


