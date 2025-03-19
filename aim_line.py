
from PySide6.QtCore import Property, QObject, QPropertyAnimation, Signal, QPoint, Qt, QTimer, QEvent
from PySide6.QtGui import QGuiApplication, QMatrix4x4, QQuaternion, QVector3D, QColor, QSurfaceFormat, QCursor, \
    QVector2D
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender

import pybullet


class AimLine:
    root_entity = None
    size = 10

    sim_body = None
    pos_list = list()

    line_length = 50

    dot_list = list()

    def __init__(self, root_entity, size):
        self.root_entity = root_entity
        self.size = size

        self.sim_body = pybullet.createMultiBody(
            baseMass=10,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE,
                radius=self.size * 0.02,
            ),
        )

    def update(self):
        if len(self.pos_list) < self.line_length:
            pos, _ = pybullet.getBasePositionAndOrientation(self.sim_body)
            self.pos_list.append(QVector3D(pos[0], pos[2], pos[1]))
            print(len(self.pos_list))
            if len(self.pos_list) == self.line_length and len(self.dot_list) == self.line_length:
                self.draw_update()

    def set_pos(self, position, vector):
        pybullet.resetBasePositionAndOrientation(self.sim_body, [
            position.x(),
            position.z(),
            position.y(),
        ], [0, 0, 0, 1])
        pybullet.resetBaseVelocity(self.sim_body, linearVelocity=[
            vector.x() * 15,
            vector.z() * 15,
            vector.y() * 15,
        ])
        self.pos_list.clear()

    def draw(self):
        print(self.pos_list)
        for i, pos in enumerate(self.pos_list):
            print(pos)
            self.dot_list.append(Dot(self.root_entity, self.size, pos))

    def draw_update(self):
        for i, pos in enumerate(self.pos_list):
            self.dot_list[i].transform.setTranslation(pos)

    def clear(self):
        for i in self.dot_list:
            i.clear()
        self.dot_list.clear()


class Dot:
    root_entity = None
    size = None

    entity = None
    mesh = None
    material = None
    transform = None

    def __init__(self, root_entity, size, position):
        self.root_entity = root_entity
        self.size = size

        self.mesh = Qt3DExtras.QSphereMesh(self.root_entity)
        self.mesh.setRadius(self.size * 0.02)

        self.material = Qt3DExtras.QPhongMaterial(self.root_entity)
        self.material.setAmbient(QColor(255, 255, 255))
        self.material.setDiffuse(QColor(255, 255, 255))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)

        self.transform = Qt3DCore.QTransform(self.root_entity)
        self.transform.setTranslation(position)

        self.entity = Qt3DCore.QEntity(self.root_entity)

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.material)
        self.entity.addComponent(self.transform)

    def clear(self):
        self.entity.setParent(None)
        self.entity.deleteLater()
