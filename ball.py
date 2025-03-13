
from PySide6.QtCore import Property, QObject, QPropertyAnimation, Signal, QPoint, Qt, QTimer, QEvent
from PySide6.QtGui import QGuiApplication, QMatrix4x4, QQuaternion, QVector3D, QColor, QSurfaceFormat, QCursor, \
    QVector2D
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender

import pybullet


class Ball:

    root_entity = None
    size = None

    ball_list = list()

    mesh = None
    material = None

    def __init__(self, root_entity, size):
        self.root_entity = root_entity
        self.size = size

        self.mesh = Qt3DExtras.QSphereMesh()
        self.mesh.setRadius(self.size / 10)

        self.material = Qt3DExtras.QPhongMaterial(self.root_entity)
        self.material.setAmbient(QColor(255, 255, 255))
        self.material.setDiffuse(QColor(255, 255, 255))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)

    def create_ball(self, position, vector):
        body = pybullet.createMultiBody(
            baseMass=10,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE,
                radius=self.size * 0.02,
            ),
            basePosition=[position.x(), position.z(), position.y()],
        )
        pybullet.resetBaseVelocity(body, linearVelocity=[
            vector.x() * 5,
            vector.z() * 5,
            vector.y() * 5,
        ])
        pos, _ = pybullet.getBasePositionAndOrientation(body)

        entity = Qt3DCore.QEntity(self.root_entity)

        transform = Qt3DCore.QTransform(self.root_entity)
        transform.setTranslation(QVector3D(
            pos[0],
            pos[2],
            pos[1],
        ))

        entity.addComponent(self.mesh)
        entity.addComponent(self.material)
        entity.addComponent(transform)

        self.ball_list.append({
            "body": body,
            "entity": entity,
            "transform": transform,
        })

        print(body, len(self.ball_list), pos)

        if len(self.ball_list) > 10:
            pybullet.removeBody(self.ball_list[0]["body"])
            self.ball_list.pop(0)

    def update(self):
        for i, ball in enumerate(self.ball_list):
            pos, _ = pybullet.getBasePositionAndOrientation(ball["body"])
            self.ball_list[i]["transform"].setTranslation(QVector3D(pos[0], pos[2], pos[1]))
