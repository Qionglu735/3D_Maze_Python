
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QColor, QQuaternion


class Coordinate:

    def __init__(self, root_entity, size):

        self.o_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 0.35, yExtent=size * 0.35, zExtent=size * 0.35)
        self.x_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 200, yExtent=size * 0.15, zExtent=size * 0.15)
        self.y_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 0.15, yExtent=size * 200, zExtent=size * 0.15)
        self.z_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 0.15, yExtent=size * 0.15, zExtent=size * 200)

        self.o_material = Qt3DExtras.QPhongMaterial(root_entity)
        self.x_material = Qt3DExtras.QPhongMaterial(root_entity)
        self.y_material = Qt3DExtras.QPhongMaterial(root_entity)
        self.z_material = Qt3DExtras.QPhongMaterial(root_entity)

        self.o_material.setAmbient(QColor(127, 127, 127))
        self.x_material.setAmbient(QColor(255, 0, 0))
        self.y_material.setAmbient(QColor(0, 255, 0))
        self.z_material.setAmbient(QColor(0, 0, 255))

        self.x_transform = Qt3DCore.QTransform(translation=QVector3D(size * 50, 0, 0))
        self.y_transform = Qt3DCore.QTransform(translation=QVector3D(0, size * 50, 0))
        self.z_transform = Qt3DCore.QTransform(translation=QVector3D(0, 0, size * 50))

        self.o_entity = Qt3DCore.QEntity(root_entity)
        self.x_entity = Qt3DCore.QEntity(root_entity)
        self.y_entity = Qt3DCore.QEntity(root_entity)
        self.z_entity = Qt3DCore.QEntity(root_entity)

        self.o_entity.addComponent(self.o_mesh)
        self.x_entity.addComponent(self.x_mesh)
        self.y_entity.addComponent(self.y_mesh)
        self.z_entity.addComponent(self.z_mesh)

        self.o_entity.addComponent(self.o_material)
        self.x_entity.addComponent(self.x_material)
        self.y_entity.addComponent(self.y_material)
        self.z_entity.addComponent(self.z_material)

        self.x_entity.addComponent(self.x_transform)
        self.y_entity.addComponent(self.y_transform)
        self.z_entity.addComponent(self.z_transform)
