
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QColor


class Coordinate:

    x_cube_list = list()
    y_cube_list = list()
    z_cube_list = list()

    def __init__(self, root_entity, size):

        self.o_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 0.35, yExtent=size * 0.35, zExtent=size * 0.35)
        self.x_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 200, yExtent=size * 0.15, zExtent=size * 0.15)
        self.y_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 0.15, yExtent=size * 200, zExtent=size * 0.15)
        self.z_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 0.15, yExtent=size * 0.15, zExtent=size * 200)

        self.cube_mesh = Qt3DExtras.QCuboidMesh(xExtent=size * 0.20, yExtent=size * 0.20, zExtent=size * 0.20)

        self.o_material = Qt3DExtras.QPhongMaterial(root_entity)
        self.x_material = Qt3DExtras.QPhongMaterial(root_entity)
        self.y_material = Qt3DExtras.QPhongMaterial(root_entity)
        self.z_material = Qt3DExtras.QPhongMaterial(root_entity)

        self.o_material.setAmbient(QColor(127, 127, 127))
        self.x_material.setAmbient(QColor(255, 0, 0))
        self.y_material.setAmbient(QColor(0, 255, 0))
        self.z_material.setAmbient(QColor(0, 0, 255))

        self.x_transform = Qt3DCore.QTransform(translation=QVector3D(0, 0, 0))
        self.y_transform = Qt3DCore.QTransform(translation=QVector3D(0, 0, 0))
        self.z_transform = Qt3DCore.QTransform(translation=QVector3D(0, 0, 0))

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

        self.count = 200
        self.distance = size
        for i in range(self.count):
            self.x_cube_list.append({
                "entity": Qt3DCore.QEntity(root_entity),
                "transform":  Qt3DCore.QTransform(
                    translation=QVector3D(self.distance * (i - self.count / 2), 0, 0))
            })
            self.x_cube_list[i]["entity"].addComponent(self.cube_mesh)
            self.x_cube_list[i]["entity"].addComponent(self.x_material)
            self.x_cube_list[i]["entity"].addComponent(self.x_cube_list[i]["transform"])

            self.y_cube_list.append({
                "entity": Qt3DCore.QEntity(root_entity),
                "transform":  Qt3DCore.QTransform(
                    translation=QVector3D(0, self.distance * (i - self.count / 2), 0))
            })
            self.y_cube_list[i]["entity"].addComponent(self.cube_mesh)
            self.y_cube_list[i]["entity"].addComponent(self.y_material)
            self.y_cube_list[i]["entity"].addComponent(self.y_cube_list[i]["transform"])

            self.z_cube_list.append({
                "entity": Qt3DCore.QEntity(root_entity),
                "transform":  Qt3DCore.QTransform(
                    translation=QVector3D(0, 0, self.distance * (i - self.count / 2)))
            })
            self.z_cube_list[i]["entity"].addComponent(self.cube_mesh)
            self.z_cube_list[i]["entity"].addComponent(self.z_material)
            self.z_cube_list[i]["entity"].addComponent(self.z_cube_list[i]["transform"])

        self.offset = 0

        self.update_skip_counter = 0

    def update(self):
        for i in range(self.count):
            pos = self.distance * (i - self.count / 2) + self.offset
            self.x_cube_list[i]["transform"].setTranslation(QVector3D(pos, 0, 0))
            self.y_cube_list[i]["transform"].setTranslation(QVector3D(0, pos, 0))
            self.z_cube_list[i]["transform"].setTranslation(QVector3D(0, 0, pos))

        self.offset = (self.offset + 0.5) % self.distance


