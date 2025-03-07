
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QColor


class Wall:

    entity = None
    mesh = None
    transform = None
    material = None

    def __init__(self, root_entity, grid_size):

        self.entity = Qt3DCore.QEntity(root_entity)

        self.mesh = Qt3DExtras.QCuboidMesh()
        self.mesh.setXExtent(grid_size * 1.0)
        self.mesh.setYExtent(grid_size)
        self.mesh.setZExtent(grid_size * 0.1)

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(QVector3D(0, 32, 0))

        self.material = Qt3DExtras.QPhongMaterial(root_entity)
        self.material.setAmbient(QColor(0, 0, 63))
        self.material.setDiffuse(QColor(127, 127, 127))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.transform)
        # self.entity.addComponent(self.material)
        # self.entity.addComponent(texture_material)
