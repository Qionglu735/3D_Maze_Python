
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QColor


class Ground:

    entity = None
    mesh = None
    transform = None
    material = None

    def __init__(self, root_entity):

        self.entity = Qt3DCore.QEntity(root_entity)

        self.mesh = Qt3DExtras.QPlaneMesh()
        self.mesh.setWidth(1024)
        self.mesh.setHeight(1024)

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(QVector3D(0, 0, 0))

        self.material = Qt3DExtras.QPhongMaterial(root_entity)
        # self.material.setAmbient(QColor(0, 127, 0))
        self.material.setDiffuse(QColor(0, 63, 0))
        self.material.setSpecular(QColor(0, 0, 0))
        self.material.setShininess(0)

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.transform)
        self.entity.addComponent(self.material)
