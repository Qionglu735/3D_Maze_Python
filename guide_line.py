
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtGui import QVector3D, QColor

from global_config import grid_size, root_entity
from viewport_manager import Layer


class DashMaterial(Qt3DExtras.QPhongAlphaMaterial):
    _material = None

    def __new__(cls, *args, **kwargs):
        if cls._material is None:
            cls._material = super().__new__(cls, *args, **kwargs)
        return cls._material

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True
            self.setAmbient(QColor(255, 255, 255))
            self.setDiffuse(QColor(255, 255, 255))
            self.setSpecular(QColor(0, 0, 0))
            self.setShininess(0)
            self.setAlpha(0.7)


class Dash:
    entity = None
    mesh = None
    material = None
    transform = None

    def __init__(self, position):
        self.mesh = Qt3DExtras.QCuboidMesh(xExtent=grid_size * 1.005, yExtent=grid_size * 0.01, zExtent=grid_size * 0.01)
        self.material = DashMaterial()
        self.transform = Qt3DCore.QTransform(translation=position)

        self.entity = Qt3DCore.QEntity(root_entity)
        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.material)
        self.entity.addComponent(self.transform)

        self.entity.addComponent(Layer().get("scene"))


class GuideLine:
    dash_list = list()

    def __init__(self, maze_v_list):
        for i, v in enumerate(maze_v_list):
            if i == 0:
                continue

            v2 = maze_v_list[i - 1]
            position = QVector3D(
                grid_size * ((v.x + v2.x) / 2 + 0.5),
                grid_size * 0.1,
                grid_size * ((v.y + v2.y) / 2 + 0.5),
            )

            self.dash_list.append(Dash(position))

            if v.x == v2.x:
                self.dash_list[-1].transform.setRotationY(90)





