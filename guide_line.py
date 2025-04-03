
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtGui import QVector3D, QColor

import math
import random

from global_config import grid_size, root_entity
from viewport_manager import Layer


class DashMaterial(Qt3DExtras.QPhongAlphaMaterial):
    color_value_list = [
        2 ** 6 - 1,
        2 ** 7 - 1,
        2 ** 8 - 1,
    ]

    def __init__(self):
        super().__init__()
        self.setAmbient(QColor(
            random.choice(self.color_value_list),
            random.choice(self.color_value_list),
            random.choice(self.color_value_list),
        ))
        self.setDiffuse(QColor(0, 0, 0))
        self.setSpecular(QColor(0, 0, 0))
        self.setShininess(0)
        self.setAlpha(0.5)


class DashMesh(Qt3DExtras.QCuboidMesh):
    _mesh = None

    size_ratio = 0.01

    def __new__(cls, *args, **kwargs):
        if cls._mesh is None:
            cls._mesh = super().__new__(cls)
        return cls._mesh

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True
            self.setXExtent(grid_size * (1 + self.size_ratio / 2))
            self.setYExtent(grid_size * self.size_ratio)
            self.setZExtent(grid_size * self.size_ratio)


class CubeMesh(Qt3DExtras.QCuboidMesh):
    _mesh = None

    size_ratio = 0.01

    def __new__(cls, *args, **kwargs):
        if cls._mesh is None:
            cls._mesh = super().__new__(cls)
        return cls._mesh

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True
            self.setXExtent(grid_size * self.size_ratio)
            self.setYExtent(grid_size * self.size_ratio)
            self.setZExtent(grid_size * self.size_ratio)


class Cube:
    entity = None
    mesh = None
    material = None
    transform = None

    def __init__(self, position, material):
        self.mesh = CubeMesh()
        self.material = material
        self.transform = Qt3DCore.QTransform(translation=position)

        self.entity = Qt3DCore.QEntity(root_entity)
        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.material)
        self.entity.addComponent(self.transform)

        self.entity.addComponent(Layer().get("scene"))


class Dash:
    entity = None
    mesh = None
    material = None
    transform = None

    direction = None

    cube_list = None
    cube_material = None
    count = 16
    distance = grid_size / count
    offset = 0

    def __init__(self, position, material):
        self.mesh = DashMesh()
        self.material = material
        self.transform = Qt3DCore.QTransform(translation=position)

        self.entity = Qt3DCore.QEntity(root_entity)
        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.material)
        self.entity.addComponent(self.transform)

        self.entity.addComponent(Layer().get("scene"))

        self.cube_list = list()

    def set_direction(self, vector):
        self.direction = vector
        self.transform.setRotationY(-90 + math.degrees(math.atan2(self.direction.x(), self.direction.z())))

    def add_cube(self, material):
        for i in range(self.count):
            pos = self.distance * (i - self.count / 2)
            self.cube_list.append(Cube(
                self.transform.translation() + self.direction * pos,
                material,
            ))

    def update(self):
        for i in range(self.count):
            pos = self.distance * (i - self.count / 2) + self.offset
            self.cube_list[i].transform.setTranslation(self.transform.translation() + self.direction * pos)

        self.offset = (self.offset + 0.01) % self.distance


class GuideLine:
    dash_list = None

    def __init__(self, maze_v_list, indent=0):
        material = DashMaterial()
        cube_material = DashMaterial()
        cube_material.setAmbient(material.ambient())
        cube_material.setAlpha(0.7)

        self.dash_list = list()

        for i, v in enumerate(maze_v_list):
            if i == 0:
                continue

            v2 = maze_v_list[i - 1]
            position = QVector3D(
                grid_size * ((v.x + v2.x) / 2 + 0.5),
                grid_size * (0.1 + 0.02 * indent),
                grid_size * ((v.y + v2.y) / 2 + 0.5),
            )

            dash = Dash(position, material)
            dash.set_direction((QVector3D(v.x, 0, v.y) - QVector3D(v2.x, 0, v2.y)).normalized())
            dash.add_cube(cube_material)
            self.dash_list.append(dash)

    def update(self):
        for dash in self.dash_list:
            dash.update()
