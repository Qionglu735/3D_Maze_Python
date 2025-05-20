
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtGui import QColor

import math
import random
import pybullet

from collision_group import CollisionGroup
from global_config import grid_size, wall_height
from texture import Texture
from viewport_manager import Layer


class WallMesh(Qt3DExtras.QCuboidMesh):

    _mesh = None

    def __new__(cls, *args, **kwargs):
        if cls._mesh is None:
            cls._mesh = super().__new__(cls)
        return cls._mesh

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True

            wall_thickness = grid_size * 0.1
            self.setXExtent(grid_size + wall_thickness * 0.0)
            self.setYExtent(wall_height)
            self.setZExtent(wall_thickness)


class WallMeshForMap(Qt3DExtras.QCuboidMesh):

    _mesh = None

    def __new__(cls, *args, **kwargs):
        if cls._mesh is None:
            cls._mesh = super().__new__(cls)
        return cls._mesh

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True
            self.setXExtent(grid_size * 1.2)
            self.setYExtent(1)
            self.setZExtent(grid_size * 0.3)


class WallMaterial:
    _texture = None
    _material = None

    def __init__(self):
        self._texture = Texture(size=grid_size * 64)
        self._texture.generate_random_texture()
        self._material = self._texture.create_material()

    def material(self):
        return self._material

    def before_exit(self):
        if self._texture is not None:
            del self._texture
            self._texture = None


class WallMaterialForMap(Qt3DExtras.QPhongMaterial):

    _material = None

    def __new__(cls, *args, **kwargs):
        if cls._material is None:
            cls._material = super().__new__(cls)
        return cls._material

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True
            self.setAmbient(QColor(255, 255, 255))
            self.setDiffuse(QColor(0, 0, 0))
            self.setSpecular(QColor(0, 0, 0))
            self.setShininess(0)


class Wall:

    entity = None
    mesh = None
    transform = None
    material = None

    body = None

    entity_map = None
    mesh_map = None
    transform_map = None
    material_map = None

    def __init__(self, root_entity, material, position, rotation):

        self.mesh = WallMesh()

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(position)
        self.transform.setRotation(rotation)

        self.material = material

        self.entity = Qt3DCore.QEntity(root_entity)
        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.transform)
        self.entity.addComponent(self.material)

        self.entity.addComponent(Layer().get("scene"))

        self.body = pybullet.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_BOX,
                halfExtents=[
                    self.mesh.xExtent() / 2,
                    self.mesh.zExtent() / 2,
                    self.mesh.yExtent() / 2,
                ],
            ),
            basePosition=[
                self.transform.translation().x(),
                self.transform.translation().z(),
                self.transform.translation().y(),
            ],
            baseOrientation=pybullet.getQuaternionFromEuler([
                self.transform.rotationX() / 180 * math.pi,
                self.transform.rotationZ() / 180 * math.pi,
                self.transform.rotationY() / 180 * math.pi,
            ]),
        )
        pybullet.changeDynamics(self.body, -1, restitution=0.9)
        pybullet.setCollisionFilterGroupMask(
            self.body, -1, CollisionGroup.get_group("env"), CollisionGroup.get_mask("env"))

        self.mesh_map = WallMeshForMap()

        self.transform_map = Qt3DCore.QTransform()
        self.transform_map.setTranslation(position)
        self.transform_map.setRotation(rotation)

        self.material_map = WallMaterialForMap()

        self.entity_map = Qt3DCore.QEntity(root_entity)
        self.entity_map.addComponent(self.mesh_map)
        self.entity_map.addComponent(self.transform_map)
        self.entity_map.addComponent(self.material_map)

        self.picker = Qt3DRender.QObjectPicker(root_entity)
        self.picker.setHoverEnabled(True)
        self.picker.clicked.connect(self.on_picker_clicked)
        self.entity.addComponent(self.picker)

        self.entity_map.addComponent(Layer().get("ui"))

    def on_picker_clicked(self, event):
        pass
        # print("wall", event)
        # if event.button() == Qt3DRender.QPickEvent.Buttons.LeftButton:
        #     print("Cube clicked at:", event.position())


class WallList:
    _list = None

    root_entity = None
    wall_material_list = None

    def __init__(self, root_entity):
        self._list = list()

        self.root_entity = root_entity
        self.wall_material_list = [WallMaterial() for _ in range(8)]

    def create_wall(self, position, rotation):
        self._list.append(Wall(
            self.root_entity,
            random.choice(self.wall_material_list).material(),
            position,
            rotation,
        ))

    def before_exit(self):
        for wall_material in self.wall_material_list:
            wall_material.before_exit()
