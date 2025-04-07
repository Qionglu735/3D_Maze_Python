
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QColor

import math
import pybullet

from collision_group import CollisionGroup
from global_config import grid_size
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
            self.setXExtent(grid_size + wall_thickness - 0.1)
            self.setYExtent(grid_size)
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


class WallMaterial(Qt3DExtras.QTextureMaterial):
    _texture = None
    _material = None

    def __new__(cls, root_entity, *args, **kwargs):
        if cls._material is None:
            # cls._material = super().__new__(cls)
            cls._texture = Texture()
            cls._texture.generate_random_texture()
            cls._material = cls._texture.create_material()
        return cls._material

    def __init__(self, root_entity):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True

    @classmethod
    def before_exit(cls):
        if cls._texture is not None:
            del cls._texture
            cls._texture = None


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

    def __init__(self, root_entity, position, rotation):

        self.mesh = WallMesh()

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(position)
        self.transform.setRotation(rotation)

        self.material = WallMaterial(root_entity)

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

        self.entity_map.addComponent(Layer().get("ui"))

    @staticmethod
    def before_exit():
        WallMaterial.before_exit()
