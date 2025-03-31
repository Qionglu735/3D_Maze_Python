
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras

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
            self.setXExtent(grid_size * 1.1)
            self.setYExtent(grid_size)
            self.setZExtent(grid_size * 0.1)


class WallMaterial(Qt3DExtras.QTextureMaterial):
    _texture = None
    _material = None

    def __new__(cls, root_entity, *args, **kwargs):
        if cls._material is None:
            # cls._material = super().__new__(cls)
            cls._texture = Texture(root_entity)
            cls._texture.generate_random_texture()
            cls._material = cls._texture.create_material()
        return cls._material

    def __init__(self, root_entity):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True


class Wall:

    entity = None
    mesh = None
    transform = None
    material = None

    body = None

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
        self.entity.addComponent(Layer().get("ui"))

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
