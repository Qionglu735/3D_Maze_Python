
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtGui import QVector3D, QColor

import pybullet

from collision_group import CollisionGroup
from global_config import grid_size, maze_size, root_entity
from texture import Texture
from viewport_manager import Layer


class GroundMaterial(Qt3DExtras.QTextureMaterial):
    _texture = None
    _material = None

    def __new__(cls, *args, **kwargs):
        if cls._material is None:
            # cls._material = super().__new__(cls)
            cls._texture = Texture(color="green", size=grid_size * maze_size * 16)
            cls._texture.generate_random_texture()
            cls._material = cls._texture.create_material()
        return cls._material

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True

    @classmethod
    def before_exit(cls):
        if cls._texture is not None:
            del cls._texture
            cls._texture = None


class Ground:

    entity = None
    mesh = None
    transform = None
    material = None

    body = None

    def __init__(self):

        self.entity = Qt3DCore.QEntity(root_entity)

        self.mesh = Qt3DExtras.QPlaneMesh()
        self.mesh.setWidth(grid_size * maze_size * 4)
        self.mesh.setHeight(grid_size * maze_size * 4)

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(QVector3D(0, 0, 0))

        # self.material = Qt3DExtras.QPhongMaterial()
        # self.material.setDiffuse(QColor(0, 63, 0))
        # self.material.setSpecular(QColor(0, 0, 0))
        # self.material.setShininess(0)

        self.material = GroundMaterial()

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.transform)
        self.entity.addComponent(self.material)

        self.entity.addComponent(Layer.get("scene"))

        self.body = pybullet.loadURDF("plane.urdf")
        pybullet.changeDynamics(self.body, -1, restitution=0.9)
        pybullet.setCollisionFilterGroupMask(
            self.body, -1, CollisionGroup.get_group("env"), CollisionGroup.get_mask("env"))

    @staticmethod
    def before_exit():
        GroundMaterial.before_exit()
