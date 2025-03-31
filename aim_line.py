
from PySide6.QtGui import QVector3D, QColor
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras

import pybullet

from collision_group import CollisionGroup
from global_config import grid_size, fps
from viewport_manager import Layer


class AimLine:
    root_entity = None

    sim_body = None
    pos_list = list()
    sim_length = 0

    run_sim = False

    dot_list = list()
    dot_feq = 2

    showing = False

    def __init__(self, root_entity):
        self.root_entity = root_entity
        self.sim_length = fps * 10

        self.sim_body = pybullet.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE,
                radius=grid_size * 0.02,
            ),
        )
        pybullet.changeDynamics(self.sim_body, -1, restitution=0.9)

        pybullet.setCollisionFilterGroupMask(
            self.sim_body, -1, CollisionGroup.get_group("aim_line"), CollisionGroup.get_mask("aim_line"))

    def update(self):
        if self.run_sim:
            if len(self.pos_list) < self.sim_length:
                pos, _ = pybullet.getBasePositionAndOrientation(self.sim_body)
                self.pos_list.append(QVector3D(pos[0], pos[2], pos[1]))
                self.dot_update()

    def enable_sim(self, position, vector):
        if self.run_sim is False:
            pybullet.resetBasePositionAndOrientation(self.sim_body, [
                position.x(),
                position.z(),
                position.y(),
            ], [0, 0, 0, 1])
            pybullet.resetBaseVelocity(self.sim_body, linearVelocity=[
                vector.x() * 25,
                vector.z() * 25,
                vector.y() * 25,
            ])
            self.run_sim = True

    def cancel_sim(self):
        if self.run_sim is True:
            self.run_sim = False
            self.pos_list.clear()

    def dot_update(self):
        for i, pos in enumerate(self.pos_list):
            if i % self.dot_feq == 0:
                dot_index = i // self.dot_feq
                if dot_index < len(self.dot_list):
                    self.dot_list[dot_index].transform.setTranslation(pos)
                else:
                    self.dot_list.append(Dot(self.root_entity, pos))

                self.dot_list[dot_index].pos_set = True
                if self.showing:
                    self.dot_list[i // self.dot_feq].entity.setEnabled(True)

        for i, dot in enumerate(self.dot_list):
            if i * self.dot_feq >= len(self.pos_list):
                dot.pos_set = False
                if self.showing:
                    dot.entity.setEnabled(False)

    def set_show(self):
        self.showing = True
        for dot in self.dot_list:
            if dot.pos_set:
                dot.entity.setEnabled(True)
            else:
                dot.entity.setEnabled(False)

    def set_hide(self):
        self.showing = False
        for dot in self.dot_list:
            dot.entity.setEnabled(False)


class DotMesh(Qt3DExtras.QSphereMesh):

    _mesh = None

    def __new__(cls, *args, **kwargs):
        if cls._mesh is None:
            cls._mesh = super().__new__(cls)
        return cls._mesh

    def __init__(self):
        if not hasattr(self, "_initialized"):
            super().__init__()
            self._initialized = True
            self.setRadius(grid_size * 0.02)


class DotMaterial(Qt3DExtras.QPhongAlphaMaterial):
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


class Dot:

    entity = None
    mesh = None
    material = None
    transform = None

    pos_set = False

    def __init__(self, root_entity, position):

        self.mesh = DotMesh()

        self.material = DotMaterial()

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(position)

        self.entity = Qt3DCore.QEntity(root_entity)
        self.entity.setEnabled(False)

        self.entity.addComponent(self.mesh)
        self.entity.addComponent(self.material)
        self.entity.addComponent(self.transform)

        self.entity.addComponent(Layer().get("scene"))
