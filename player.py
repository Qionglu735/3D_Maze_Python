
import pybullet


class Player:
    mass = 10

    body = None

    def __init__(self, grid_size):
        self.mass = grid_size
        self.body = pybullet.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_CAPSULE,
                radius=grid_size * 0.1,
                height=grid_size * 0.5,
            ),
            basePosition=[
                grid_size * 0.5,
                grid_size * 0.5,
                grid_size * 0.5,
            ],
        )
        pybullet.changeDynamics(
            self.body, -1,
            angularDamping=0.0,
            linearDamping=1.0,
            localInertiaDiagonal=[1e9, 1e9, 0],
        )
