
from PySide6.QtCore import Property, QObject, QPropertyAnimation, Signal, QPoint, Qt, QTimer, QEvent
from PySide6.QtGui import QGuiApplication, QMatrix4x4, QQuaternion, QVector3D, QColor, QSurfaceFormat, QCursor, \
    QVector2D
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender

import pybullet
import pybullet_data

import math
import random
import sys

random.seed(1001)


class OrbitTransformController(QObject):
    def __init__(self, parent):
        super().__init__(parent)
        self._target = None
        self._matrix = QMatrix4x4()
        self._radius = 1
        self._angle = 0

    def setTarget(self, t):
        self._target = t

    def getTarget(self):
        return self._target

    def setRadius(self, radius):
        if self._radius != radius:
            self._radius = radius
            self.updateMatrix()
            self.radiusChanged.emit()

    def getRadius(self):
        return self._radius

    def setAngle(self, angle):
        if self._angle != angle:
            self._angle = angle
            self.updateMatrix()
            self.angleChanged.emit()

    def getAngle(self):
        return self._angle

    def updateMatrix(self):
        self._matrix.setToIdentity()
        self._matrix.rotate(self._angle, QVector3D(0, 1, 0))
        self._matrix.translate(self._radius, 0, 0)
        if self._target is not None:
            self._target.setMatrix(self._matrix)

    angleChanged = Signal()
    radiusChanged = Signal()
    angle = Property(float, getAngle, setAngle, notify=angleChanged)
    radius = Property(float, getRadius, setRadius, notify=radiusChanged)


grid_size = 10
maze_size = 5

# fps = 1
# fps = 10
fps = 30
# fps = 60


class Window(Qt3DExtras.Qt3DWindow):

    camera_index = 0
    camera_list = [None for _ in range(8)]

    coordinate = None
    ground = None
    wall_list = list()

    text_list = list()

    yaw_angle = 0
    pitch_angle = 0

    def __init__(self):
        super().__init__()

        self.light_transform = None
        self.directional_light = None
        self.directional_light_entity = None

        self.setWidth(160)
        self.setHeight(90)

        # self.setWidth(640)
        # self.setHeight(360)

        self.setPosition(QPoint(
            int(QGuiApplication.primaryScreen().size().width() * 0.65),
            int(QGuiApplication.primaryScreen().size().height() * 0.83),
            # int(QGuiApplication.primaryScreen().size().height() * 0.23),
        ))

        self.defaultFrameGraph().setClearColor(QColor(63, 63, 63))

        surface_format = QSurfaceFormat()
        surface_format.setSamples(4)  # 设置抗锯齿
        self.setFormat(surface_format)

        # 物理模拟
        self.physics = BulletPhysics()

        # Camera
        self.camera_list = [CameraSave() for _ in range(8)]
        self.camera_list[0].save(self.camera())

        self.camera().lens().setPerspectiveProjection(45, 16 / 9, 0.1, 1000)

        camera_1 = self.camera_list[1]
        camera_1.position = QVector3D(grid_size * maze_size / 2, grid_size * maze_size * 2, grid_size * maze_size)
        camera_1.view_center = QVector3D(grid_size * maze_size / 2, 0, grid_size * maze_size / 2)

        camera_2 = self.camera_list[2]
        camera_2.position = QVector3D(grid_size / 2, grid_size / 2, grid_size / 2)
        camera_2.view_center = QVector3D(grid_size, grid_size / 2, grid_size / 2)
        camera_2.up_vector = QVector3D(0, 1, 0)

        camera_3 = self.camera_list[3]
        camera_3.position = QVector3D(100, 100, 100)
        camera_3.view_center = QVector3D(0, 0, 0)
        camera_3.up_vector = QVector3D(0, 1, 0)

        # Root entity
        self.root_entity = Qt3DCore.QEntity()
        self.setRootEntity(self.root_entity)

        # For camera controls
        self.orbit_controller = Qt3DExtras.QOrbitCameraController(self.root_entity)
        self.orbit_controller.setLinearSpeed(180)
        self.orbit_controller.setLookSpeed(360)
        self.orbit_controller.setCamera(self.camera())

        self.fp_controller = Qt3DExtras.QFirstPersonCameraController(self.root_entity)
        self.fp_controller.setLinearSpeed(100)
        self.fp_controller.setLookSpeed(200)
        # self.fp_controller.setCamera(self.camera())

        # Light
        self.directional_light_entity = Qt3DCore.QEntity(self.root_entity)

        self.directional_light = Qt3DRender.QDirectionalLight()
        self.directional_light.setColor(QColor(255, 255, 255))
        self.directional_light.setIntensity(1.0)

        self.light_transform = Qt3DCore.QTransform()
        self.light_transform.setRotation(QQuaternion.fromEulerAngles(0, 50, 0))

        self.directional_light_entity.addComponent(self.directional_light)
        self.directional_light_entity.addComponent(self.light_transform)

        self.create_scene()
        from camera_control import CameraController

        self.controller = CameraController(self.camera())
        self.controller.last_cursor_pos = self.mapFromGlobal(QCursor.pos())

        from player import Player

        self.player = Player(grid_size)
        self.last_player_move_vector = None

        player_pos, _ = pybullet.getBasePositionAndOrientation(self.player.body)

        camera_2 = self.camera_list[2]
        camera_2.position = QVector3D(player_pos[0], player_pos[2], player_pos[1])
        camera_2.view_center = QVector3D(player_pos[0] + 1, player_pos[2], player_pos[1])
        camera_2.up_vector = QVector3D(0, 1, 0)

        self.camera_index = 1
        self.camera_list[self.camera_index].load(self.camera())

        self.sphere_body = pybullet.createMultiBody(
            baseMass=10,
            baseCollisionShapeIndex=pybullet.createCollisionShape(
                pybullet.GEOM_SPHERE,
                radius=grid_size * 0.05,
            ),
            basePosition=[grid_size / 2, grid_size / 2, grid_size * 2],
        )
        pybullet.resetBaseVelocity(self.sphere_body, linearVelocity=[grid_size * 4, 0, 0],)

        # self.ball = Ball(self.root_entity, grid_size)

        from ball import Ball
        self.ball = Ball(self.root_entity, grid_size)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_physics)
        self.timer.start(math.floor(1000 / fps))

    def update_physics(self):
        try:
            self.physics.step_simulation()
        except pybullet.error:
            self.timer.stop()
            self.close()

        self.coordinate.update()

        player_pos, player_ori = pybullet.getBasePositionAndOrientation(self.player.body)
        player_linear_vel, player_angular_vel = pybullet.getBaseVelocity(self.player.body)

        camera_pos = self.controller.camera.position()
        camera_view_center = self.controller.camera.viewCenter()

        if self.camera_index == 2:

            player_move_vector = self.controller.cal_movement_vector()
            if player_move_vector.length() > 0.01:
                force = [
                    player_move_vector.x() * self.player.mass,
                    player_move_vector.z() * self.player.mass,
                    player_move_vector.y() / 100 * self.player.mass,
                ]
                pybullet.applyExternalForce(self.player.body, -1, force, [0, 0, 0], pybullet.LINK_FRAME)

            player_pos, player_ori = pybullet.getBasePositionAndOrientation(self.player.body)

            camera_pos_new = QVector3D(
                player_pos[0],
                player_pos[2],
                player_pos[1],
            )
            self.controller.camera.setPosition(camera_pos_new)

            player_view_vector = self.controller.cal_view_vector(self.mapFromGlobal(QCursor.pos()))

            forward = (camera_view_center - camera_pos).normalized()
            right = QVector3D.crossProduct(forward, QVector3D(0, 1, 0)).normalized()
            up = QVector3D.crossProduct(right, forward).normalized()

            camera_view_vector = (
                forward
                + right * player_view_vector.x()
                + up * player_view_vector.y()
            ).normalized()
            camera_view_center_new = camera_pos_new + camera_view_vector

            self.camera().setViewCenter(camera_view_center_new)
            self.camera().setUpVector(QVector3D(0, 1, 0))

            camera_yaw = math.degrees(math.asin(camera_view_vector.x()))
            if camera_view_vector.z() < 0:
                camera_yaw *= -1

            self.center_cursor()
            self.controller.last_cursor_pos = self.mapFromGlobal(QCursor.pos())
        else:
            player_liner_vel_new = [0, 0, 0]
            camera_yaw = 0
            self.controller.update_camera_position()

        # pybullet.resetBaseVelocity(
        #     self.player.body,
        #     linearVelocity=player_liner_vel_new,
        #     # angularVelocity=[0, 0, angular_vel[2]],
        # )

        pybullet.resetDebugVisualizerCamera(
            cameraDistance=grid_size,  # 摄像机与玩家的距离
            cameraYaw=camera_yaw,  # 水平旋转角
            cameraPitch=-60,  # 俯仰角
            cameraTargetPosition=player_pos,
        )

        # sphere_position, _ = pybullet.getBasePositionAndOrientation(self.sphere_body)
        # self.ball.transform.setTranslation(QVector3D(sphere_position[0], sphere_position[2], sphere_position[1]))

        self.ball.update()

    def center_cursor(self):
        center_pos = self.mapToGlobal(QPoint(self.width() // 2, self.height() // 2))
        QCursor.setPos(center_pos)

    def create_scene(self):

        from coordinate import Coordinate

        self.coordinate = Coordinate(self.root_entity, grid_size)

        from ground import Ground

        self.ground = Ground(self.root_entity)
        self.ground.mesh.setWidth(grid_size * maze_size * 4)
        self.ground.mesh.setHeight(grid_size * maze_size * 4)

        self.create_maze()

    def create_maze(self):
        from wall import Wall
        from text import Text
        from texture import Texture

        from map_generator import Maze

        maze = Maze(maze_size)
        maze.init_maze()
        maze.prim()
        maze.after_prim()

        wall_texture = Texture(self.root_entity)
        wall_texture.generate_random_texture()
        wall_material = wall_texture.create_material()

        def create_wall(x, y, z, rotate=False):
            _wall = Wall(self.root_entity, grid_size)
            _wall.transform.setTranslation(QVector3D(x, y, z))
            if rotate:
                _wall.transform.setRotation(QQuaternion.fromEulerAngles(0, 90, 0))
            _wall.entity.addComponent(wall_material)
            self.wall_list.append(_wall)

        for v1 in maze.v_list:
            if v1.x == 0:
                create_wall(
                    0,
                    grid_size * 0.5,
                    grid_size * 0.55 + v1.y * grid_size * 1.1,
                    True,
                )

                # text = Text(self.root_entity, f"{v1.x}, {v1.y}")
                # text.transform.setTranslation(QVector3D(
                #     0 + grid_size * 0.2,
                #     grid_size,
                #     grid_size * 0.5 + v1.y * grid_size,
                # ))
                # text.transform.setRotation(QQuaternion.fromEulerAngles(-90, 0, 0))
                # self.text_list.append(text)

            if v1.y == 0:
                create_wall(
                    grid_size * 0.55 + v1.x * grid_size * 1.1,
                    grid_size * 0.5,
                    0,
                )

            if v1.x + 1 < maze.size:
                v2 = maze.v_list[(v1.x + 1) * maze.size + v1.y]
                if maze.e_prim_list[v1.get_id()][v2.get_id()] is None:
                    create_wall(
                        v2.x * grid_size * 1.1,
                        grid_size / 2,
                        grid_size * 0.55 + v2.y * grid_size * 1.1,
                        True,
                    )

                # text = Text(self.root_entity, f"{v2.x}, {v2.y}")
                # text.transform.setTranslation(QVector3D(
                #     v2.x * grid_size + grid_size * 0.2,
                #     grid_size,
                #     grid_size * 0.5 + v2.y * grid_size,
                # ))
                # text.transform.setRotation(QQuaternion.fromEulerAngles(-90, 0, 0))
                # self.text_list.append(text)
            else:
                create_wall(
                    (v1.x + 1) * grid_size * 1.1,
                    grid_size / 2,
                    grid_size * 0.55 + v1.y * grid_size * 1.1,
                    True,
                )

            if v1.y + 1 < maze.size:
                v2 = maze.v_list[v1.x * maze.size + (v1.y + 1)]
                if maze.e_prim_list[v1.get_id()][v2.get_id()] is None:
                    create_wall(
                        grid_size * 0.55 + v2.x * grid_size * 1.1,
                        grid_size / 2,
                        v2.y * grid_size * 1.1,
                    )
            else:
                create_wall(
                    grid_size * 0.55 + v1.x * grid_size * 1.1,
                    grid_size / 2,
                    (v1.y + 1) * grid_size * 1.1,
                )

        for wall in self.wall_list:
            wall_shape = pybullet.createCollisionShape(
                pybullet.GEOM_BOX,
                halfExtents=[
                    wall.mesh.xExtent() / 2,
                    wall.mesh.zExtent() / 2,
                    wall.mesh.yExtent() / 2,
                ],
            )
            pybullet.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=wall_shape,
                basePosition=[
                    wall.transform.translation().x(),
                    wall.transform.translation().z(),
                    wall.transform.translation().y(),
                ],
                baseOrientation=pybullet.getQuaternionFromEuler([
                    wall.transform.rotationX() / 180 * math.pi,
                    wall.transform.rotationZ() / 180 * math.pi,
                    wall.transform.rotationY() / 180 * math.pi,
                ]),
            )

    def keyPressEvent(self, event):
        # print(f"Key pressed: {event.modifiers()} {event.text()}")

        if self.controller.key_press_event(event):
            pass
        elif event.modifiers() == Qt.KeyboardModifier.NoModifier:
            if event.key() == Qt.Key.Key_F1:
                self.camera_list[self.camera_index].save(self.camera())
                # self.camera_list[self.camera_index].print_status(self.camera())
                self.camera_index = 1
                self.camera_list[self.camera_index].load(self.camera())

                self.controller.movement_speed = grid_size / 20
                self.unsetCursor()
            elif event.key() == Qt.Key.Key_F2:
                self.camera_list[self.camera_index].save(self.camera())
                # self.camera_list[self.camera_index].print_status(self.camera())
                self.camera_index = 2
                self.camera_list[self.camera_index].load(self.camera())

                self.controller.movement_speed = grid_size * 10
                self.setCursor(Qt.CursorShape.BlankCursor)
            elif event.key() == Qt.Key.Key_F3:
                self.camera_list[self.camera_index].save(self.camera())
                # self.camera_list[self.camera_index].print_status(self.camera())
                self.camera_index = 3
                self.camera_list[self.camera_index].load(self.camera())

                self.controller.movement_speed = grid_size / 20
                self.unsetCursor()
            elif event.key() == Qt.Key.Key_Escape:
                self.close()
            else:
                print(f"Key pressed: {event.text()}")
                event.ignore()
        else:
            print(f"Key pressed: {event.modifiers()} {event.text()}")
            event.ignore()

    def keyReleaseEvent(self, event):
        self.controller.key_release_event(event)

    def event(self, event):
        if event.type() == QEvent.Type.Enter:
            self.controller.last_cursor_pos = self.mapFromGlobal(QCursor.pos())
        elif event.type() == QEvent.Type.Leave:
            pass
        return super().event(event)

    def mousePressEvent(self, event):
        if self.camera_index == 2:
            player_pos, _ = pybullet.getBasePositionAndOrientation(self.player.body)
            view_vector = (self.controller.camera.viewCenter() - self.controller.camera.position()).normalized()
            self.ball.create_ball(
                QVector3D(player_pos[0], player_pos[2], player_pos[1]) + view_vector,
                view_vector * 5,
            )

    def mouseMoveEvent(self, event):
        delta_x = event.position().x() - self.controller.last_cursor_pos.x()
        delta_y = event.position().y() - self.controller.last_cursor_pos.y()

        if self.camera_index != 2:

            camera_position = self.camera().position()
            camera_view_center = self.camera().viewCenter()

            view_speed = 0.005

            forward = (camera_view_center - camera_position).normalized()
            right = QVector3D.crossProduct(forward, QVector3D(0, 1, 0)).normalized()
            up = QVector3D.crossProduct(right, forward).normalized()

            direction = (forward + right * delta_x * -view_speed + up * delta_y * view_speed).normalized()
            camera_view_center_new = camera_position + direction

            self.camera().setViewCenter(camera_view_center_new)
            self.camera().setUpVector(QVector3D(0, 1, 0))

        self.controller.last_cursor_pos = self.mapFromGlobal(QCursor.pos())


class BulletPhysics:
    def __init__(self):
        # self.physics_client = pybullet.connect(pybullet.DIRECT)
        self.physics_client = pybullet.connect(pybullet.GUI)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setTimeStep(1 / fps)

        self.plane_id = pybullet.loadURDF("plane.urdf")

    @staticmethod
    def step_simulation():
        pybullet.stepSimulation()


class CameraSave:
    position = None
    view_center = None
    up_vector = None

    def __init__(self):
        pass

    def save(self, camera):
        self.position = camera.position()
        self.view_center = camera.viewCenter()
        self.up_vector = camera.upVector()

    def load(self, camera):
        if self.position is not None:
            camera.setPosition(self.position)
        if self.view_center is not None:
            camera.setViewCenter(self.view_center)
        if self.up_vector is not None:
            camera.setUpVector(self.up_vector)

    @staticmethod
    def print_status(camera):
        print("position:", camera.position())
        print("up_vector:", camera.upVector())
        print("view_center:", camera.viewCenter())

        view_matrix = camera.viewMatrix()
        rotation_matrix = view_matrix.normalMatrix()
        rotation_quat = QQuaternion.fromRotationMatrix(rotation_matrix)
        euler_angles = rotation_quat.toEulerAngles()
        print(f"Pitch: {euler_angles.x():.2f} degrees")
        print(f"Yaw: {euler_angles.y():.2f} degrees")
        print(f"Roll: {euler_angles.z():.2f} degrees")


# class Ball:
#
#     entity = None
#     mesh = None
#     transform = None
#     material = None
#
#     def __init__(self, root_entity, grid_size):
#
#         self.entity = Qt3DCore.QEntity(root_entity)
#
#         self.mesh = Qt3DExtras.QSphereMesh()
#         self.mesh.setRadius(grid_size / 10)
#
#         self.transform = Qt3DCore.QTransform()
#         self.transform.setTranslation(QVector3D(10, 10, 10))
#
#         self.material = Qt3DExtras.QPhongMaterial(root_entity)
#         self.material.setAmbient(QColor(255, 255, 255))
#         self.material.setDiffuse(QColor(255, 255, 255))
#         self.material.setSpecular(QColor(0, 0, 0))
#         self.material.setShininess(0)
#
#         self.entity.addComponent(self.mesh)
#         self.entity.addComponent(self.transform)
#         self.entity.addComponent(self.material)


if __name__ == '__main__':
    app = QGuiApplication(sys.argv)
    view = Window()
    view.show()
    sys.exit(app.exec())
