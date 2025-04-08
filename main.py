
from PySide6.QtCore import Property, QObject, Signal, QPoint, Qt, QTimer, QEvent, QRectF
from PySide6.QtGui import QGuiApplication, QMatrix4x4, QQuaternion, QVector3D, QColor, QSurfaceFormat, QCursor
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender

import math
import pybullet
import pybullet_data
import sys

from collision_group import CollisionGroup
from global_config import grid_size, maze_size, fps, root_entity
from player import Player
from viewport_manager import Layer, Viewport


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


CollisionGroup.init()


class Window(Qt3DExtras.Qt3DWindow):

    camera_index = 0
    camera_list = [None for _ in range(8)]

    scene = None

    player = None

    yaw_angle = 0
    pitch_angle = 0

    frame_counter = 0

    surface_selector = None
    viewport_list = list()

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

        self.defaultFrameGraph().setClearColor(Qt.GlobalColor.black)

        surface_format = QSurfaceFormat()
        surface_format.setSamples(4)  # 设置抗锯齿
        self.setFormat(surface_format)

        # 物理模拟
        self.physics = BulletPhysics()

        # Camera
        self.camera_list = [CameraSave() for _ in range(8)]
        self.camera_list[0].save(self.camera())

        self.camera().lens().setPerspectiveProjection(45, 16 / 9, 0.001, 1000)

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

        from camera_control import CameraController

        self.controller = CameraController(self.camera())
        self.controller.last_cursor_pos = self.mapFromGlobal(QCursor.pos())

        self.camera_index = 1
        self.camera_list[self.camera_index].load(self.camera())

        # Root entity
        self.root_entity = root_entity
        self.setRootEntity(self.root_entity)

        self.surface_selector = Qt3DRender.QRenderSurfaceSelector()
        self.renderSettings().setActiveFrameGraph(self.surface_selector)

        self.viewport_list.append(Viewport(self.surface_selector, self.camera()))
        self.viewport_list[0].layer_filter.addLayer(Layer().get("scene"))

        self.viewport_list.append(Viewport(self.surface_selector))
        self.viewport_list[1].camera.lens().setPerspectiveProjection(45.0, 16.0 / 9.0, 0.1, 1000.0)
        self.viewport_list[1].camera.setPosition(QVector3D(
            grid_size * maze_size / 2,
            grid_size * maze_size * 2,
            grid_size * maze_size / 2,
        ))
        self.viewport_list[1].camera.setViewCenter(QVector3D(
            grid_size * maze_size / 2,
            0,
            grid_size * maze_size / 2 - 1,
        ))
        self.viewport_list[1].camera.setUpVector(QVector3D(0, 1, 0))
        self.viewport_list[1].viewport.setNormalizedRect(QRectF(0.3, 0.3, 0.7, 0.7))
        # self.viewport_list[1].layer_filter.addLayer(Layer().get("ui"))

        self.viewport_list.append(Viewport(self.surface_selector))
        self.viewport_list[2].camera.lens().setPerspectiveProjection(45.0, 16.0 / 9.0, 0.1, 1000.0)
        self.viewport_list[2].camera.setPosition(QVector3D(
            grid_size * maze_size / 2,
            grid_size * maze_size / 2,
            grid_size * maze_size * 1,
        ))
        self.viewport_list[2].camera.setViewCenter(QVector3D(
            grid_size * maze_size / 2,
            grid_size * maze_size / 2,
            0,
        ))
        self.viewport_list[2].camera.setUpVector(QVector3D(0, 1, 0))
        self.viewport_list[2].viewport.setNormalizedRect(QRectF(0, 0, 1, 1))

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

        self.directional_light_entity.addComponent(Layer().get("scene"))

        self.create_scene()

        self.render_capture = None
        self.render_capture_reply = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_physics)
        self.timer.start(math.floor(1000 / fps))

    def update_physics(self):
        try:
            self.physics.step_simulation()
        except pybullet.error:
            self.timer.stop()
            self.close()
        self.scene.update()

        if self.camera_index == 2:
            self.player.update(self.controller)

            self.controller.update_fp_camera(
                QVector3D(
                    self.player.latest_player_pos[0],
                    self.player.latest_player_pos[2],
                    self.player.latest_player_pos[1],
                ),
                self.mapFromGlobal(QCursor.pos()),
            )
            self.player.update_map(self.controller.latest_view_vector)

            if self.player.latest_pos_move_vector.length() + self.controller.latest_view_move_vector.length() > 0:
                self.scene.cancel_aim_sim()
            else:
                self.scene.enable_aim_sim(self.player.latest_player_pos, self.controller.latest_view_vector)

            self.center_cursor()
            self.controller.last_cursor_pos = self.mapFromGlobal(QCursor.pos())

            # for pybullet debug camera
            camera_view_vector = self.controller.latest_view_vector
            try:
                camera_yaw = math.degrees(math.asin(camera_view_vector.x()))
                if camera_view_vector.z() < 0:
                    camera_yaw *= -1
            except ValueError:
                print("camera_view_vector.x:", camera_view_vector.x())
                camera_yaw = 0

            pybullet.resetDebugVisualizerCamera(
                cameraDistance=grid_size,  # 摄像机与玩家的距离
                cameraYaw=camera_yaw,  # 水平旋转角
                cameraPitch=-60,  # 俯仰角
                cameraTargetPosition=self.player.latest_player_pos,
            )

        else:
            self.controller.update_camera_position()

        self.frame_counter = (self.frame_counter + 1) % fps

    def center_cursor(self):
        center_pos = self.mapToGlobal(QPoint(self.width() // 2, self.height() // 2))
        QCursor.setPos(center_pos)

    def create_scene(self):

        from scene_a import SceneA
        from scene_b import SceneB

        self.scene = SceneA(self.viewport_list[0])
        self.viewport_list[1].layer_filter.addLayer(Layer().get("ui"))

        # self.scene = SceneB(self.viewport_list[0])
        # self.viewport_list[2].layer_filter.addLayer(Layer().get("alchemy"))

        self.player = Player()

        player_pos, player_ori = pybullet.getBasePositionAndOrientation(self.player.body)

        camera_2 = self.camera_list[2]
        camera_2.position = QVector3D(player_pos[0], player_pos[2], player_pos[1])
        camera_2.view_center = QVector3D(player_pos[0] + 1, player_pos[2], player_pos[1])
        camera_2.up_vector = QVector3D(0, 1, 0)

        self.player.transform_map.setTranslation(QVector3D(player_pos[0], player_pos[2], player_pos[1]))
        self.player.transform_map.setRotationZ(-90)

    def to_exit(self):
        self.scene.before_exit()
        self.close()

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
                self.to_exit()
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
            if event.button() == Qt.MouseButton.LeftButton:
                player_pos, _ = pybullet.getBasePositionAndOrientation(self.player.body)
                view_vector = (self.controller.camera.viewCenter() - self.controller.camera.position()).normalized()
                self.scene.on_mouse_press(event, player_pos=player_pos, view_vector=view_vector)
            elif event.button() == Qt.MouseButton.RightButton:
                self.scene.on_mouse_press(event)

    def mouseReleaseEvent(self, event):
        if self.camera_index == 2:
            if event.button() == Qt.MouseButton.RightButton:
                self.scene.on_mouse_release(event)

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
        self.physics_client = pybullet.connect(pybullet.DIRECT)
        # self.physics_client = pybullet.connect(pybullet.GUI)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setTimeStep(1 / fps)

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


if __name__ == '__main__':
    app = QGuiApplication(sys.argv)
    view = Window()
    view.show()
    sys.exit(app.exec())
