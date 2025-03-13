
from PySide6.QtCore import QObject, Qt, QTimer
from PySide6.QtGui import QVector3D, QCursor, QVector2D

import math


class CameraController:
    movement_speed = 5.0
    view_speed = 0.01

    last_cursor_pos = None

    def __init__(self, camera):
        self.camera = camera
        self.keys_pressed = set()

    def key_press_event(self, event):
        self.keys_pressed.add(event.key())
        if event.key() in [
            Qt.Key.Key_W,
            Qt.Key.Key_S,
            Qt.Key.Key_A,
            Qt.Key.Key_D,
            Qt.Key.Key_C,
            Qt.Key.Key_E,
        ]:
            return True
        else:
            return False

    def key_release_event(self, event):
        self.keys_pressed.discard(event.key())
        return True

    def cal_movement_vector(self):
        forward = self.camera.viewVector().normalized()
        right = QVector3D.crossProduct(forward, QVector3D(0, 1, 0)).normalized()
        up = QVector3D.crossProduct(right, forward).normalized()

        if Qt.Key.Key_W in self.keys_pressed:
            forward *= self.movement_speed
        elif Qt.Key.Key_S in self.keys_pressed:
            forward *= - self.movement_speed
        else:
            forward *= 0
        if Qt.Key.Key_A in self.keys_pressed:
            right *= - self.movement_speed
        elif Qt.Key.Key_D in self.keys_pressed:
            right *= self.movement_speed
        else:
            right *= 0
        up *= 0
        return forward + right + up

    def cal_view_vector(self, cursor_pos):
        view_speed = 0.01

        return QVector2D(
            (self.last_cursor_pos.x() - cursor_pos.x()) * -view_speed,
            (self.last_cursor_pos.y() - cursor_pos.y()) * view_speed,
        )

    def update_camera_position(self):
        position = self.camera.position()
        view_center = self.camera.viewCenter()
        forward = self.camera.viewVector().normalized()
        right = QVector3D.crossProduct(forward, QVector3D(0, 1, 0)).normalized()
        up = QVector3D.crossProduct(right, forward).normalized()

        if Qt.Key.Key_W in self.keys_pressed:
            # self.camera.translate(self.camera.viewVector() * self.movement_speed)
            position += forward * self.movement_speed
            view_center += forward * self.movement_speed
            self.camera.setPosition(position)
            self.camera.setViewCenter(view_center)
        if Qt.Key.Key_S in self.keys_pressed:
            # self.camera.translate(-self.camera.viewVector() * self.movement_speed)
            position -= forward * self.movement_speed
            view_center -= forward * self.movement_speed
            self.camera.setPosition(position)
            self.camera.setViewCenter(view_center)
        if Qt.Key.Key_A in self.keys_pressed:
            # self.camera.translate(-self.camera.rightVector() * self.movement_speed)
            position -= right * self.movement_speed
            view_center -= right * self.movement_speed
            self.camera.setPosition(position)
            self.camera.setViewCenter(view_center)
        if Qt.Key.Key_D in self.keys_pressed:
            # self.camera.translate(self.camera.rightVector() * self.movement_speed)
            position += right * self.movement_speed
            view_center += right * self.movement_speed
            self.camera.setPosition(position)
            self.camera.setViewCenter(view_center)
        if Qt.Key.Key_E in self.keys_pressed:
            position += up * self.movement_speed
            view_center += up * self.movement_speed
            self.camera.setPosition(position)
            self.camera.setViewCenter(view_center)
        if Qt.Key.Key_C in self.keys_pressed:
            position -= up * self.movement_speed
            view_center -= up * self.movement_speed
            self.camera.setPosition(position)
            self.camera.setViewCenter(view_center)

    def update_camera_rotation(self):
        pass
