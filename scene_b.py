
from PySide6.QtCore import Qt
from PySide6.QtGui import QQuaternion, QVector3D

import pybullet
import itertools

from aim_line import AimLine
from ball import BallList
from coordinate import Coordinate
from global_config import grid_size, maze_size, wall_height, root_entity
from ground import Ground
from target import Target
from wall import Wall


class SceneB:
    coordinate = None
    ground = None
    maze = None
    wall_list = None
    text_list = None
    aim_line = None
    guide_line_list = None

    ingredient_list = None

    viewport = None
    render_capture_reply = None
    render_stage = 0

    def __init__(self, viewport):
        self.viewport = viewport

        self.coordinate = Coordinate(root_entity)
        self.ground = Ground()

        self.aim_line = AimLine(root_entity)
        self.ball_list = BallList(root_entity)
        self.target = Target(
            root_entity,
            QVector3D(grid_size * (maze_size - 0.5), grid_size * 0.5, grid_size * (maze_size - 0.5)),
            QVector3D(0, 0, 90),
        )
        self.guide_line_list = list()

        self.wait_for_render()

    def wait_for_render(self):
        self.render_stage += 1
        self.render_capture_reply = self.viewport.render_capture.requestCapture(self.render_stage)
        self.render_capture_reply.completed.connect(self.on_render_complete)
        print("wait_for_render:", self.render_stage)

    def on_render_complete(self):
        print("on_render_complete:", self.render_stage)
        if self.render_stage == 1:
            self.create_maze()
            self.wait_for_render()
        elif self.render_stage == 2:
            self.alchemy_desk()
            self.wait_for_render()

    def create_maze(self):

        def create_wall(x, y, z, rotate=False):
            if rotate:
                _wall = Wall(root_entity, QVector3D(x, y, z), QQuaternion.fromEulerAngles(0, 90, 0))
            else:
                _wall = Wall(root_entity, QVector3D(x, y, z), QQuaternion.fromEulerAngles(0, 0, 0))

            self.wall_list.append(_wall)

        self.wall_list = list()
        for x, y in itertools.product(range(0,  maze_size), range(0, maze_size)):
            if x == 0:
                create_wall(
                    0,
                    wall_height * 0.2,
                    grid_size * 0.5 + y * grid_size,
                    True,
                )
            if y == 0:
                create_wall(
                    grid_size * 0.5 + x * grid_size,
                    wall_height * 0.2,
                    0,
                )
            if x + 1 == maze_size:
                create_wall(
                    (x + 1) * grid_size,
                    wall_height * 0.2,
                    grid_size * 0.5 + y * grid_size,
                    True,
                )

            if y + 1 == maze_size:
                create_wall(
                    grid_size * 0.5 + x * grid_size,
                    wall_height * 0.2,
                    (y + 1) * grid_size,
                )

    def alchemy_desk(self):
        from ingredient import IngredientList
        self.ingredient_list = IngredientList()

        self.ingredient_list.create_ingredient(QVector3D(0, 0, -5), QVector3D(0, 0, 0))
        self.ingredient_list.create_ingredient(QVector3D(5, 5, -5), QVector3D(0, 0, 0))
        self.ingredient_list.create_ingredient(QVector3D(10, 10, -5), QVector3D(0, 0, 0))
        self.ingredient_list.create_ingredient(QVector3D(15, 15, -5), QVector3D(0, 0, 0))
        self.ingredient_list.create_ingredient(QVector3D(20, 20, -5), QVector3D(0, 0, 0))
        self.ingredient_list.create_ingredient(QVector3D(25, 25, -5), QVector3D(0, 0, 0))

    def cancel_aim_sim(self):
        self.aim_line.cancel_sim()
        if self.aim_line.showing:
            self.aim_line.set_hide()

    def enable_aim_sim(self, player_pos, view_vector):
        self.aim_line.enable_sim(
            QVector3D(
                player_pos[0],
                player_pos[2] - grid_size * 0.1,
                player_pos[1],
            ) + view_vector,
            view_vector * grid_size ** 1.8,
        )
        if self.aim_line.showing:
            self.aim_line.set_show()

    def update(self):
        self.coordinate.update()
        self.ball_list.update()
        self.aim_line.update()
        self.target.update()
        for guide_line in self.guide_line_list:
            guide_line.update()

        contact_points = pybullet.getContactPoints()
        collision_events = []
        for contact in contact_points:
            if contact[1] in self.ball_list.body_list and contact[2] == self.target.body \
                    or contact[2] in self.ball_list.body_list and contact[1] == self.target.body:
                body_a = contact[1]
                body_b = contact[2]
                print(body_a, body_b, contact[9])
                collision_events.append((body_a, body_b, contact[9]))

    def on_mouse_press(self, event, player_pos=None, view_vector=None):
        if event.button() == Qt.MouseButton.LeftButton:
            self.ball_list.create_ball(
                QVector3D(
                    player_pos[0],
                    player_pos[2] - grid_size * 0.1,
                    player_pos[1],
                ) + view_vector,
                view_vector * grid_size ** 1.8,
            )
        elif event.button() == Qt.MouseButton.RightButton:
            self.aim_line.set_show()

    def on_mouse_release(self, event):
        if event.button() == Qt.MouseButton.RightButton:
            self.aim_line.set_hide()

    @staticmethod
    def before_exit():
        Ground.before_exit()
        Wall.before_exit()
