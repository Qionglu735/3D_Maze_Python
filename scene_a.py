
from PySide6.QtCore import Qt
from PySide6.QtGui import QQuaternion, QVector3D

import pybullet

from aim_line import AimLine
from ball import BallList
from coordinate import Coordinate
from global_config import grid_size, maze_size, wall_height, root_entity
from ground import Ground
from guide_line import GuideLine
from map_generator import Maze
from target import Target
from text import Text
from wall import WallList


class SceneA:
    coordinate = None
    ground = None
    maze = None
    wall_list = None
    text_list = None
    aim_line = None
    guide_line_list = None

    viewport = None
    render_capture_reply = None
    render_stage = 1

    def __init__(self, viewport):
        self.viewport = viewport

        self.coordinate = Coordinate(root_entity)
        self.ground = Ground()

        self.create_maze()

        self.aim_line = AimLine(root_entity)
        self.ball_list = BallList(root_entity)
        self.target = Target(
            root_entity,
            QVector3D(grid_size * (maze_size - 0.5), grid_size * 0.5, grid_size * (maze_size - 0.5)),
            QVector3D(0, 0, 90),
        )
        self.guide_line_list = list()

    def create_maze(self):
        self.maze = Maze(maze_size)
        self.maze.init_maze()
        self.maze.prim()
        self.maze.after_prim()
        # self.maze.solve_maze()

        def create_wall(x, y, z, rotate=False):
            if rotate:
                self.wall_list.create_wall(QVector3D(x, y, z), QQuaternion.fromEulerAngles(0, 90, 0))
            else:
                self.wall_list.create_wall(QVector3D(x, y, z), QQuaternion.fromEulerAngles(0, 0, 0))

        self.wall_list = WallList(root_entity)
        self.text_list = list()
        for v1 in self.maze.v_list:
            if v1.x == 0:
                create_wall(
                    0,
                    wall_height * 0.5,
                    grid_size * 0.5 + v1.y * grid_size,
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
                    grid_size * 0.5 + v1.x * grid_size,
                    wall_height * 0.5,
                    0,
                )

            if v1.x + 1 < self.maze.size:
                v2 = self.maze.v_list[(v1.x + 1) * self.maze.size + v1.y]
                if self.maze.e_prim_list[v1.get_id()][v2.get_id()] is None:
                    create_wall(
                        v2.x * grid_size,
                        wall_height * 0.5,
                        grid_size * 0.5 + v2.y * grid_size,
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
                    (v1.x + 1) * grid_size,
                    wall_height * 0.5,
                    grid_size * 0.5 + v1.y * grid_size,
                    True,
                )

            if v1.y + 1 < self.maze.size:
                v2 = self.maze.v_list[v1.x * self.maze.size + (v1.y + 1)]
                if self.maze.e_prim_list[v1.get_id()][v2.get_id()] is None:
                    create_wall(
                        grid_size * 0.5 + v2.x * grid_size,
                        wall_height * 0.5,
                        v2.y * grid_size,
                    )
            else:
                create_wall(
                    grid_size * 0.5 + v1.x * grid_size,
                    wall_height * 0.5,
                    (v1.y + 1) * grid_size,
                )

        self.wait_for_render()

    def wait_for_render(self):
        self.render_capture_reply = self.viewport.render_capture.requestCapture(self.render_stage)
        self.render_capture_reply.completed.connect(self.on_render_complete)
        print("wait_for_render:", self.render_stage)

    def on_render_complete(self):
        print("on_render_complete:", self.render_stage)
        if self.render_stage == 1:
            self.maze.solve_maze("dijkstra")
            self.guide_line_list.append(GuideLine(self.maze.dijkstra_solution.path, 0))
            # self.guide_line_list.append(GuideLine(self.maze.dfs_solution.path, 1))
            # self.guide_line_list.append(GuideLine(self.maze.dfs_reverse_solution.path, 2))
            # self.guide_line_list.append(GuideLine(self.maze.dfs_shuffle_solution.path, 3))
            # self.guide_line_list.append(GuideLine(self.maze.bfs_solution.path, 4))
            # self.guide_line_list.append(GuideLine(self.maze.bfs_reverse_solution.path, 5))
            # self.guide_line_list.append(GuideLine(self.maze.bfs_shuffle_solution.path, 6))

            self.render_stage += 1

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
            ),
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
            if contact[1] in self.ball_list.body_list:
                ball = self.ball_list.find_by_body(contact[1])
                if ball.body_type == "sticky":
                    ball.fix_position = True
            elif contact[2] in self.ball_list.body_list:
                ball = self.ball_list.find_by_body(contact[2])
                if ball.body_type == "sticky":
                    ball.fix_position = True


            if contact[1] in self.ball_list.body_list and contact[2] == self.target.body:
                print(contact[1], contact[2], contact[9])

            elif contact[2] in self.ball_list.body_list and contact[1] == self.target.body:
                print(contact[1], contact[2], contact[9])

    def on_mouse_press(self, event, player_pos=None, view_vector=None):
        if event.button() == Qt.MouseButton.LeftButton:
            self.ball_list.create_ball(
                QVector3D(
                    player_pos[0],
                    player_pos[2] - grid_size * 0.1,
                    player_pos[1],
                ),
                view_vector * grid_size ** 1.8,
            )
        elif event.button() == Qt.MouseButton.RightButton:
            self.aim_line.set_show()

    def on_mouse_release(self, event):
        if event.button() == Qt.MouseButton.RightButton:
            self.aim_line.set_hide()

    def before_exit(self):
        Ground.before_exit()
        self.wall_list.before_exit()
