
import itertools
import random

# random.seed(1001)

random_range = 100

node_access_rate = 1  # close some node before prim. 0 for all, 1 for none
edge_access_rate = 1  # open extra path after prim. 1 for none, 2 for all


class Room:

    top = 0
    bottom = 0
    left = 0
    right = 0

    v_list = list()

    def __init__(self, entrance_x, entrance_y, size, room_size):
        entrance_direction = ["top", "bottom", "left", "right"]
        if entrance_x < 2:
            entrance_direction.remove("top")
        if entrance_x > size - 2:
            entrance_direction.remove("bottom")
        if entrance_y < 2:
            entrance_direction.remove("left")
        if entrance_y > size - 2:
            entrance_direction.remove("right")

        final_entrance_direction = entrance_direction[random.randint(0, len(entrance_direction) - 1)]
        # print("entrance:", entrance_x, entrance_y, final_entrance_direction)
        
        room_size -= 1
        if final_entrance_direction == "top":
            self.top = 0
            self.bottom = room_size
        elif final_entrance_direction == "bottom":
            self.top = room_size
            self.bottom = 0
        elif final_entrance_direction == "left":
            self.left = 0
            self.right = room_size
        elif final_entrance_direction == "right":
            self.left = room_size
            self.right = 0

        if final_entrance_direction in ["left", "right"]:
            self.top = random.randint(0, min(room_size, entrance_y))
            self.bottom = room_size - self.top
        elif final_entrance_direction in ["top", "bottom"]:
            self.left = random.randint(0, min(room_size, entrance_x))
            self.right = room_size - self.left

        # print(f"    {self.top:4}\n{self.left:4}    {self.right:4}\n    {self.bottom:4}")


class V:

    maze_size = 0

    x = 0
    y = 0
    w = 0

    room = None

    def __init__(self, x, y, maze_size):
        self.maze_size = maze_size
        self.x = x
        self.y = y
        self.w = random.randint(0, random_range - 1)

        if 0 < x < self.maze_size - 1 and 0 < y < self.maze_size - 1 and self.w > random_range * 0.97:
            self.room = Room(x, y, self.maze_size, 3)

    def get_id(self):
        return self.x * self.maze_size + self.y



class E:
    v1 = None
    v2 = None
    w = 0

    def __init__(self, v1, v2):
        self.v1 = v1
        self.v2 = v2
        self.w = random.randint(0, random_range - 1)


class Maze:

    size = 5

    v_list = list()
    e_list = list()
    e_prim_list = list()

    def __init__(self, size):
        self.size = size

    def init_maze(self):
        size = self.size
        
        for i in range(size ** 2):
            self.e_list.append(list())
            self.e_prim_list.append(list())
            for j in range(size ** 2):
                if i < size and j < size:
                    self.v_list.append(V(i, j, size))
                self.e_list[i].append(None)
                self.e_prim_list[i].append(None)

        for i in range(size):
            for j in range(size):
                if self.v_list[i * size + j].w > random_range * node_access_rate:
                    self.v_list[i * size + j].w = -1
                    if j > 0:
                        self.e_list[i * size + j][i * size + (j - 1)] = None
                        self.e_list[i * size + (j - 1)][i * size + j] = None
                    if i > 0:
                        self.e_list[i * size + j][(i - 1) * size + j] = None
                        self.e_list[(i - 1) * size + j][i * size + j] = None
                    continue
                    
                if j < size - 1:
                    self.e_list[i * size + j][i * size + (j + 1)] = E(
                        self.v_list[i * size + j],
                        self.v_list[i * size + (j + 1)],
                    )
                    self.e_list[i * size + (j + 1)][i * size + j] = E(
                        self.v_list[i * size + (j + 1)],
                        self.v_list[i * size + j],
                    )
                    
                if i < size - 1:
                    self.e_list[i * size + j][(i + 1) * size + j] = E(
                        self.v_list[i * size + j],
                        self.v_list[(i + 1) * size + j],
                    )
                    self.e_list[(i + 1) * size + j][i * size + j] = E(
                        self.v_list[(i + 1) * size + j],
                        self.v_list[i * size + j],
                    )

        # remove room space before prim
        for i, j in itertools.product(range(self.size), range(self.size)):
            if self.v_list[i * size + j].room is not None:
                room = self.v_list[i * size + j].room
                for _i, _j in itertools.product(
                    range(i - room.top, i + room.bottom + 1),
                    range(j - room.left, j + room.right + 1),
                ):
                    room.v_list.append(_i * size + _j)
                    if i == _i and j == _j:
                        # room entrance
                        continue
                    if _i < 0 or _i >= size or _j < 0 or _j >= size:
                        # out of bound
                        continue
                    self.v_list[_i * size + _j].w = -1
                    if _j > 0:
                        self.e_list[_i * size + _j][_i * size + (_j - 1)] = None
                        self.e_list[_i * size + (_j - 1)][_i * size + _j] = None
                    if _i > 0:
                        self.e_list[_i * size + _j][(_i - 1) * size + _j] = None
                        self.e_list[(_i - 1) * size + _j][_i * size + _j] = None
                    if _j < size - 1:
                        self.e_list[_i * size + _j][_i * size + (_j + 1)] = None
                        self.e_list[_i * size + (_j + 1)][_i * size + _j] = None
                    if _i < size - 1:
                        self.e_list[_i * size + _j][(_i + 1) * size + _j] = None
                        self.e_list[(_i + 1) * size + _j][_i * size + _j] = None

    def prim(self):
        s_list = list()
        v_list = list()
        for i in range(self.size ** 2):
            if self.v_list[i].w != -1:
                v_list.append(self.v_list[i])
        s_list.append(self.v_list[0])
        v_list.remove(self.v_list[0])
        count = 1

        while True:
            edge_list = list()
            for s in s_list:
                for e in self.e_list[s.get_id()]:
                    if e is not None and e.v2 not in s_list:
                        edge_list.append(e)

            min_w = -1
            min_e = None
            for e in edge_list:
                if min_w == -1 or e.w < min_w:
                    min_w = e.w
                    min_e = e
            if min_e is not None:
                s_list.append(min_e.v2)
                v_list.remove(min_e.v2)
                self.e_prim_list[min_e.v1.x * self.size + min_e.v1.y][min_e.v2.x * self.size + min_e.v2.y] = min_e
                self.e_prim_list[min_e.v2.x * self.size + min_e.v2.y][min_e.v1.x * self.size + min_e.v1.y] = min_e
                count += 1
            else:
                break

    def after_prim(self):
        size = self.size
        # add room space after prim
        for i, j in itertools.product(range(self.size), range(self.size)):
            if self.v_list[i * self.size + j].room is not None:
                room = self.v_list[i * self.size + j].room
                for _i, _j in itertools.product(
                    range(i - room.top, i + room.bottom + 1),
                    range(j - room.left, j + room.right + 1),
                ):
                    if _i < 0 or _i >= self.size or _j < 0 or _j >= self.size:
                        # out of bound
                        continue
                    self.v_list[_i * self.size + _j].w = 1
                    if _j + 1 < self.size:
                        self.e_list[_i * size + _j][_i * size + (_j + 1)] = E(
                            self.v_list[_i * size + _j],
                            self.v_list[_i * size + (_j + 1)],
                        )
                        self.e_list[_i * size + (_j + 1)][_i * size + _j] = E(
                            self.v_list[_i * size + (_j + 1)],
                            self.v_list[_i * size + _j],
                        )
                        if _i * size + (_j + 1) in room.v_list:
                            self.e_prim_list[_i * size + _j][_i * size + (_j + 1)] = self.e_list[_i * size + _j][_i * size + (_j + 1)]
                            self.e_prim_list[_i * size + (_j + 1)][_i * size + _j] = self.e_list[_i * size + (_j + 1)][_i * size + _j]

                    if _i + 1 < size:
                        self.e_list[_i * size + _j][(_i + 1) * size + _j] = E(
                            self.v_list[_i * size + _j],
                            self.v_list[(_i + 1) * size + _j],
                        )
                        self.e_list[(_i + 1) * size + _j][_i * size + _j] = E(
                            self.v_list[(_i + 1) * size + _j],
                            self.v_list[_i * size + _j],
                        )
                        if (_i + 1) * size + _j in room.v_list:
                            self.e_prim_list[_i * size + _j][(_i + 1) * size + _j] = self.e_list[_i * size + _j][(_i + 1) * size + _j]
                            self.e_prim_list[(_i + 1) * size + _j][_i * size + _j] = self.e_list[(_i + 1) * size + _j][_i * size + _j]

        # add extra edge
        for i, j in itertools.product(range(self.size), range(self.size)):
            if self.e_list[i][j] is not None and self.e_prim_list[i][j] is None:
                if self.e_list[i][j].w < random_range * (edge_access_rate - 1):
                    self.e_prim_list[i][j] = self.e_list[i][j]
                    self.e_prim_list[j][i] = self.e_list[j][i]





