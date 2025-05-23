
import itertools
import random

random_range = 100

node_access_rate = 0.93  # close some node before prim. 0 for all, 1 for none
edge_access_rate = 1.33  # open extra path after prim. 1 for none, 2 for all


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

    dijkstra_solution = None
    dfs_solution = None
    dfs_reverse_solution = None
    dfs_shuffle_solution = None
    bfs_solution = None
    bfs_reverse_solution = None
    bfs_shuffle_solution = None

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

        for i, j in itertools.product(range(self.size), range(self.size)):
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
        for i, j in itertools.product(range(self.size ** 2), range(self.size ** 2)):
            if self.e_list[i][j] is not None and self.e_prim_list[i][j] is None:
                if self.e_list[i][j].w < random_range * (edge_access_rate - 1):
                    self.e_prim_list[i][j] = self.e_list[i][j]
                    self.e_prim_list[j][i] = self.e_list[j][i]

    def solve_maze(self, method=""):
        if method == "" or method == "dijkstra":
            self.dijkstra_solution = DijkstraSolution(self)
            self.dijkstra_solution.solve(0, self.size ** 2 - 1)
            print("dijkstra:", len(self.dijkstra_solution.path))

        if method == "" or method == "dfs":
            self.dfs_solution = DfsSolution(self)
            self.dfs_solution.solve(0, self.size ** 2 - 1)
            print("dfs:", len(self.dfs_solution.path))

        if method == "" or method == "dfs_reverse":
            self.dfs_reverse_solution = DfsSolution(self, reverse=True)
            self.dfs_reverse_solution.solve(0, self.size ** 2 - 1)
            print("dfs reverse:", len(self.dfs_reverse_solution.path))

        if method == "" or method == "dfs_shuffle":
            self.dfs_shuffle_solution = DfsSolution(self, shuffle=True)
            self.dfs_shuffle_solution.solve(0, self.size ** 2 - 1)
            print("dfs shuffle:", len(self.dfs_shuffle_solution.path))

        if method == "" or method == "bfs":
            self.bfs_solution = BfsSolution(self)
            self.bfs_solution.solve(0, self.size ** 2 - 1)
            print("bfs:", len(self.bfs_solution.path))

        if method == "" or method == "bfs_reverse":
            self.bfs_reverse_solution = BfsSolution(self, reverse=True)
            self.bfs_reverse_solution.solve(0, self.size ** 2 - 1)
            print("bfs reverse:", len(self.bfs_reverse_solution.path))

        if method == "" or method == "bfs_shuffle":
            self.bfs_shuffle_solution = BfsSolution(self, shuffle=True)
            self.bfs_shuffle_solution.solve(0, self.size ** 2 - 1)
            print("bfs shuffle:", len(self.bfs_shuffle_solution.path))


class DijkstraSolution:
    maze = None
    path = None
    cost = 0

    def __init__(self, maze):
        self.maze = maze

    def solve(self, from_id, to_id):
        u_list = list()
        v_list = [x for x in range(0, self.maze.size ** 2)]
        d_dict = dict()

        u_list.append(from_id)
        v_list.remove(from_id)
        d_dict[from_id] = {
            "id": from_id,
            "path": [from_id],
            "cost": 0
        }

        while to_id not in u_list:
            temp_u_list = list()
            for u_id in u_list:
                for v_id in v_list:
                    if self.maze.e_prim_list[u_id][v_id] is not None:
                        temp_u_list.append({
                            "id": v_id,
                            "path": d_dict[u_id]["path"] + [v_id],
                            "cost": d_dict[u_id]["cost"] + self.maze.e_prim_list[u_id][v_id].w,
                        })
            min_cost = -1
            min_u_info = None
            for u_info in temp_u_list:
                if min_cost == -1 or u_info["cost"] < min_cost:
                    min_cost = u_info["cost"]
                    min_u_info = u_info
            if min_u_info is not None:
                u_list.append(min_u_info["id"])
                v_list.remove(min_u_info["id"])
                d_dict[min_u_info["id"]] = min_u_info
            else:
                break

        self.path = [self.maze.v_list[x] for x in d_dict[to_id]["path"]]
        self.cost = d_dict[to_id]["cost"]


class DfsSolution:
    maze = None
    path = None
    cost = 0

    to_id = 0
    reverse = False
    shuffle = False

    u_list = list()
    v_list = list()
    d_dict = dict()

    def __init__(self, maze, reverse=False, shuffle=False):
        self.maze = maze
        self.reverse = reverse
        self.shuffle = shuffle

    def solve(self, from_id, to_id):
        self.to_id = to_id

        self.u_list = list()
        self.v_list = [x for x in range(0, self.maze.size ** 2)]
        self.d_dict = dict()

        self.u_list.append(from_id)
        self.v_list.remove(from_id)
        self.d_dict[from_id] = {
            "id": from_id,
            "path": [from_id],
            "cost": 0
        }

        self.dfs(from_id)

        self.path = [self.maze.v_list[x] for x in self.d_dict[to_id]["path"]]
        self.cost = self.d_dict[to_id]["cost"]

    def dfs(self, u_id):
        if u_id == self.to_id:
            return True
        u_list = list()
        for v_id in self.v_list:
            if self.maze.e_prim_list[u_id][v_id] is not None:
                u_list.append(v_id)
                self.d_dict[v_id] = {
                    "id": v_id,
                    "path": self.d_dict[u_id]["path"] + [v_id],
                    "cost": self.d_dict[u_id]["cost"] + self.maze.e_prim_list[u_id][v_id].w,
                }

        if self.reverse:
            u_list.reverse()
        if self.shuffle:
            random.shuffle(u_list)

        for u_id in u_list:
            self.u_list.append(u_id)
            self.v_list.remove(u_id)

            if self.dfs(u_id) is True:
                return True
            else:
                self.u_list.remove(u_id)
                self.v_list.append(u_id)
        return False


class BfsSolution:
    maze = None
    path = None
    cost = 0

    to_id = 0
    reverse = False
    shuffle = False

    u_list = list()
    v_list = list()
    d_dict = dict()

    def __init__(self, maze, reverse=False, shuffle=False):
        self.maze = maze
        self.reverse = reverse
        self.shuffle = shuffle

    def solve(self, from_id, to_id):
        self.to_id = to_id

        self.u_list = list()
        self.v_list = [x for x in range(0, self.maze.size ** 2)]
        self.d_dict = dict()

        self.u_list.append(from_id)
        self.v_list.remove(from_id)
        self.d_dict[from_id] = {
            "id": from_id,
            "path": [from_id],
            "cost": 0
        }

        self.bfs()

        self.path = [self.maze.v_list[x] for x in self.d_dict[to_id]["path"]]
        self.cost = self.d_dict[to_id]["cost"]

    def bfs(self):
        if self.reverse:
            self.v_list.reverse()
        if self.shuffle:
            random.shuffle(self.v_list)

        i = 0
        while i < len(self.u_list):
            u_id = self.u_list[i]
            if u_id == self.to_id:
                break
            j = 0
            while j < len(self.v_list):
                v_id = self.v_list[j]
                if self.maze.e_prim_list[u_id][v_id] is not None:
                    self.d_dict[v_id] = {
                        "id": v_id,
                        "path": self.d_dict[u_id]["path"] + [v_id],
                        "cost": self.d_dict[u_id]["cost"] + self.maze.e_prim_list[u_id][v_id].w,
                    }
                    self.u_list.append(v_id)
                    self.v_list.remove(v_id)
                else:
                    j += 1
            i += 1
