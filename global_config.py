
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DRender import Qt3DRender

import random
import sys

# seed = 1001
# seed = 7097220057833736452
# seed = 2588775276581901677  # maze_size = 5; ValueError: list.remove(x): x not in list
seed = random.randint(0, sys.maxsize)

print("SEED:", seed)
random.seed(seed)

grid_size = 5
maze_size = 15

wall_height = grid_size * 1.0

# fps = 1
# fps = 10
# fps = 30
fps = 60

root_entity = Qt3DCore.QEntity()
