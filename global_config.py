
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DRender import Qt3DRender

import random

random.seed(1001)

grid_size = 5
maze_size = 5

# fps = 1
# fps = 10
# fps = 30
fps = 60

root_entity = Qt3DCore.QEntity()
