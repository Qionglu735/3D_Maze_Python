
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtCore import QUrl
from PySide6.QtGui import QColor, QImage

import noise
import os
import random
import string

from global_config import root_entity


class Texture:
    color = ""

    texture_file = ""

    texture_image = None
    texture = None

    color_value = [
        # 2 ** 2.6 - 1,
        2 ** 3 - 1,
        2 ** 3.6 - 1,
        2 ** 4 - 1,
        2 ** 4.6 - 1,
        2 ** 5 - 1,
        2 ** 5.6 - 1,
        2 ** 6 - 1,
        2 ** 6.6 - 1,
        2 ** 7 - 1,
    ]
    color_index = {
        "test": [3, 2, 1],
        "red": [4, 0, 0],
        "yellow": [2, 2, 0],
        "green": [0, 4, 0],
        "cyan": [0, 2, 2],
        "blue": [0, 0, 4],
        "magenta": [2, 0, 2],
    }

    def __init__(self, color="red", size=512):
        # self.color = random.choice([x for x in self.color_index.keys()])
        # self.color = "magenta"
        self.color = color
        self.size = size

        if not os.path.isdir("temp"):
            os.mkdir("temp")

        self.texture_file = f"temp/texture_{"".join([random.choice(string.ascii_lowercase) for _ in range(8)])}.png"

    def generate_random_texture(self):
        seed_x = random.uniform(0, 1000)
        seed_y = random.uniform(0, 1000)
        scale = self.size / 4
        image = QImage(self.size, self.size, QImage.Format.Format_RGB32)
        for x in range(self.size):
            for y in range(self.size):
                value = noise.pnoise2(
                    seed_x + x / scale, seed_y + y / scale,
                    octaves=16, persistence=0.5, lacunarity=2,
                    repeatx=self.size, repeaty=self.size)
                """
                scale: number that determines at what distance to view the noisemap.
                octaves: the number of levels of detail you want you perlin noise to have.
                lacunarity: number that determines how much detail is added or removed at each octave (adjusts frequency).
                persistence: number that determines how much each octave contributes to the overall shape (adjusts amplitude).
                https://medium.com/@yvanscher/playing-with-perlin-noise-generating-realistic-archipelagos-b59f004d8401
                """

                value_normalized = max(-0.5, min(value, 0.5 - 1e-9)) + 0.5
                color_index_indent = int(value_normalized * (len(self.color_value) - max(self.color_index[self.color])))
                image.setPixelColor(x, y, QColor(
                    self.color_value[self.color_index[self.color][0] + color_index_indent],
                    self.color_value[self.color_index[self.color][1] + color_index_indent],
                    self.color_value[self.color_index[self.color][2] + color_index_indent],
                ))

        image.save(self.texture_file)

        return image

    def create_material(self):
        texture_image = Qt3DRender.QTextureImage()
        texture_image.setSource(QUrl.fromLocalFile(self.texture_file))

        texture = Qt3DRender.QTexture3D()
        texture.addTextureImage(texture_image)

        texture_loader = Qt3DRender.QTextureLoader(root_entity)
        texture_loader.setSource(QUrl.fromLocalFile(self.texture_file))

        texture_material = Qt3DExtras.QTextureMaterial()
        texture_material.setTexture(texture_loader)

        return texture_material

    def __del__(self):
        os.remove(self.texture_file)
