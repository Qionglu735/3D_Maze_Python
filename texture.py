
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

    texture_file = "texture.png"

    texture_image = None
    texture = None

    def __init__(self, color="red"):
        self.color = color

        if not os.path.isdir("temp"):
            os.mkdir("temp")

        self.texture_file = f"temp/texture_{"".join([random.choice(string.ascii_lowercase) for _ in range(8)])}.png"

    def generate_random_texture(self, size=1024):
        scale = 512
        image = QImage(size, size, QImage.Format.Format_RGB32)
        for x in range(size):
            for y in range(size):
                value = noise.pnoise2(
                    x / scale, y / scale,
                    octaves=6, persistence=0.5, lacunarity=2,
                    repeatx=size, repeaty=size)
                """
                scale: number that determines at what distance to view the noisemap.
                octaves: the number of levels of detail you want you perlin noise to have.
                lacunarity: number that determines how much detail is added or removed at each octave (adjusts frequency).
                persistence: number that determines how much each octave contributes to the overall shape (adjusts amplitude).
                https://medium.com/@yvanscher/playing-with-perlin-noise-generating-realistic-archipelagos-b59f004d8401
                """
                color_value = [
                    2 ** 3 - 1,
                    2 ** 4 - 1,
                    2 ** 5 - 1,
                    2 ** 6 - 1,
                    2 ** 7 - 1,
                ]
                if value < -0.3:
                    if self.color == "red":
                        image.setPixelColor(x, y, QColor(color_value[1], color_value[0], color_value[0]))
                    elif self.color == "green":
                        image.setPixelColor(x, y, QColor(color_value[0], color_value[1], color_value[0]))
                    elif self.color == "blue":
                        image.setPixelColor(x, y, QColor(color_value[0], color_value[0], color_value[1]))
                elif value < 0:
                    if self.color == "red":
                        image.setPixelColor(x, y, QColor(color_value[2], color_value[1], color_value[1]))
                    elif self.color == "green":
                        image.setPixelColor(x, y, QColor(color_value[1], color_value[2], color_value[1]))
                    elif self.color == "blue":
                        image.setPixelColor(x, y, QColor(color_value[1], color_value[1], color_value[2]))
                elif value < 0.3:
                    if self.color == "red":
                        image.setPixelColor(x, y, QColor(color_value[3], color_value[2], color_value[2]))
                    elif self.color == "green":
                        image.setPixelColor(x, y, QColor(color_value[2], color_value[3], color_value[2]))
                    elif self.color == "blue":
                        image.setPixelColor(x, y, QColor(color_value[2], color_value[2], color_value[3]))
                else:
                    if self.color == "red":
                        image.setPixelColor(x, y, QColor(color_value[4], color_value[3], color_value[3]))
                    elif self.color == "green":
                        image.setPixelColor(x, y, QColor(color_value[3], color_value[4], color_value[3]))
                    elif self.color == "blue":
                        image.setPixelColor(x, y, QColor(color_value[3], color_value[3], color_value[4]))

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
