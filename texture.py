
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtCore import QUrl, QBuffer, QIODevice
from PySide6.QtGui import QVector3D, QColor, QImage, QImageReader

import noise


class Texture:
    texture_file = "random_texture.png"

    root_entity = None

    texture_image = None
    texture = None

    def __init__(self, root_entity):
        self.root_entity = root_entity

    def generate_random_texture(self, size=1024):
        scale = 128
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
                if value < -0.5:
                    image.setPixelColor(x, y, QColor(31, 0, 0))
                elif value < 0:
                    image.setPixelColor(x, y, QColor(63, 31, 31))
                elif value < 0.5:
                    image.setPixelColor(x, y, QColor(127, 63, 63))
                else:
                    image.setPixelColor(x, y, QColor(191, 127, 127))

        image.save(self.texture_file)

        return image

    def create_material(self):
        texture_image = Qt3DRender.QTextureImage()
        texture_image.setSource(QUrl.fromLocalFile(self.texture_file))

        texture = Qt3DRender.QTexture3D()
        texture.addTextureImage(texture_image)

        texture_loader = Qt3DRender.QTextureLoader(self.root_entity)
        texture_loader.setSource(QUrl.fromLocalFile(self.texture_file))

        texture_material = Qt3DExtras.QTextureMaterial(self.root_entity)
        texture_material.setTexture(texture_loader)

        return texture_material
