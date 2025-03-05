
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QColor, QFont


class Text:

    font_size = 1

    def __init__(self, root_entity, text):
        self.entity = Qt3DExtras.QText2DEntity(root_entity)
        self.entity.setText(text)
        self.entity.setFont(QFont("Courier New", self.font_size))
        self.entity.setWidth(self.font_size * 0.8 * len(text))
        self.entity.setHeight(self.font_size * 1.5)
        self.entity.setColor(QColor(255, 255, 255))

        self.transform = Qt3DCore.QTransform()
        self.transform.setTranslation(QVector3D(self.entity.width() / -2, self.entity.height() / -2, 0))

        self.entity.addComponent(self.transform)