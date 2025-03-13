import struct

from PySide6.QtCore import QByteArray
from PySide6.QtGui import QVector3D
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DRender import Qt3DRender


class LineStrip(Qt3DCore.QEntity):
    def __init__(self, parent=None):
        super().__init__(parent)

        # 创建几何体
        self.geometry = Qt3DCore.QGeometry(self)

        # 顶点数据（例如：折线 (0,0,0) -> (1,1,1) -> (2,0,2)）
        vertex_data = [
            QVector3D(0.0, 0.0, 0.0),
            QVector3D(1.0, 1.0, 1.0),
            QVector3D(2.0, 0.0, 2.0),
            QVector3D(3.0, -1.0, 3.0)
        ]

        # 将顶点数据转换为 QByteArray
        raw_vertex_data = QByteArray()
        for v in vertex_data:
            raw_vertex_data.append(struct.pack("<fff", v.x(), v.y(), v.z()))

        # 顶点缓冲区
        vertex_buffer = Qt3DCore.QBuffer(
            self.geometry,
            usage=Qt3DCore.QBuffer.UsageType.StaticDraw,
            accessType=Qt3DCore.QBuffer.AccessType.Write,
        )
        vertex_buffer.setData(raw_vertex_data)

        # 顶点属性
        position_attr = Qt3DCore.QAttribute()
        position_attr.setName(Qt3DCore.QAttribute.defaultPositionAttributeName())
        position_attr.setVertexBaseType(Qt3DCore.QAttribute.VertexBaseType.Float)
        position_attr.setVertexSize(3)  # 每个顶点有3个分量 (x, y, z)
        position_attr.setAttributeType(Qt3DCore.QAttribute.AttributeType.VertexAttribute)
        position_attr.setBuffer(vertex_buffer)
        position_attr.setByteStride(12)  # 3 * 4 bytes (float)
        position_attr.setCount(len(vertex_data))

        # 绑定到几何体
        self.geometry.addAttribute(position_attr)

        # 创建 LineStrip 渲染器
        self.renderer = Qt3DRender.QGeometryRenderer(self)
        self.renderer.setGeometry(self.geometry)
        self.renderer.setPrimitiveType(Qt3DRender.QGeometryRenderer.PrimitiveType.LineStrip)

        # 设置材质
        # self.material = Qt3DRender.QMaterial(self)
        self.material = Qt3DExtras.QPhongMaterial(self)
        self.material.setDiffuse("red")  # 设为红色，便于观察

        # 绑定组件
        self.addComponent(self.renderer)
        self.addComponent(self.material)


class MainWindow(Qt3DExtras.Qt3DWindow):
    def __init__(self):
        super().__init__()

        # 创建 Qt3D 窗口
        self.defaultFrameGraph().setClearColor("black")

        # 创建 Qt3D 场景
        self.root_entity = Qt3DCore.QEntity()
        self.line_strip = LineStrip(self.root_entity)

        from coordinate import Coordinate
        self.coordinate = Coordinate(self.root_entity, 0.1)

        # 相机设置
        self.camera = self.camera()
        self.camera.lens().setPerspectiveProjection(45.0, 16/9, 0.1, 1000.0)
        self.camera.setPosition(QVector3D(5, 5, 10))
        self.camera.setViewCenter(QVector3D(1, 0, 1))

        # 轨道摄像机控制器
        self.cam_controller = Qt3DExtras.QOrbitCameraController(self.root_entity)
        self.cam_controller.setLinearSpeed(50.0)
        self.cam_controller.setLookSpeed(180.0)
        self.cam_controller.setCamera(self.camera)

        # 关联 Qt3D 窗口和场景
        self.setRootEntity(self.root_entity)


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.resize(400, 300)
    window.show()
    app.exec()
