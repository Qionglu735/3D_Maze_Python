import sys
import pybullet as p
import pybullet_data
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer, Qt, QPoint
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.Qt3DCore import Qt3DCore
from PySide6.QtGui import QGuiApplication, QMatrix4x4, QQuaternion, QVector3D, QColor, QSurfaceFormat, QFont


class BulletPhysics:
    def __init__(self):
        # self.physics_client = p.connect(p.DIRECT)  # 物理模拟（无 GUI）
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置 URDF 搜索路径
        p.setGravity(0, -9.8, 0)  # 设置重力

        # 创建地面
        self.plane_id = p.loadURDF("plane.urdf")

        # 创建一个立方体（刚体）
        self.box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[25, 25, 25])
        self.box_body = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=self.box_id, basePosition=[0, 50, 0])

        # 墙体参数
        wall_thickness = 0.2  # 墙的厚度
        wall_height = 2.0  # 墙的高度
        room_size = 5.0  # 房间的宽度（X-Y 方向）

        # 创建墙的形状（长方体）
        wall_shape = p.createCollisionShape(p.GEOM_BOX,
                                            halfExtents=[room_size / 2, wall_thickness / 2, wall_height / 2])

        # 创建四面墙（每面墙的位置不同）
        walls = [
            # 前墙 (Y+)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_shape,
                              basePosition=[0, room_size / 2, wall_height / 2]),

            # 后墙 (Y-)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_shape,
                              basePosition=[0, -room_size / 2, wall_height / 2]),

            # 左墙 (X-)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_shape,
                              basePosition=[-room_size / 2, 0, wall_height / 2],
                              baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57])),  # 旋转 90 度

            # 右墙 (X+)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_shape,
                              basePosition=[room_size / 2, 0, wall_height / 2],
                              baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57]))  # 旋转 90 度
        ]

        # 添加一个测试刚体（立方体）
        box_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
        box_body = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=box_shape, basePosition=[0, 0, 1])

    def step_simulation(self):
        """执行一步物理模拟"""
        p.stepSimulation()
        pos, _ = p.getBasePositionAndOrientation(self.box_body)  # 获取物体位置
        return pos


class Qt3DBulletExample(Qt3DExtras.Qt3DWindow):
    """Qt3D + Bullet 示例"""
    def __init__(self):
        super().__init__()

        # 设置 3D 视图
        self.root_entity = Qt3DCore.QEntity()
        self.setRootEntity(self.root_entity)

        self.setWidth(160)
        self.setHeight(90)

        self.setPosition(QPoint(2500, 1800))

        self.defaultFrameGraph().setClearColor(QColor(31, 31, 31))

        # 设置摄像机
        camera = self.camera()
        camera.lens().setPerspectiveProjection(45.0, 16.0/9.0, 0.1, 1000.0)
        camera.setPosition(QVector3D(0, 50, 25))
        camera.setViewCenter(QVector3D(0, 0, 0))

        # 轨道摄像机控制
        self.camera_controller = Qt3DExtras.QOrbitCameraController(self.root_entity)
        self.camera_controller.setLinearSpeed(100)
        self.camera_controller.setLookSpeed(100)
        self.camera_controller.setCamera(camera)

        from coordinate import Coordinate

        self.coordinate = Coordinate(self.root_entity)

        # 添加立方体
        self.box_entity = self.create_box()

        # 物理模拟
        self.physics = BulletPhysics()

        # 定时器更新物理
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_physics)
        self.timer.start(1000)  # 约 60 FPS

    def create_box(self):
        """创建一个 3D 立方体"""
        box_entity = Qt3DCore.QEntity(self.root_entity)

        # 立方体网格
        self.mesh = Qt3DExtras.QCuboidMesh()
        box_entity.addComponent(self.mesh)

        # 立方体材质
        self.material = Qt3DExtras.QPhongMaterial()
        box_entity.addComponent(self.material)

        # 立方体变换
        self.transform = Qt3DCore.QTransform()
        box_entity.addComponent(self.transform)

        return box_entity

    def update_physics(self):
        """更新物理模拟，并同步 3D 物体位置"""
        pos = self.physics.step_simulation()
        print(pos)
        self.transform.setTranslation(QVector3D(pos[0], pos[1], pos[2]))  # 设置立方体位置


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Qt3DBulletExample()
    window.show()
    sys.exit(app.exec())