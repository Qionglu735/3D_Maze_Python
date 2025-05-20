
from PySide6.Qt3DRender import Qt3DRender

from global_config import root_entity


class Layer:
    _dict = dict()
    # camera_layer["scene"].setRecursive(True)

    @classmethod
    def get(cls, name):
        if name not in cls._dict:
            cls._dict[name] = Qt3DRender.QLayer(root_entity)
        return cls._dict[name]


class Viewport:

    """
    QRenderSurfaceSelector
        QViewport
            QPickingSettings
                QRenderCapture
                    QLayerFilter
                        QSortPolicy
                            QCameraSelector
                                QClearBuffers
    """

    def __init__(self, surface_selector, camera=None):
        if camera is not None:
            self.camera = camera
        else:
            self.camera = Qt3DRender.QCamera(root_entity)

        self.picking_settings = Qt3DRender.QPickingSettings(surface_selector)
        self.picking_settings.setPickMethod(Qt3DRender.QPickingSettings.PickMethod.TrianglePicking)
        self.picking_settings.setPickResultMode(Qt3DRender.QPickingSettings.PickResultMode.AllPicks)
        self.picking_settings.setFaceOrientationPickingMode(Qt3DRender.QPickingSettings.FaceOrientationPickingMode.FrontAndBackFace)

        self.viewport = Qt3DRender.QViewport(self.picking_settings)

        # self.render_capture = Qt3DRender.QRenderCapture(self.picking_settings)
        self.render_capture = Qt3DRender.QRenderCapture(self.viewport)

        self.layer_filter = Qt3DRender.QLayerFilter(self.render_capture)

        self.sort_policy = Qt3DRender.QSortPolicy(self.layer_filter)
        self.sort_policy.setSortTypes([
            # Qt3DRender.QSortPolicy.SortType.BackToFront,
            # Qt3DRender.QSortPolicy.SortType.Material,
            # Qt3DRender.QSortPolicy.SortType.StateChangeCost,
        ])

        self.camera_selector = Qt3DRender.QCameraSelector(self.sort_policy)
        self.camera_selector.setCamera(self.camera)

        # self.clear_buffers = Qt3DRender.QClearBuffers(self.camera_selector)
        # self.clear_buffers.setBuffers(Qt3DRender.QClearBuffers.BufferType.DepthBuffer)
        # self.clear_buffers.setBuffers(Qt3DRender.QClearBuffers.BufferType.AllBuffers)
