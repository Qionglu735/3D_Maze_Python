
from PySide6.Qt3DRender import Qt3DRender

from global_config import root_entity


class Layer:
    _dict = {
        "scene": Qt3DRender.QLayer(root_entity),
        "ui": Qt3DRender.QLayer(root_entity),
    }
    # camera_layer["scene"].setRecursive(True)

    @classmethod
    def get(cls, name):
        if name in cls._dict:
            return cls._dict[name]
        else:
            return None


class Viewport:

    """
    QRenderSurfaceSelector
        QViewport
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

        self.viewport = Qt3DRender.QViewport(surface_selector)
        self.render_capture = Qt3DRender.QRenderCapture(self.viewport)
        self.layer_filter = Qt3DRender.QLayerFilter(self.render_capture)
        self.sort_policy = Qt3DRender.QSortPolicy(self.layer_filter)
        self.camera_selector = Qt3DRender.QCameraSelector(self.sort_policy)
        self.camera_selector.setCamera(self.camera)

        self.sort_policy.setSortTypes([
            # Qt3DRender.QSortPolicy.SortType.BackToFront,
            # Qt3DRender.QSortPolicy.SortType.Material,
            # Qt3DRender.QSortPolicy.SortType.StateChangeCost,
        ])

        # self.clear_buffers = Qt3DRender.QClearBuffers(self.camera_selector)
        # self.clear_buffers.setBuffers(Qt3DRender.QClearBuffers.BufferType.DepthBuffer)
        # self.clear_buffers.setBuffers(Qt3DRender.QClearBuffers.BufferType.AllBuffers)
