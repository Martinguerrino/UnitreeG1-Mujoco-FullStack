"""Optional image acceleration with CPU-compatible outputs."""

from .geometry import attempt


def distance_transform(mask):
    def compute(cp):
        from cupyx.scipy.ndimage import distance_transform_edt

        return cp.asnumpy(distance_transform_edt(cp.asarray(mask)))

    return attempt('distance_transform', mask.size, compute, minimum=16384)


class Orb:
    """CUDA ORB when OpenCV supports it; unchanged CPU detector otherwise."""

    def __init__(self, cv2, **options):
        self.cv2 = cv2
        self.cpu = cv2.ORB_create(**options)
        self.gpu = None
        from ..acceleration import enabled

        if enabled():
            try:
                if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                    self.gpu = cv2.cuda_ORB.create(**options)
            except (AttributeError, cv2.error):
                pass

    def detectAndCompute(self, image, mask):
        from ..acceleration import disable, enabled

        if self.gpu is not None and enabled() and mask is None:
            try:
                # CUDA ORB accepts grayscale only; CPU ORB also converts RGB input.
                if image.ndim == 3:
                    image = self.cv2.cvtColor(image, self.cv2.COLOR_BGR2GRAY)
                buffer = self.cv2.cuda_GpuMat()
                buffer.upload(image)
                points, descriptors = self.gpu.detectAndComputeAsync(buffer, None)
                keypoints = self.gpu.convert(points)
                return keypoints, descriptors.download() if keypoints else None
            except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
                disable('orb', error)
                self.gpu = None
        return self.cpu.detectAndCompute(image, mask)


def overlay(rgb, instances, colors):
    def compute(cp):
        result = cp.asarray(rgb).copy()
        for instance in instances:
            mask = cp.asarray(instance.mask)
            color = cp.asarray(colors.get(instance.label, (200, 200, 200)), dtype=cp.uint8)
            result[mask] = (0.55 * result[mask] + 0.45 * color).astype(cp.uint8)
        return cp.asnumpy(result)

    return attempt('overlay', rgb.size, compute, minimum=1000000)


def depth_colors(values, minimum, maximum):
    def compute(cp):
        import cv2
        import numpy as np

        depth = cp.asarray(values, dtype=cp.float32)
        valid = cp.isfinite(depth) & (depth >= minimum) & (depth <= maximum)
        safe = cp.where(valid, depth, minimum)
        scaled = cp.rint(cp.clip((safe - minimum) / (maximum - minimum), 0, 1) * 255).astype(
            cp.uint8
        )
        colors = cv2.cvtColor(
            cv2.applyColorMap(np.arange(256, dtype=np.uint8)[:, None], cv2.COLORMAP_TURBO),
            cv2.COLOR_BGR2RGB,
        ).reshape(256, 3)
        result = cp.asarray(colors)[scaled]
        result[~valid] = cp.asarray([100, 0, 100], dtype=cp.uint8)
        return cp.asnumpy(result)

    return attempt('depth_preview', values.size, compute, minimum=131072)
