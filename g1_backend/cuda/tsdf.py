"""Tensor TSDF adapter preserving metric depths and the legacy mesh interface."""

import numpy as np
from ..acceleration import enabled


class CudaTSDFVolume:
    def __init__(self, o3d, voxel_size, depth_max):
        if not enabled() or not o3d.core.cuda.is_available():
            raise RuntimeError('Open3D CUDA unavailable or disabled')
        self.o3d, self.depth_max = o3d, depth_max
        self.voxel_size = voxel_size
        self.device = o3d.core.Device('CUDA:0')
        self.grid = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf', 'weight', 'color'),
            attr_dtypes=(o3d.core.float32, o3d.core.float32, o3d.core.float32),
            attr_channels=((1,), (1,), (3,)),
            voxel_size=voxel_size,
            block_resolution=16,
            block_count=2048,
            device=self.device,
        )

    def integrate_arrays(self, color, depth, intrinsics, extrinsic):
        o3d = self.o3d
        _, _, fx, fy, cx, cy = intrinsics
        depth = o3d.t.geometry.Image(
            o3d.core.Tensor(np.ascontiguousarray(depth), device=self.device)
        )
        color = o3d.t.geometry.Image(
            o3d.core.Tensor(
                np.ascontiguousarray(color, dtype=np.float32) / 255.0, device=self.device
            )
        )
        intrinsic = o3d.core.Tensor(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=o3d.core.float64
        )
        pose = o3d.core.Tensor(extrinsic, dtype=o3d.core.float64)
        blocks = self.grid.compute_unique_block_coordinates(
            depth, intrinsic, pose, 1.0, self.depth_max, 3.0
        )
        # Unique union; existing blocks must not count twice toward the budget.
        _, exists = self.grid.hashmap().find(blocks)
        new_count = int(exists.logical_not().to(o3d.core.int32).sum().item())
        if self.grid.hashmap().size() + new_count > 2048:
            raise RuntimeError('CUDA TSDF block budget exhausted')
        self.grid.integrate(
            blocks, depth, color, intrinsic, intrinsic, pose, 1.0, self.depth_max, 3.0
        )

    def extract_triangle_mesh(self):
        if self.grid.hashmap().size() == 0:
            return self.o3d.geometry.TriangleMesh()
        mesh = self.grid.extract_triangle_mesh(weight_threshold=0.5)
        if len(mesh.vertex.positions) == 0:
            return mesh.to_legacy()
        # Open3D's CUDA extraction interpolates unobserved (zero) color voxels
        # at image boundaries. Assign colors from observed voxels only, keeping
        # a constant-colored measured surface constant at its edges.
        coordinates, indices = self.grid.voxel_coordinates_and_flattened_indices()
        weights = self.grid.attribute('weight').reshape((-1,))[indices]
        observed = weights > 0
        coordinates = coordinates[observed].contiguous()
        colors = self.grid.attribute('color').reshape((-1, 3))[indices[observed]]
        search = self.o3d.core.nns.NearestNeighborSearch(coordinates)
        # Radius indexing avoids a quadratic global nearest-neighbor search.
        radius = self.voxel_size * 1.8
        search.hybrid_index(radius)
        nearest, _, count = search.hybrid_search(mesh.vertex.positions.contiguous(), radius, 1)
        if count.min().item() < 1:
            raise RuntimeError('Mesh vertex has no observed color voxel')
        mesh.vertex.colors = colors[nearest.reshape((-1,))] / 255.0
        return mesh.to_legacy()
