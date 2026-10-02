"""Reader for the bounded, colored triangle PLY produced by cloud_mesh."""
import numpy as np


def read_mesh(path):
    with open(path, encoding='ascii') as stream:
        if stream.readline().strip() != 'ply' or stream.readline().strip() != 'format ascii 1.0':
            raise ValueError('Expected an ASCII PLY mesh')
        vertices = faces = None
        for line in stream:
            fields = line.split()
            if fields[:2] == ['element', 'vertex']:
                vertices = int(fields[2])
            elif fields[:2] == ['element', 'face']:
                faces = int(fields[2])
            elif fields == ['end_header']:
                break
        if vertices is None or faces is None or not 0 <= vertices <= 50000 or not 0 <= faces <= 100000:
            raise ValueError('Invalid mesh size')
        points = np.array([list(map(float, stream.readline().split())) for _ in range(vertices)],
                          dtype=np.float32).reshape(vertices, 6)
        triangles = np.array([list(map(int, stream.readline().split())) for _ in range(faces)],
                             dtype=np.int32).reshape(faces, 4)
        if not np.isfinite(points).all() or np.any(points[:, 3:] < 0) or np.any(points[:, 3:] > 255):
            raise ValueError('Invalid vertices or colors')
        if np.any(triangles[:, 0] != 3) or np.any(triangles[:, 1:] < 0) or np.any(triangles[:, 1:] >= vertices):
            raise ValueError('Invalid triangle indices')
    return points[:, :3], points[:, 3:].astype(np.uint8), triangles[:, 1:]
