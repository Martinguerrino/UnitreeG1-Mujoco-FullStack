"""Spatially hashed radius counts and connected components; no dense N x N graph."""

import numpy as np
from .geometry import attempt

_SOURCE = r"""
__device__ int root(int* parent,int i) {
  while(parent[i]!=i) i=parent[i]; return i;
}
__device__ void join(int* parent,int a,int b) {
  while(true) {
    a=root(parent,a);b=root(parent,b);if(a==b) return;
    int high=max(a,b),low=min(a,b);
    if(atomicCAS(parent+high,high,low)==high) return;
  }
}
extern "C" __global__ void radius(
 const double* points,const long long* coords,const long long* keys,
 const long long* order,int n,long long ey,long long ez,double radius2,
 int* parent,int* counts,int connect) {
 int i=blockIdx.x*blockDim.x+threadIdx.x; if(i>=n) return;
 int count=0;
 for(int x=-1;x<=1;x++) for(int y=-1;y<=1;y++) for(int z=-1;z<=1;z++) {
   long long cx=coords[3*i]+x,cy=coords[3*i+1]+y,cz=coords[3*i+2]+z;
   if(cx<0 || cy<0 || cz<0 || cy>=ey || cz>=ez) continue;
   long long key=(cx*ey+cy)*ez+cz;
   int lower=0,upper=n;
   while(lower<upper) {int mid=(lower+upper)/2;if(keys[mid]<key) lower=mid+1;else upper=mid;}
   for(int at=lower;at<n && keys[at]==key;at++) {
     int j=order[at];double squared=0;
     for(int axis=0;axis<3;axis++) {double d=points[3*i+axis]-points[3*j+axis];squared+=d*d;}
     if(squared<=radius2) {count++;if(connect) join(parent,i,j);}
   }
 }
 counts[i]=count;
}
extern "C" __global__ void compress(int* parent,int n) {
 int i=blockIdx.x*blockDim.x+threadIdx.x;if(i<n) parent[i]=root(parent,i);
}
"""


def _radius(cp, points, radius, connect):
    if not np.isfinite(points).all() or not np.isfinite(radius) or radius <= 0:
        raise ValueError('Spatial hashing requires finite points and positive radius')
    cloud = cp.asarray(points, dtype=cp.float64)
    coords = cp.floor(cloud / radius).astype(cp.int64)
    coords -= coords.min(axis=0)
    extent = cp.asnumpy(coords.max(axis=0)) + 2
    if int(extent[0]) * int(extent[1]) * int(extent[2]) > np.iinfo(np.int64).max:
        raise RuntimeError('Spatial key range exceeds int64')
    ey, ez = int(extent[1]), int(extent[2])
    keys = (coords[:, 0] * ey + coords[:, 1]) * ez + coords[:, 2]
    order = cp.argsort(keys).astype(cp.int64)
    keys = keys[order]
    n = len(cloud)
    labels = cp.arange(n, dtype=cp.int32)
    counts = cp.empty(n, dtype=cp.int32)
    kernel = cp.RawKernel(_SOURCE, 'radius')
    kernel(
        ((n + 127) // 128,),
        (128,),
        (
            cloud,
            coords,
            keys,
            order,
            np.int32(n),
            np.int64(ey),
            np.int64(ez),
            np.float64(radius**2),
            labels,
            counts,
            np.int32(connect),
        ),
    )
    if connect:
        cp.RawKernel(_SOURCE, 'compress')(((n + 127) // 128,), (128,), (labels, np.int32(n)))
        labels = cp.asnumpy(labels)
        # Components are emitted in original start-index order, matching CPU traversal.
        unique = np.unique(labels)
        return [np.flatnonzero(labels == label) for label in unique]
    return cp.asnumpy(counts)


def radius_counts(points, radius):
    return attempt(
        'radius_counts', len(points), lambda cp: _radius(cp, points, radius, False), minimum=4096
    )


def components(points, radius):
    return attempt(
        'components', len(points), lambda cp: _radius(cp, points, radius, True), minimum=4096
    )
