"""Independent CUDA ray intersections, checked against MuJoCo before use.

MuJoCo still owns physics and geom poses. Unsupported heightfields/SDFs use
mj_multiRay. A backend must pass full parity and warm timing checks before auto
selection, with sampled validation of subsequent scans (including geom IDs).
"""

import time
import weakref

import numpy as np
from ..acceleration import cupy, enabled, disable

_CACHE = weakref.WeakKeyDictionary()
_SOURCE = r"""
__device__ double dot3(const double* a, const double* b) {
  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
__device__ void cross3(const double* a,const double* b,double* c) {
  c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}
__device__ void roots(double a,double b,double c,double* lo,double* hi) {
  double det=b*b-a*c; *lo=*hi=-1;
  if (a<1e-15 || det<0) return;
  *lo=(-b-sqrt(det))/a; *hi=(-b+sqrt(det))/a;
}
__device__ void accept(double value,double* best) {
  if(value>=0 && value<*best) *best=value;
}
extern "C" __global__ void cast(
 const double* origin,const double* dirs,int nr,const int* ids,const int* types,
 const double* sizes,const double* poses,const double* rotations,int ng,
 const int* starts,const int* counts,const double* triangles,
 double* distances,int* hits,double maximum) {
 int r=blockDim.x*blockIdx.x+threadIdx.x; if(r>=nr) return;
 const double* dir=dirs+3*r; double best=maximum+1; int hit=-1;
 for(int g=0;g<ng;g++) {
   double p[3],v[3],diff[3]; const double* mat=rotations+9*g;
   for(int j=0;j<3;j++) diff[j]=origin[j]-poses[3*g+j];
   for(int j=0;j<3;j++) {
     p[j]=mat[j]*diff[0]+mat[3+j]*diff[1]+mat[6+j]*diff[2];
     v[j]=mat[j]*dir[0]+mat[3+j]*dir[1]+mat[6+j]*dir[2];
   }
   const double* s=sizes+3*g; int type=types[g]; double t=1e300,lo,hi;
   if(type==0) {
     if(v[2]<-1e-15) {
       double x=-p[2]/v[2];
       if((s[0]<=0 || fabs(p[0]+x*v[0])<=s[0]) &&
          (s[1]<=0 || fabs(p[1]+x*v[1])<=s[1])) accept(x,&t);
     }
   } else if(type==2 || type==4) {
     double q[3],d[3];
     for(int j=0;j<3;j++) { double scale=type==2?s[0]:s[j]; q[j]=p[j]/scale; d[j]=v[j]/scale; }
     roots(dot3(d,d),dot3(d,q),dot3(q,q)-1,&lo,&hi);
     accept(lo,&t);accept(hi,&t);
   } else if(type==3 || type==5) {
     roots(v[0]*v[0]+v[1]*v[1],v[0]*p[0]+v[1]*p[1],
           p[0]*p[0]+p[1]*p[1]-s[0]*s[0],&lo,&hi);
     if(fabs(p[2]+lo*v[2])<=s[1]) accept(lo,&t);
     if(fabs(p[2]+hi*v[2])<=s[1]) accept(hi,&t);
     for(int side=-1;side<=1;side+=2) {
       if(type==5) {
         if(fabs(v[2])>1e-15) {
           double x=(side*s[1]-p[2])/v[2];
           double xx=p[0]+x*v[0],yy=p[1]+x*v[1];
           if(xx*xx+yy*yy<=s[0]*s[0]) accept(x,&t);
         }
       } else {
         double q[3]={p[0],p[1],p[2]-side*s[1]};
         roots(dot3(v,v),dot3(v,q),dot3(q,q)-s[0]*s[0],&lo,&hi);
         if(side*(p[2]+lo*v[2]-side*s[1])>=0) accept(lo,&t);
         if(side*(p[2]+hi*v[2]-side*s[1])>=0) accept(hi,&t);
       }
     }
   } else if(type==6) {
     for(int axis=0;axis<3;axis++) if(fabs(v[axis])>1e-15) {
       for(int side=-1;side<=1;side+=2) {
         double x=(side*s[axis]-p[axis])/v[axis]; bool valid=x>=0;
         for(int j=0;j<3;j++) if(j!=axis && fabs(p[j]+x*v[j])>s[j]) valid=false;
         if(valid) accept(x,&t);
       }
     }
   } else if(type==7) {
     // Bound each mesh with its local box before walking its triangle buffer.
     bool possible=true; double entry=-1e300,exit=1e300;
     for(int j=0;j<3;j++) {
       if(fabs(v[j])<1e-15) { if(fabs(p[j])>s[j]) possible=false; }
       else { double a=(-s[j]-p[j])/v[j], b=(s[j]-p[j])/v[j];
         entry=fmax(entry,fmin(a,b));exit=fmin(exit,fmax(a,b)); }
     }
     if(entry>exit || exit<0 || entry>best) possible=false;
     if(possible) for(int k=starts[g];k<starts[g]+counts[g];k++) {
       const double* a=triangles+9*k; double e1[3],e2[3],delta[3],h[3],q[3];
       for(int j=0;j<3;j++) { e1[j]=a[3+j]-a[j]; e2[j]=a[6+j]-a[j]; delta[j]=p[j]-a[j]; }
       cross3(v,e2,h); double det=dot3(e1,h); if(fabs(det)<1e-15) continue;
       double u=dot3(delta,h)/det; if(u<0 || u>1) continue;
       cross3(delta,e1,q); double w=dot3(v,q)/det; if(w<0 || u+w>1) continue;
       accept(dot3(e2,q)/det,&t);
     }
   }
   if(t<best) {best=t;hit=ids[g];}
 }
 if(best>maximum) hit=-1;
 distances[r]=hit<0?-1:best; hits[r]=hit;
}
"""


class Raycaster:
    def __init__(self, model, groups, cp):
        self.cp, self.groups = cp, tuple(groups)
        alpha = model.geom_rgba[:, 3].copy()
        material = model.geom_matid >= 0
        alpha[material] = model.mat_rgba[model.geom_matid[material], 3]
        self.ids = np.flatnonzero(
            (alpha != 0) & np.asarray(groups)[np.clip(model.geom_group, 0, 5)].astype(bool)
        )
        if not np.isin(model.geom_type[self.ids], [0, 2, 3, 4, 5, 6, 7]).all():
            raise RuntimeError('Heightfields/SDFs require MuJoCo raycasting')
        types = model.geom_type[self.ids].astype(np.int32)
        sizes = model.geom_size[self.ids].copy()
        triangles, starts, counts = [], [], []
        offset = 0
        for index, geom in enumerate(self.ids):
            starts.append(offset)
            count = 0
            if types[index] == 7:
                mesh = model.geom_dataid[geom]
                va, vn = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                fa, fn = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
                vertices = model.mesh_vert[va : va + vn].astype(np.float64)
                faces = model.mesh_face[fa : fa + fn]
                triangles.append(vertices[faces].reshape(-1, 9))
                sizes[index] = np.max(np.abs(vertices), axis=0)
                count = fn
            counts.append(count)
            offset += count
        self.ids_gpu = cp.asarray(self.ids, dtype=cp.int32)
        self.types, self.sizes = cp.asarray(types), cp.asarray(sizes)
        self.starts, self.counts = (
            cp.asarray(starts, dtype=cp.int32),
            cp.asarray(counts, dtype=cp.int32),
        )
        self.triangles = cp.asarray(np.concatenate(triangles) if triangles else np.empty((0, 9)))
        self.kernel = cp.RawKernel(_SOURCE, 'cast')
        self.selected = None
        self.capacity = 0

    def cast(self, data, origin, directions, maximum):
        cp = self.cp
        n = len(directions)
        if n != self.capacity:
            self.capacity = n
            self.distances = cp.empty(n, dtype=cp.float64)
            self.hits = cp.empty(n, dtype=cp.int32)
        poses = cp.asarray(data.geom_xpos[self.ids])
        rotations = cp.asarray(data.geom_xmat[self.ids])
        self.kernel(
            ((n + 127) // 128,),
            (128,),
            (
                cp.asarray(origin, dtype=cp.float64),
                cp.asarray(directions, dtype=cp.float64),
                np.int32(n),
                self.ids_gpu,
                self.types,
                self.sizes,
                poses,
                rotations,
                np.int32(len(self.ids)),
                self.starts,
                self.counts,
                self.triangles,
                self.distances,
                self.hits,
                np.float64(maximum),
            ),
        )
        return cp.asnumpy(self.distances), cp.asnumpy(self.hits)


def multi_ray(model, data, origin, directions, groups, maximum):
    """Return (distances, geom IDs), or None to execute the unchanged CPU call."""
    if not enabled() or len(directions) == 0:
        return None
    cp = cupy()
    if cp is None:
        return None
    key = tuple(groups)
    cache = _CACHE.setdefault(model, {})
    if cache.get(key) is False:
        return None
    try:
        caster = cache.get(key)
        if caster is None:
            caster = Raycaster(model, groups, cp)
            cache[key] = caster
        if caster.selected is False:
            return None
        import mujoco

        def reference(vec):
            dist = np.empty(len(vec), dtype=np.float64)
            ids = np.empty(len(vec), dtype=np.int32)
            mujoco.mj_multiRay(
                model,
                data,
                origin,
                np.ascontiguousarray(vec).ravel(),
                groups,
                1,
                -1,
                ids,
                dist,
                len(vec),
                maximum,
            )
            # mj_multiRay can return beyond cutoff for intersected bounding spheres.
            far = dist > maximum
            dist[far], ids[far] = -1, -1
            return dist, ids

        output = caster.cast(data, origin, directions, maximum)
        if caster.selected is None:
            start = time.perf_counter()
            expected = reference(directions)
            cpu_time = time.perf_counter() - start
            start = time.perf_counter()
            output = caster.cast(data, origin, directions, maximum)
            gpu_time = time.perf_counter() - start
            parity = np.allclose(output[0], expected[0], rtol=1e-7, atol=1e-8)
            parity &= np.array_equal(output[1], expected[1])
            caster.selected = bool(parity and gpu_time < cpu_time)
            if not caster.selected:
                return None
        else:
            sample = np.linspace(0, len(directions) - 1, min(16, len(directions)), dtype=int)
            expected = reference(directions[sample])
            if not (
                np.allclose(output[0][sample], expected[0], rtol=1e-7, atol=1e-8)
                and np.array_equal(output[1][sample], expected[1])
            ):
                caster.selected = False
                return None
        return output
    except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
        cache[key] = False
        disable('lidar', error)
        return None
