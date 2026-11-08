"""Batched bilinear resize, box cropping and thresholding without logits readback."""

import numpy as np

_SOURCE = r"""
extern "C" __global__ void masks(
 const float* logits,const int* boxes,unsigned char* masks,
 int n,int height,int width,int ph,int pw,int left,int top,int cw,int ch) {
 int at=blockIdx.x*blockDim.x+threadIdx.x;if(at>=n*height*width) return;
 int item=at/(height*width),pixel=at%(height*width),x=pixel%width,y=pixel/width;
 const int* box=boxes+4*item;
 if(x<box[0] || y<box[1] || x>=box[2] || y>=box[3]) {masks[at]=0;return;}
 float sx=(float)(((double)x+.5)*cw/width-.5);
 float sy=(float)(((double)y+.5)*ch/height-.5);
 sx=fmaxf(0,fminf(cw-1,sx));sy=fmaxf(0,fminf(ch-1,sy));
 int x0=(int)sx,y0=(int)sy,x1=min(x0+1,cw-1),y1=min(y0+1,ch-1);
 float ax=sx-x0,ay=sy-y0;const float* p=logits+item*ph*pw;
 float a=p[(top+y0)*pw+left+x0],b=p[(top+y0)*pw+left+x1];
 float c=p[(top+y1)*pw+left+x0],d=p[(top+y1)*pw+left+x1];
 float value=(a*(1-ax)+b*ax)*(1-ay)+(c*(1-ax)+d*ax)*ay;
 masks[at]=value>0;
}
"""


def resize_masks(cp, logits, boxes, image_shape):
    height, width = image_shape
    ph, pw = logits.shape[1:]
    gain = min(pw / width, ph / height)
    ml, mt = (pw - width * gain) / 2, (ph - height * gain) / 2
    left, top = round(ml - 0.1), round(mt - 0.1)
    cw, ch = pw - left - round(ml + 0.1), ph - top - round(mt + 0.1)
    result = cp.empty((len(boxes), height, width), dtype=cp.uint8)
    kernel = cp.RawKernel(_SOURCE, 'masks', options=('--fmad=false',))
    count = result.size
    kernel(
        ((count + 255) // 256,),
        (256,),
        (
            logits,
            cp.asarray(boxes, dtype=cp.int32),
            result,
            np.int32(len(boxes)),
            np.int32(height),
            np.int32(width),
            np.int32(ph),
            np.int32(pw),
            np.int32(left),
            np.int32(top),
            np.int32(cw),
            np.int32(ch),
        ),
    )
    return cp.asnumpy(result).astype(bool)
