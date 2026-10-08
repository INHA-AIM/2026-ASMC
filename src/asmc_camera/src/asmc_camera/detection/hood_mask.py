"""YOLO-only exclusion, preserving the source image and its coordinate system."""
import hashlib
import math
from pathlib import Path
import threading

import cv2
import numpy as np
import yaml


def load_view_masks(views, front_mask, side_enabled=False, side_configs=None):
    """Reuse exclusion geometry while keeping each camera's calibration separate."""
    if not views or len(set(views)) != len(views) or set(views)-{'front', 'left', 'right'}:
        raise ValueError('ego masks require unique front/left/right views')
    masks = {view: front_mask if view == 'front' else HoodMask() for view in views}
    if side_enabled:
        if not isinstance(side_configs, dict) or set(side_configs)-{'left', 'right'}:
            raise ValueError('side ego mask configs must map left/right to their own files')
        for view in views:
            if view == 'front':
                continue
            if not side_configs.get(view):
                raise ValueError('missing ego mask config for ' + view)
            masks[view] = HoodMask.load(True, side_configs[view])
    return masks


class HoodMask:
    def __init__(self, polygon=None, config_sha256="", fill_bgr=(0,0,0), inset_pixels=0):
        self.enabled = polygon is not None
        self.config_sha256 = config_sha256
        self.polygon = self._validate(polygon) if self.enabled else ()
        if (not isinstance(fill_bgr,(tuple,list)) or len(fill_bgr)!=3 or
                any(isinstance(v,bool) or not isinstance(v,int) or not 0<=v<=255 for v in fill_bgr)):
            raise ValueError('hood mask fill_bgr requires three integer channels in [0,255]')
        self.fill_bgr=tuple(fill_bgr)
        if isinstance(inset_pixels,bool) or not isinstance(inset_pixels,int) or not 0<=inset_pixels<=16:
            raise ValueError('hood mask inset_pixels must be an integer in [0,16]')
        self.inset_pixels=inset_pixels
        self._cache = {}
        self._lock = threading.Lock()
        self._last_geometry = {}

    @staticmethod
    def _validate(polygon):
        try:
            points = tuple(tuple(float(v) for v in p) for p in polygon)
        except (TypeError, ValueError) as exc:
            raise ValueError("hood mask polygon must contain normalized xy pairs") from exc
        if len(points) < 3 or any(len(p) != 2 for p in points):
            raise ValueError("hood mask polygon requires at least three xy pairs")
        if any(not math.isfinite(v) or not 0 <= v <= 1 for p in points for v in p):
            raise ValueError("hood mask coordinates must be finite and in [0,1]")
        if len(set(points)) != len(points):
            raise ValueError("hood mask polygon has repeated vertices")
        def cross(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        def intersects(a, b, c, d):
            if max(a[0], b[0]) < min(c[0], d[0]) or max(c[0], d[0]) < min(a[0], b[0]):
                return False
            if max(a[1], b[1]) < min(c[1], d[1]) or max(c[1], d[1]) < min(a[1], b[1]):
                return False
            return cross(a,b,c)*cross(a,b,d) <= 0 and cross(c,d,a)*cross(c,d,b) <= 0
        for i in range(len(points)):
            for j in range(i+1, len(points)):
                if j == i+1 or (i == 0 and j == len(points)-1):
                    continue
                if intersects(points[i], points[(i+1)%len(points)], points[j], points[(j+1)%len(points)]):
                    raise ValueError("hood mask polygon self-intersects")
        area = abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))) / 2
        if not 0 < area < 1:
            raise ValueError("hood mask must exclude a nonempty proper subset of the image")
        return points

    @classmethod
    def load(cls, enabled, config_path):
        if not enabled:
            return cls()
        raw = Path(config_path).read_bytes()
        config = yaml.safe_load(raw)
        if not isinstance(config,dict): raise ValueError('hood mask config must be a mapping')
        if config.get('version')==1 and set(config)=={'version','polygon'}:
            fill=(0,0,0)
        elif config.get('version')==2 and set(config) in ({'version','polygon','fill_bgr'},{'version','polygon','fill_bgr','inset_pixels'}):
            fill=config['fill_bgr']
        else:
            raise ValueError('hood mask config requires version1/polygon or version2/polygon/fill_bgr')
        return cls(config["polygon"], hashlib.sha256(raw).hexdigest(),fill,config.get('inset_pixels',0))

    def geometry(self, height, width):
        if height < 1 or width < 1:
            raise ValueError("hood mask requires positive image dimensions")
        key = (height, width)
        with self._lock:
            if key not in self._cache:
                pixels = np.array([(min(width-1, int(x*width)), min(height-1, int(y*height)))
                                   for x,y in self.polygon], dtype=np.int32).reshape(-1,2)
                bitmap = np.zeros((height,width), dtype=np.uint8)
                if self.enabled:
                    cv2.fillPoly(bitmap, [pixels], 1)
                    if self.inset_pixels:
                        size=2*self.inset_pixels+1
                        bitmap=cv2.erode(bitmap,np.ones((size,size),np.uint8))
                excluded = int(np.count_nonzero(bitmap))
                if self.enabled and (excluded == 0 or excluded == height*width):
                    raise ValueError("hood mask raster leaves no excluded or observable pixels")
                if self.enabled and self.inset_pixels:
                    contours,_=cv2.findContours(bitmap,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    if len(contours)!=1:
                        raise ValueError('inset mask is disconnected; refusing to silently omit components')
                    pixels=contours[0][:,0,:]
                bitmap = bitmap.astype(bool)
                bitmap.setflags(write=False)
                pixels.setflags(write=False)
                visible = np.logical_not(bitmap).astype(np.uint8)*255
                visible.setflags(write=False)
                self._cache[key] = (bitmap, pixels, excluded, visible)
                # Bounding memory for malformed streams that change shape repeatedly.
                if len(self._cache) > 4:
                    del self._cache[next(iter(self._cache))]
            bitmap, pixels, excluded, _ = self._cache[key]
            self._last_geometry = dict(image_width=width, image_height=height,
                                       polygon_pixels=pixels.tolist(), excluded_pixels=excluded)
            return bitmap, pixels

    def apply(self, image):
        if not self.enabled:
            return image
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("hood mask expects a decoded uint8 BGR image")
        self.geometry(*image.shape[:2])
        with self._lock:
            visible = self._cache[image.shape[:2]][3]
        if self.fill_bgr==(0,0,0): return cv2.copyTo(image, visible)
        result=np.empty_like(image);result[:]=self.fill_bgr
        return cv2.copyTo(image,visible,result)

    @property
    def info(self):
        with self._lock:
            return dict(enabled=self.enabled, config_sha256=self.config_sha256,
                        fill_bgr=list(self.fill_bgr),inset_pixels=self.inset_pixels,
                        polygon_normalized=[list(p) for p in self.polygon], **self._last_geometry)
