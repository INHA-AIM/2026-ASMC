"""Diagnostic projection using the pinned filter's coordinate conventions."""
import math
import struct
import numpy as np


def rotation(roll, pitch, yaw=0):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return rz @ ry @ rx


def projection(vehicle):
    def pose(sensor):
        t = np.eye(4)
        t[:3,:3] = rotation(*(math.radians(sensor[k]) for k in ('roll','pitch','yaw')))
        t[:3,3] = [sensor[k] for k in ('x','y','z')]
        return t
    camera = vehicle['Camera']
    k = np.array([[camera['fx'],0,camera['cx'],0],
                  [0,camera['fy'],camera['cy'],0],[0,0,1,0]])
    optical = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])
    return k @ optical @ np.linalg.inv(pose(camera)) @ pose(vehicle['LiDAR'])


def project_leveled(xyz, roll, pitch, p):
    # Column-vector leveling = Ry(pitch) Rx(roll); its inverse is its transpose.
    raw = np.asarray(xyz) @ rotation(roll,pitch)
    homogeneous = np.column_stack([raw,np.ones(len(raw))]) @ p.T
    positive = homogeneous[:,2] > 1e-3
    uv = np.full((len(raw),2),np.nan)
    uv[positive] = homogeneous[positive,:2] / homogeneous[positive,2,None]
    return uv, positive


def clouds(path, requested):
    """Read the original evaluator's little-endian cloud stream, rejecting truncation."""
    result = {}
    with open(path,'rb') as f:
        while True:
            header = f.read(20)
            if not header: break
            if len(header) != 20: raise ValueError('truncated cloud header')
            index,stamp,na,nb = struct.unpack('<idii',header)
            if min(na,nb)<0 or max(na,nb)>10000000: raise ValueError('invalid cloud size')
            size=12*na+16*nb
            data=f.read(size)
            if len(data)!=size: raise ValueError('truncated cloud payload')
            if index not in requested: continue
            a=np.frombuffer(data,dtype='<f4',count=na*3).reshape(-1,3).copy()
            b=np.frombuffer(data,dtype='<f4',count=nb*3,offset=12*na).reshape(-1,3).copy()
            tags=np.frombuffer(data,dtype=np.dtype([('label','u1'),('cluster','<i2'),('protect','u1')]),
                               count=nb,offset=12*(na+nb)).copy()
            result[index]=dict(stamp=stamp,A=a,B=b,tags=tags)
    if set(requested)-set(result): raise ValueError('requested cloud frame missing')
    return result
