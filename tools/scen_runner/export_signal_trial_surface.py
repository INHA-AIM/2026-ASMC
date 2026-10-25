#!/usr/bin/env python3
"""Read an operator-owned MORAI scene bundle; export an independent road oracle.

Requires UnityPy in an isolated inspection environment (Python >=3.9), numpy,
scipy and shapely. Never changes a simulator asset. Raw geometry belongs in an
ignored runtime directory. The reviewed object names and frame transform below
apply to R_KR_PR_K-city_2025 v.S4.251001.MolitComp03 only.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import Point,Polygon
from shapely.ops import unary_union


def transform_matrix(t):
    q=t['m_LocalRotation'];x,y,z,w=(q[k] for k in ('x','y','z','w'))
    m=np.eye(4)
    m[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    m[:3,:3]*=np.array([t['m_LocalScale'][k] for k in ('x','y','z')])[None,:]
    m[:3,3]=[t['m_LocalPosition'][k] for k in ('x','y','z')]
    return m


def extract(bundle):
    import UnityPy
    from UnityPy.helpers.MeshHelper import MeshHandler
    environment=UnityPy.load(str(bundle))
    transforms={o.path_id:o.read_typetree() for o in environment.objects if o.type.name=='Transform'}
    by_object={t['m_GameObject']['m_PathID']:t for t in transforms.values()}
    expected={'road-mesh-subset1-1','road-mesh-subset2-0','road-mesh-subset3-0'};meshes=[]
    for obj in environment.objects:
        if obj.type.name!='MeshCollider':continue
        collider=obj.read();go=collider.m_GameObject.read()
        if go.m_Name not in expected:continue
        if not collider.m_Enabled or collider.m_IsTrigger or not go.m_IsActive:
            raise ValueError('reviewed pavement collider disabled/triggered')
        matrix=np.eye(4);node=by_object[go.object_reader.path_id]
        while node:
            matrix=transform_matrix(node)@matrix;node=transforms.get(node['m_Father']['m_PathID'])
        helper=MeshHandler(collider.m_Mesh.read());helper.process()
        local=np.asarray(helper.m_Vertices)
        unity=(np.c_[local,np.ones(len(local))]@matrix.T)[:,:3]
        triangles=np.array([t for submesh in helper.get_triangles() for t in submesh],dtype=np.int64)
        # This transform is independently checked against actual MGeo bounds,
        # full corridor coverage, stop-line locations and vehicle footprints.
        map_vertices=np.stack((-unity[:,0],-unity[:,2],unity[:,1]),axis=1)
        meshes.append((go.m_Name,map_vertices,triangles));expected.remove(go.m_Name)
    if expected:raise ValueError('missing reviewed pavement meshes: '+str(sorted(expected)))
    return meshes


def surface(meshes,path,end_station_m=860.):
    if len(path)<2 or not np.isfinite(path).all():
        raise ValueError('finite route with at least two points required')
    station=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1))]
    if end_station_m is not None:
        if not np.isfinite(end_station_m) or end_station_m<=0:
            raise ValueError('positive finite route endpoint required')
        path=path[station<end_station_m]
    tree=cKDTree(path[:,:2]);polygons=[];sources=[]
    for name,vertices,indices in meshes:
        triangle=vertices[indices];center=triangle.mean(axis=1)
        normal=np.cross(triangle[:,1]-triangle[:,0],triangle[:,2]-triangle[:,0])
        magnitude=np.linalg.norm(normal,axis=1);distance,index=tree.query(center[:,:2])
        keep=(distance<25)&(abs(center[:,2]-path[index,2])<.75)&(abs(normal[:,2])>=magnitude*.98)&(magnitude>1e-8)
        for tri in triangle[keep]:
            polygon=Polygon(tri[:,:2])
            if polygon.is_valid and polygon.area>1e-8:polygons.append(polygon)
        sources.append(dict(object=name,vertices=len(vertices),triangles=len(indices),selected=int(keep.sum())))
    pavement=unary_union(polygons)
    coverage=sum(pavement.covers(Point(*p[:2])) for p in path)
    if coverage!=len(path):raise ValueError(f'physical road has uncovered route samples: {coverage}/{len(path)}')
    areas=[]
    for polygon in (list(pavement.geoms) if hasattr(pavement,'geoms') else [pavement]):
        if polygon.area>1e-6:
            areas.append(dict(polygon=list(polygon.exterior.coords),holes=[list(r.coords) for r in polygon.interiors]))
    return dict(schema_version=1,source='active MORAI road MeshCollider triangles, read-only',
        coordinate_transform='Unity (x,y,z) -> local UTM (-x,-z,y)',
        surface_selection='within 25m reviewed route, height error <0.75m, slope normal >=0.98',
        route_end_station_m=end_station_m,
        map_id='R_KR_PR_K-city_2025',mesh_sources=sources,drivable_areas=areas,
        route_point_count=len(path),route_points_covered=coverage)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,required=True);p.add_argument('--global-path',type=Path,required=True)
    p.add_argument('--full-route',action='store_true',help='require coverage of the whole official route')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result=surface(extract(args.bundle),np.loadtxt(args.global_path)[:,:3],
                   end_station_m=None if args.full_route else 860.)
    result['bundle_sha256']=digest(args.bundle)
    result['global_path_sha256']=digest(args.global_path)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result))
    print(json.dumps({k:v for k,v in result.items() if k not in ('drivable_areas','mesh_sources')}))


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


if __name__=='__main__':main()
