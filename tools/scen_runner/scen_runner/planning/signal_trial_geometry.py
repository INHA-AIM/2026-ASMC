"""Derive trial geometry from the exact simulator map, with explicit checks.

The input is an operator-owned MGeo snapshot. No simulator assets or host
paths are written to tracked configuration. Live camera association and clock
verification remain separate gates, not consequences of this builder.
"""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path

from shapely.geometry import LineString, MultiPoint, Point, Polygon
from shapely.ops import unary_union

APPROACHES = (
    ('A1256W000682', 'C1256W000077', 'A2256W000215', 'IntTL9', 'straight'),
    ('A1256W000807', 'C1256W000091', 'A2256W000222', 'IntTL7', 'straight'),
    ('A1256W000208', 'C1256W000016', 'A2256W000599', 'IntTL3', 'straight'),
    ('A1256W000742', 'C1256W000052', 'A2256W000083', 'IntTL1', 'left'),
)
COLOR = {'R':1, 'Y':4, 'SG':16, 'LG':32, 'R_with_GLeft':33, 'G_with_GLeft':48}


def unit(dx, dy):
    length = math.hypot(dx, dy)
    if length < 1e-6:
        raise ValueError('degenerate map direction')
    return dx/length, dy/length


def clip(points, origin, normal, keep_after):
    """Clip a convex polygon to a physical entry/exit half-plane."""
    sign = 1. if keep_after else -1.
    def distance(p):
        return sign*((p[0]-origin[0])*normal[0]+(p[1]-origin[1])*normal[1])
    result=[]
    for a,b in zip(points,points[1:]+points[:1]):
        da,db=distance(a),distance(b)
        if da>=-1e-9:
            result.append(a)
        if (da<0)!=(db<0):
            t=da/(da-db)
            result.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
    if len(result)<3:
        raise ValueError('empty intersection clipping')
    return result


def build(snapshot, reviewed_nodes, camera_frame='camera_extra'):
    root=Path(snapshot)
    def read(name):
        return json.loads((root/name).read_text())
    lights={x['idx']:x for x in read('traffic_light_set.json')}
    links={x['idx']:x for x in read('link_set.json')}
    controls={x['idx']:x for x in read('intersection_controller_set.json')}
    schedules={x['idx']:x for x in read('intersection_controller_data.json')}
    paint=[(x,LineString([p[:2] for p in x['points']])) for x in read('lane_boundary_set.json')
           if 530 in x.get('lane_type',[])]
    nodes={x['idx']:x for x in reviewed_nodes}
    bindings=[]
    for node,aid,lid,cid,movement in APPROACHES:
        if lid not in lights[aid]['link_id_list']:
            raise ValueError('signal does not control the configured route link: '+aid)
        groups=controls[cid]['TL']
        group=next(i for i,ids in enumerate(groups) if aid in ids)
        anchor=Point(nodes[node]['point'][:2])
        line,shape=min(paint,key=lambda value:value[1].distance(anchor))
        if shape.distance(anchor)>.05:
            raise ValueError('reviewed stop node differs from actual painted stop line: '+node)
        point=shape.interpolate(shape.project(anchor));line_point=[point.x,point.y]
        pts=line['points'];dx,dy=pts[-1][0]-pts[0][0],pts[-1][1]-pts[0][1]
        normal=unit(-dy,dx)
        own=links[lid]['points'];direction=unit(own[1][0]-own[0][0],own[1][1]-own[0][1])
        if normal[0]*direction[0]+normal[1]*direction[1]<0:
            normal=(-normal[0],-normal[1])
        exit_point=own[-1][:2]
        exit_direction=unit(own[-1][0]-own[-2][0],own[-1][1]-own[-2][1])
        controlled={link for ids in groups for light in ids for link in lights[light]['link_id_list']}
        # The complete junction envelope is NOT proof of conflict occupancy:
        # it includes the queue/crosswalk approach after the physical line.
        # Use actual crossings/merges with other incoming movements instead.
        own_shape=LineString([p[:2] for p in own]);conflicts=[];conflicting_links=[]
        half_width=max(float(links[lid]['width_start']),float(links[lid]['width_end']))/2
        if not .5<half_width<3:raise ValueError('unreviewed route lane width')
        for key in sorted(controlled):
            other=links[key]
            if other['from_node_idx']==links[lid]['from_node_idx']:continue
            width=max(float(other['width_start']),float(other['width_end']))/2
            if not .5<width<3:raise ValueError('unreviewed conflicting lane width')
            shape=LineString([p[:2] for p in other['points']])
            overlap=own_shape.buffer(half_width,cap_style=2).intersection(shape.buffer(width,cap_style=2))
            if overlap.area>.25 and overlap.intersects(own_shape):
                conflicts.append(overlap);conflicting_links.append(key)
        if not conflicts:raise ValueError('no verified road conflict region: '+aid)
        polygon=unary_union(conflicts).convex_hull
        coordinates=list(polygon.exterior.coords)[:-1]
        coordinates=clip(coordinates,line_point,normal,True)
        coordinates=clip(coordinates,exit_point,exit_direction,False)
        polygon=Polygon(coordinates)
        if not polygon.is_valid or polygon.area<5 or polygon.area>15000:
            raise ValueError('invalid conflict region: '+aid)
        if not polygon.intersects(own_shape):raise ValueError('conflict region does not intersect movement')
        phases=schedules[cid]['TLState']
        allowed=('SG','G_with_GLeft') if movement=='straight' else ('R_with_GLeft','G_with_GLeft','LG')
        phase=next(i for i,p in enumerate(phases) if p['lightcolor'][group] in allowed)
        bindings.append(dict(reviewed=True,stop_node_id=node,movement=movement,camera_frame=camera_frame,
            line_point=line_point,approach_direction=list(normal),conflict_reviewed=True,
            conflict_polygon=[list(p) for p in coordinates],exit_point=exit_point,
            exit_direction=list(exit_direction),traffic_light_id=aid,controlled_link_id=lid,
            intersection_id=cid,green_phase=phase,green_color=COLOR[phases[phase]['lightcolor'][group]],
            phase_duration_s=phases[phase]['duration'],source_stopline_id=line['idx'],
            source_conflicting_link_ids=conflicting_links,
            conflict_definition='overlap of actual crossing movement lane corridors, clipped to entry/exit planes',
            source_stopline_points=[p[:2] for p in pts]))
    names=['lane_boundary_set.json','traffic_light_set.json','link_set.json',
           'intersection_controller_set.json','intersection_controller_data.json']
    return dict(schema_version=1,scope='start_to_before_roundabout',
        review='actual simulator map/route-link/paint/phase consistency checks; camera and clock checked separately',
        source_sha256={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in names},reviewed_bindings=bindings)
