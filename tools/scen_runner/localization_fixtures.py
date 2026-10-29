"""Portable deterministic fixtures; native actor truth never enters planning."""
import math

NAMES=('none','center','left','right','staggered','parked_vehicle','parked_truck','angled_vehicle','crossing_left','crossing_right','standing_person')

def fixture_definition(name,route,coordinates):
    if name not in NAMES or name=='none':raise ValueError('named nonempty fixture required')
    def pose(station,lateral=0):
        x,y,z,h=route.curve.pose(station)
        return dict(x=x-math.sin(h)*lateral,y=y+math.cos(h)*lateral,
                    z=z+.1,roll=0.,pitch=0.,yaw=coordinates.enu_yaw_degrees(x,y,h))
    def item(key,kind,model,target,definition):
        return dict(record=dict(key=key,kind=kind,model=model,target_kmh=target,
                    source='localization_fixture',definition=definition))
    baseline=[]
    if name in ('center','left','right','staggered'):
        placements={'center':[(1915.,0.)],'left':[(1915.,.65)],'right':[(1915.,-.65)],
                    'staggered':[(1905.,.5),(1930.,-.5)]}[name]
        baseline=[item('static_'+str(i),3,'WoodBox',0.,dict(type='static',model='WoodBox',pose=pose(s,d)))
                  for i,(s,d) in enumerate(placements)]
    elif name in ('crossing_left','crossing_right','standing_person'):
        lateral=3.2 if name=='crossing_left' else -3.2
        if name=='standing_person':lateral=0.
        a,b=pose(1910.,lateral),pose(1910.,-lateral)
        speed=0. if name=='standing_person' else 4.3
        heading=math.atan2(b['y']-a['y'],b['x']-a['x']) if lateral else route.curve.pose(1910.)[3]
        cfg=dict(walk_speed_kmh=speed,activation_distance_m=18.,spawn_height_offset_m=0.)
        definition=dict(type='crossing',group='pedestrian',model='Man1',cfg=cfg,
            start=[a[k] for k in ('x','y','z')],end=[b[k] for k in ('x','y','z')],
            heading_deg=coordinates.enu_yaw_degrees(a['x'],a['y'],heading),
            direction=name,timing=dict(trigger='native ego activation distance',activation_distance_m=18.))
        baseline=[item('pedestrian_0',2,'Man1',speed,definition)]
    else:
        model='OBJ_Hyundai_HD65' if name=='parked_truck' else 'OBJ_Kia_K5'
        p=pose(1910.)
        if name=='angled_vehicle':p['yaw']+=25.
        baseline=[item('static_vehicle_0',3,model,0.,dict(type='static',model=model,pose=p))]
    return dict(schema='localization.fixture.v1',name=name,baseline=baseline,events=[],
                conditions=dict(weather='Foggy',hour=13),truth_scope='native state is orchestration/evaluation only')
