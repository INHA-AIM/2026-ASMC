"""Independent MORAI pose/aspect evaluation, never a policy input.

The reviewed drivable surface is checked against actual full vehicle boxes and
their sampled sweep, independently of the planner's road check. Transition
order is bounded by RPC observations; ambiguous crossings are not called legal.
"""
import math
from shapely.geometry import Polygon,MultiPoint,LineString,Point
from shapely.ops import unary_union
from shapely.prepared import prep


def footprint(x,y,yaw,front,rear,half_width):
    c,s=math.cos(yaw),math.sin(yaw)
    return Polygon([(x+c*a-s*b,y+s*a+c*b) for a,b in
                    ((front,half_width),(front,-half_width),(-rear,-half_width),(-rear,half_width))])


def box_projection(center,length,width,height,roll=0.,pitch=0.):
    x,y,yaw=center;c,s=math.cos(yaw),math.sin(yaw)
    cr,sr,cp,sp=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
    vertices=[]
    for a in (-length,length):
        for b in (-width,width):
            for z in (-height,height):
                forward=cp*a+sp*sr*b+sp*cr*z;left=cr*b-sr*z
                vertices.append((x+c*forward-s*left,y+s*forward+c*left))
    return MultiPoint(vertices).convex_hull


def red_recovery_exit_proof(binding, actual, policy):
    """Permit test-phase restoration only AFTER independently verified exit.

    This milestone is evaluation/control-of-stimulus only, never policy input.
    It does not replace the trial endpoint, road/contact or observation checks.
    """
    if policy.get('motion')!='EXITED' or not policy.get('actual_control_allowed') or \
       policy.get('stop_line')!=binding['stop_node_id'] or policy.get('severity')!=0 or \
       any(policy.get('causes',{}).get(c,2)!=0 for c in
           ('SIGNAL','COLLISION','PEDESTRIAN','INPUT_INVALID','EXECUTION_FAILURE')) or \
       actual.get('aspects',{}).get(binding['traffic_light_id'])!=1 or \
       not actual.get('road_ok') or actual.get('collisions') or not actual.get('observation_complete'):
        return None
    ex,ey=binding['exit_point'];nx,ny=binding['exit_direction'];b=actual['bounding_box']
    if not all(map(math.isfinite,(ex,ey,nx,ny))) or abs(math.hypot(nx,ny)-1)>1e-5:return None
    shape=box_projection(b['center'],b['half_length'],b['half_width'],b['half_height'],b['roll'],b['pitch'])
    clearance=min((x-ex)*nx+(y-ey)*ny for x,y in shape.exterior.coords)
    if clearance<=.5:return None
    return dict(actual_rear_body_exit_clearance_m=clearance,
                policy_route_generation=policy.get('route_generation'),
                policy_decision_ros=policy.get('decision_ros'),actual_aspect=1)


class Oracle:
    def __init__(self,drivable,bindings,coordinates=None,maximum_gap_s=None):
        polygons=[Polygon([p[:2] for p in a['polygon']],a.get('holes',[])) for a in drivable['drivable_areas']]
        self.road=prep(unary_union([p if p.is_valid else p.buffer(0) for p in polygons]))
        self.coordinates=coordinates
        self.bindings=bindings;self.previous=None;self.crossings=[];self.violations=[]
        self.road_departures=[];self.collisions=[]
        self.maximum_gap=maximum_gap_s;self.observation_gaps=[]

    def observe(self,actor,now,aspects):
        t=actor.transform;bb=actor.bounding_box
        x,y,yaw=t.location.x,t.location.y,math.radians(t.rotation.z)
        offset=self.coordinates.yaw_offset(x,y) if self.coordinates else 0.
        yaw+=offset
        # ActorState.velocity is km/h in this installed MORAI build (also the
        # existing MoraiLink contract), unlike canonical ROS VehicleState m/s.
        speed_kmh=math.sqrt(sum(getattr(actor.velocity,k)**2 for k in ('x','y','z')))
        speed=speed_kmh/3.6
        center=(bb.location.x,bb.location.y,math.radians(bb.rotation.z)+offset)
        height=getattr(bb.extent,'z',0.)
        roll=math.radians(getattr(bb.rotation,'x',0.));pitch=math.radians(getattr(bb.rotation,'y',0.))
        if not all(map(math.isfinite,(*center,bb.extent.x,bb.extent.y,height,roll,pitch))) or min(bb.extent.x,bb.extent.y)<=0:
            raise ValueError('invalid physical vehicle bounding box')
        shape=box_projection(center,bb.extent.y,bb.extent.x,height,roll,pitch)
        road_ok=self.road.covers(shape)
        if self.previous:
            old=self.previous
            gap=now-old['at']
            if gap<=0 or (self.maximum_gap is not None and gap>self.maximum_gap):
                self.observation_gaps.append(dict(begin=old['at'],end=now,gap_s=gap))
            # Up to 0.1m translation or 1 degree between evaluated boxes.
            distance=math.hypot(center[0]-old['center'][0],center[1]-old['center'][1])
            angle=math.remainder(center[2]-old['center'][2],2*math.pi)
            count=max(1,math.ceil(distance/.1),math.ceil(abs(angle)/math.radians(1)))
            for i in range(1,count):
                q=i/count
                box=box_projection((old['center'][0]+q*(center[0]-old['center'][0]),
                    old['center'][1]+q*(center[1]-old['center'][1]),old['center'][2]+q*angle),
                    bb.extent.y,bb.extent.x,height,old['roll']+q*(roll-old['roll']),old['pitch']+q*(pitch-old['pitch']))
                road_ok=road_ok and self.road.covers(box)
        if not road_ok:self.road_departures.append(now)
        collisions=[k for k in actor.vehicle_state.collision_objects if k and k!='Ego']
        if collisions:self.collisions.append(dict(at=now,actors=collisions))
        fronts={};wheels={}
        for b in self.bindings:
            aid=b['traffic_light_id'];nx,ny=b['approach_direction'];lx,ly=b['line_point']
            # Earliest physical front-wheel crossing, using measured front track.
            wheels[aid]=[(x+3*math.cos(yaw)-side*.814*math.sin(yaw),
                          y+3*math.sin(yaw)+side*.814*math.cos(yaw)) for side in (-1,1)]
            distances=[-((wx-lx)*nx+(wy-ly)*ny) for wx,wy in wheels[aid]]
            fronts[aid]=min(distances)
            if self.previous and self.previous['fronts'][aid]>0>=fronts[aid]:
                # A full lap can cross the infinite mathematical plane far
                # from this signal. Require the actual wheel sweep to cross
                # its finite painted stop line whenever source geometry exists.
                points=b.get('source_stopline_points')
                if points:
                    line=LineString(points);crossed=False
                    for old_wheel,new_wheel,new_distance in zip(self.previous['wheels'][aid],wheels[aid],distances):
                        old_distance=-((old_wheel[0]-lx)*nx+(old_wheel[1]-ly)*ny)
                        if old_distance>0>=new_distance:
                            fraction=old_distance/(old_distance-new_distance)
                            crossing=Point(*(a+fraction*(z-a) for a,z in zip(old_wheel,new_wheel)))
                            crossed=crossed or line.distance(crossing)<=.02
                    if not crossed:continue
                aspect=aspects.get(aid);old_aspect=self.previous['aspects'].get(aid)
                legal=aspect==old_aspect==b['green_color']
                result=dict(light=aid,begin=self.previous['at'],end=now,
                    before_aspect=old_aspect,after_aspect=aspect,legal_confirmed=legal,
                    classification='green' if legal else 'red_or_yellow' if aspect==old_aspect and aspect in (1,4) else 'uncertain')
                self.crossings.append(result)
                if not legal:self.violations.append(result)
        self.previous=dict(at=now,center=center,roll=roll,pitch=pitch,fronts=fronts,wheels=wheels,aspects=dict(aspects))
        return dict(x=x,y=y,yaw=yaw,actor_enu_yaw_deg=t.rotation.z,enu_to_grid_offset=offset,
            speed=speed,speed_kmh=speed_kmh,
            actual_actuation=dict(brake=getattr(actor.vehicle_state,'brake',None),
                throttle=getattr(actor.vehicle_state,'throttle',None),steering=getattr(actor.vehicle_state,'steer',None)),
            reported_acceleration={k:getattr(getattr(actor,'acceleration',None),k,None) for k in ('x','y','z')},
            bounding_box=dict(center=list(center),half_length=bb.extent.y,half_width=bb.extent.x,
                half_height=height,roll=roll,pitch=pitch),
            outside_area_m2=shape.difference(self.road.context).area,
            road_ok=road_ok,collisions=collisions,fronts=fronts,observation_complete=not self.observation_gaps)

    def result(self):
        return dict(violation_free=not(self.violations or self.road_departures or self.collisions or self.observation_gaps),
            observation_gaps=self.observation_gaps,
            crossings=self.crossings,signal_violations_or_uncertain=self.violations,
            road_departure_samples=len(self.road_departures),collisions=self.collisions)
