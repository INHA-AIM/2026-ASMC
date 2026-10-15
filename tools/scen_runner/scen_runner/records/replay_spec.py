"""Portable chosen values, separate from native IDs and random generators."""
import copy


def crossing_spec(plan):
    return dict(type='crossing', model=plan.models[0][1], group=plan.models[0][0],
                cfg=copy.deepcopy(plan.cfg), start=list(plan.start), end=list(plan.end),
                heading_deg=plan.heading_deg, move_distance=plan.move_distance,
                direction=plan.direction_label, timing=copy.deepcopy(plan.timing))


def static_spec(ob, x, y):
    return dict(type='static', model=ob.morai_model,
                pose=dict(x=float(x), y=float(y), z=float(ob.z), yaw=float(ob.to_dict()['yaw_deg']), roll=0., pitch=0.),
                object=ob.to_dict())


def vehicle_spec(model, target, plan, method, source=''):
    cfg=copy.deepcopy(plan.cfg)
    cfg['models']={model:cfg['models'][model]}
    return dict(type='vehicle', model=model, target_kmh=target, method=method,
                cfg=cfg, source=str(source), period_s=getattr(plan,'period',None),
                destination=list(plan.destination) if hasattr(plan,'destination') else None)
