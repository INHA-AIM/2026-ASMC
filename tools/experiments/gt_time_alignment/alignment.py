"""Evaluation only: integer-ns GT interpolation; no ROS or inference dependencies."""
from bisect import bisect_left
from collections import Counter
import math
import numpy as np

def angle_delta(a,b):
    """Physical heading shortest arc, period 2pi, never OBB axis period pi."""
    return math.atan2(math.sin(b-a),math.cos(b-a))

def signature(s):
    return (tuple(s["p"]),s["heading"],tuple(s.get("dims",[])),s.get("cls"),
            tuple(s["offset"]) if s.get("offset") is not None else None,
            bool(s.get("conflict",False)))

def center(s):
    p=np.array(s["p"],dtype=float);o=s.get("offset") or (0.,0.)
    return p+np.array([o[0]*math.cos(s["heading"]),o[0]*math.sin(s["heading"]),o[1]])

def body_transform(ego,obj):
    """World box center at t -> body at same t; ground velocity is NOT ego-relative."""
    h=ego["heading"];c,s=math.cos(h),math.sin(h)
    R=np.array([[c,s],[-s,c]])
    p=R@(np.array(obj["center"][:2])-np.array(ego["p"][:2]))
    v=R@np.array(obj.get("velocity",[np.nan,np.nan])[:2])
    return {"x":p[0],"y":p[1],"z":obj["center"][2]-ego["p"][2],
            "yaw":angle_delta(h,obj["heading"]),"vx":v[0],"vy":v[1]}

class Series:
    def __init__(self,states,policy):
        self.policy=policy;self.audit=Counter();self.rows=[]
        self.nonmonotonic=any(b["ns"]<a["ns"] for a,b in zip(states,states[1:]))
        self.audit["nonmonotonic_edges"]=sum(b["ns"]<a["ns"] for a,b in zip(states,states[1:]))
        grouped={}
        for s in states:grouped.setdefault(int(s["ns"]),[]).append(s)
        for ns,group in sorted(grouped.items()):
            v=dict(group[0]);self.audit["duplicate_rows"]+=len(group)-1
            if len({signature(x) for x in group})>1:
                v["conflict"]=True;self.audit["conflict_timestamps"]+=1
            elif len(group)>1:self.audit["identical_duplicate_rows"]+=len(group)-1
            if not all(math.isfinite(x) for x in [*v["p"],v["heading"],*v.get("dims",[])]):
                v["invalid"]="nonfinite_state"
            self.rows.append(v)
        self.times=[s["ns"] for s in self.rows];self.edges=[];generation=0
        for i,s in enumerate(self.rows):
            if i:
                reason=self.edge_reason(self.rows[i-1],s)
                self.edges.append(reason)
                if reason:generation+=1;self.audit["break_"+reason]+=1
            s["generation"]=generation
    def edge_reason(self,a,b):
        if a.get("conflict") or b.get("conflict"):return "conflict_timestamp"
        if a.get("invalid") or b.get("invalid"):return "nonfinite_state"
        dt=b["ns"]-a["ns"]
        if dt<=0:return "nonpositive_interval"
        if dt>self.policy["max_gap_ns"]:return "long_gap"
        if "seq" in a and b["seq"]!=a["seq"]+1:return "object_absence"
        if a.get("cls")!=b.get("cls"):return "class_change"
        if len(a.get("dims",[]))!=len(b.get("dims",[])):return "dimension_change"
        if a.get("dims") and max(abs(x-y) for x,y in zip(a["dims"],b["dims"]))>self.policy["dimension_break_m"]:return "dimension_change"
        if a.get("offset")!=b.get("offset"):return "reference_change"
        jump=float(np.linalg.norm(np.array(b["p"])-a["p"]))
        if jump>self.policy["jump_slack_m"]+self.policy["max_reference_speed_mps"]*dt/1e9:return "position_jump"
        return None
    def query(self,t):
        t=int(t)
        if self.nonmonotonic:return None,"nonmonotonic_stream"
        if not self.rows:return None,"empty_stream"
        k=bisect_left(self.times,t);exact=k<len(self.times) and self.times[k]==t
        if exact:
            s=self.rows[k]
            if s.get("conflict"):return None,"conflict_timestamp"
            if s.get("invalid"):return None,s["invalid"]
            # At knots use valid right derivative, else valid left derivative.
            edge=k if k<len(self.edges) and self.edges[k] is None else k-1
            if edge<0 or edge>=len(self.edges) or self.edges[edge] is not None:edge=None
            out=dict(s);out["center"]=center(s).tolist();out["source_a_ns"]=out["source_b_ns"]=t;out["alpha"]=0.;out["mode"]="exact"
            out["velocity"]=[np.nan]*3;out["velocity_reason"]="no_valid_adjacent_interval"
            if edge is not None:self.add_velocity(out,self.rows[edge],self.rows[edge+1])
            return out,None
        if k==0:return None,"before_range"
        if k==len(self.rows):return None,"after_range"
        reason=self.edges[k-1]
        if reason:return None,reason
        a,b=self.rows[k-1:k+1];u=(t-a["ns"])/(b["ns"]-a["ns"])
        out=dict(a);out["p"]=((1-u)*np.array(a["p"])+u*np.array(b["p"])).tolist()
        out["heading"]=a["heading"]+u*angle_delta(a["heading"],b["heading"])
        if a.get("dims"):out["dims"]=((1-u)*np.array(a["dims"])+u*np.array(b["dims"])).tolist()
        out["center"]=center(out).tolist();out.update(source_a_ns=a["ns"],source_b_ns=b["ns"],alpha=u,mode="interpolated")
        self.add_velocity(out,a,b);return out,None
    def add_velocity(self,out,a,b):
        out["velocity_a_ns"]=a["ns"];out["velocity_b_ns"]=b["ns"]
        dt=(b["ns"]-a["ns"])/1e9
        v=(np.array(b["p"])-a["p"])/dt
        omega=angle_delta(a["heading"],b["heading"])/dt
        ox=(out.get("offset") or (0.,0.))[0];h=out["heading"]
        out["velocity"]=(v+np.array([-ox*math.sin(h)*omega,ox*math.cos(h)*omega,0])).tolist()
        out["reference_interval_velocity"]=v.tolist()
        out["center_interval_average_velocity"]=((center(b)-center(a))/dt).tolist()
        out["angular_rate"]=omega;out["velocity_reason"]="valid_interpolation_model_derivative"
