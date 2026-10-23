"""Choose examples by timestamp discrepancy only, never prediction error."""
import argparse,json,math,bisect
from pathlib import Path
import numpy as np,pandas as pd
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag
from sensor_msgs import point_cloud2

def nearest(ts,t):
    k=bisect.bisect_left(ts,t)
    return min([i for i in [k-1,k] if 0<=i<len(ts)],key=lambda i:abs(ts[i]-t))
def corners(x,y,l,w,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return (np.array([[-1,-1],[-1,1],[1,1],[1,-1],[-1,-1]])*[l/2,w/2])@np.array([[c,s],[-s,c]])+[x,y]
def main():
    ap=argparse.ArgumentParser();ap.add_argument("--work",type=Path,required=True);ap.add_argument("--previous",type=Path,required=True);ap.add_argument("--bag",type=Path,required=True);a=ap.parse_args();W=a.work
    raw=json.loads((W/"source_gt_records.json").read_text());E=[r["ns"] for r in raw["ego"]];O=[r["ns"] for r in raw["snapshots"]]
    old=pd.read_csv(a.previous/"inputs/actual_lap/gt.csv");new=pd.read_csv(W/"gt_aligned_v1/actual_lap/gt.csv")
    new=new[(new.kind=="gt")&(new.class_id==0)&(new.x>0)&(np.hypot(new.x,new.y)<35)]
    common=new.merge(old[["idx","track_id"]],on=["idx","track_id"],validate="one_to_one")
    ranked=[]
    for f in raw["frames"]:
        ids=common[common.idx==f["idx"]]
        if not len(ids):continue
        t=f["stamp_ns"];de=E[nearest(E,t)]-t;do=O[nearest(O,t)]-t
        g=ids.iloc[np.argmin(np.hypot(ids.x,ids.y))]
        ranked.append({"idx":f["idx"],"gt_id":int(g.track_id),"ego_nearest_minus_lidar_ns":de,"object_nearest_minus_lidar_ns":do,"rank_abs_ns":abs(de)+abs(do)})
    chosen=[]
    for row in sorted(ranked,key=lambda r:r["rank_abs_ns"],reverse=True):
        if all(abs(row["idx"]-v["idx"])>180 for v in chosen):chosen.append(row)
        if len(chosen)==3:break
    pred={d["idx"]:d for d in map(json.loads,(a.previous/"runs/AB_ON_reader_v2/out.jsonl").open())}
    lookup={v["idx"]:v for v in chosen}
    with rosbag.Bag(str(a.bag)) as b:
        idx=-1
        for topic,m,_ in b.read_messages(topics=["/velodyne_points"]):
            idx+=1
            if idx not in lookup:continue
            row=lookup[idx];tid=row["gt_id"]
            before=old[(old.idx==idx)&(old.track_id==tid)].iloc[0];after=new[(new.idx==idx)&(new.track_id==tid)].iloc[0]
            P=np.array(list(point_cloud2.read_points(m,field_names=["x","y","z"],skip_nans=True)))+np.array([.8,0,1.45])
            fig,axes=plt.subplots(1,2,figsize=(12,5),sharex=True,sharey=True)
            for ax,gt,title,color in zip(axes,[before,after],["Previous nearest GT","Aligned GT at LiDAR header"],["#be3c20","#1266aa"]):
                ax.scatter(P[:,0],P[:,1],s=1,c="gray",alpha=.5)
                box=corners(gt.x,gt.y,gt.l,gt.w,gt.yaw);ax.plot(box[:,0],box[:,1],c=color,lw=2,label="GT "+str(tid))
                for obj in pred[idx]["local"]:
                    if np.hypot(obj["c"][0]-after.x,obj["c"][1]-after.y)<4:
                        q=corners(obj["c"][0],obj["c"][1],obj["l"],obj["w"],obj["yaw"]);ax.plot(q[:,0],q[:,1],c="#23843d",ls="--",alpha=.8,label="AB pred "+str(obj["id"]))
                ax.set(xlim=(after.x-5,after.x+5),ylim=(after.y-5,after.y+5),title=title,xlabel="body x (m)")
                ax.grid(alpha=.25);ax.set_aspect("equal");ax.legend(fontsize=8)
            axes[0].set_ylabel("body y (m)")
            shift=float(np.hypot(after.x-before.x,after.y-before.y))
            fig.suptitle("Scan %d | Ego nearest offset %.2f ms, Object %.2f ms | GT center shift %.3f m"%(idx,row["ego_nearest_minus_lidar_ns"]/1e6,row["object_nearest_minus_lidar_ns"]/1e6,shift))
            fig.tight_layout();path=W/"results"/("ALIGNMENT_OVERLAY_%d.png"%idx);fig.savefig(path,dpi=150);plt.close(fig)
            row.update(center_shift_m=shift,figure=path.name,object_source_a_ns=int(after.source_a_ns),object_source_b_ns=int(after.source_b_ns))
    (W/"results/OVERLAY_SELECTION.json").write_text(json.dumps(chosen,indent=2));print(json.dumps(chosen,indent=2))
if __name__=="__main__":main()
