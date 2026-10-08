#!/usr/bin/env python3
"""Compare materialized TensorRT boxes/masks to saved same-image PyTorch output."""
import argparse
from collections import Counter
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from asmc_camera.detection.batched_backend import BatchedSegmentationDetector
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.classes import load_class_groups
from seg_measurements import match_boxes


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True);p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--groups',type=Path,required=True)
    p.add_argument('--hood-config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();cfg=yaml.safe_load(a.config.read_text());cfg['conf_thres']=.1
    model=BatchedSegmentationDetector(**cfg);hood=HoodMask.load(True,a.hood_config)
    groups=load_class_groups(yaml.safe_load(a.groups.read_text())['class_groups'])
    rows=[json.loads(l) for l in (a.reference/'predictions.jsonl').read_text().splitlines()]
    selected=rows[::max(1,len(rows)//90)];records=[];total=Counter()
    for row in selected:
        image=cv2.imread(str(a.datasets/row['image']));image=hood.apply(image) if row['view']=='front' else image
        result=model.infer_images({row['view']:image},[row['view']])[row['view']]
        predicted=[dict(source_class_name=d.source_class_name,group=groups.group_of(d.source_class_name),confidence=d.confidence,xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in result.detections]
        # Source labels must agree too; group-only correspondence is insufficient here.
        def exact_group(d):return dict(d,group=d['source_class_name'])
        pairs=match_boxes([exact_group(d) for d in row['predictions']],[exact_group(d) for d in predicted])
        original_masks=[]
        if row['masks']:
            with np.load(a.reference/row['masks']) as data:original_masks=np.unpackbits(data['masks'],axis=2)[:,:,:data['width'].item()].astype(bool)
        comparisons=[]
        for r,c,box_iou in pairs:
            original=row['predictions'][r];current=predicted[c];mask_iou=None
            if len(original_masks):
                x,y=original_masks[r],result.masks[c];union=np.count_nonzero(x|y);mask_iou=float(np.count_nonzero(x&y)/union) if union else 1.
            comparisons.append(dict(label=current['source_class_name'],box_iou=box_iou,mask_iou=mask_iou,
                                    max_coordinate_delta=max(abs(x-y) for x,y in zip(original['xyxy'],current['xyxy'])),confidence_delta=current['confidence']-original['confidence']))
        total.update(reference=len(row['predictions']),engine=len(predicted),matched=len(pairs),reference_unmatched=len(row['predictions'])-len(pairs),engine_unmatched=len(predicted)-len(pairs))
        records.append(dict(image=row['image'],view=row['view'],pairs=comparisons,reference=row['predictions'],engine=predicted,timings=result.timings))
    a.output.write_text(json.dumps(dict(model=model.info,frames=len(records),totals=dict(total),records=records,
        interpretation='FP16 differences retained; no blanket confidence-boundary exemption or exact-equivalence claim'),indent=2)+'\n')


if __name__=='__main__':main()
