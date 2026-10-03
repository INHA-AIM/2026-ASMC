"""GT-conditioned shape diagnostics, independent of any detector clustering."""
from collections import Counter
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

RADII = (.2, .4, .8)
CLASSES = {0: "unassigned_or_sentinel", 10: "etc", 85: "stop_line",
           86: "vehicle", 109: "standing_object", 118: "pedestrian",
           125: "vehicle", 127: "asphalt", 128: "road_sign",
           129: "sidewalk", 132: "traffic_sign", 135: "vehicle",
           136: "crosswalk", 144: "blue_lane", 145: "vehicle",
           153: "building", 155: "vehicle", 164: "obstacle", 165: "vehicle",
           170: "yellow_lane", 175: "vehicle", 178: "road_edge",
           185: "vehicle", 190: "traffic_light", 255: "white_lane"}
ACTORS = {"vehicle", "pedestrian", "obstacle"}


def proportions(mask):
    mask = np.asarray(mask)
    n = len(mask)
    kept = int(np.count_nonzero(mask == 2))
    ground = int(np.count_nonzero(mask == 1))
    unknown = n - kept - ground
    return dict(input_points=n, retained_points=kept, ground_points=ground,
                unclassified_points=unknown, retained_fraction=kept/n if n else None,
                complete_ground_loss=bool(n and ground == n),
                partial_retention=bool(0 < kept < n),
                no_known_retained_points=bool(n and kept == 0))


def regions(points, mask):
    """2 m sensor-origin leveled cubes; not physical instances or map cells."""
    cells = np.floor(np.asarray(points) / 2.).astype(np.int64)
    _, inverse, count = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    kept = np.bincount(inverse, weights=(np.asarray(mask) == 2), minlength=len(count))
    ground = np.bincount(inverse, weights=(np.asarray(mask) == 1), minlength=len(count))
    unknown = np.bincount(inverse, weights=(np.asarray(mask) == 3), minlength=len(count))
    eligible = count >= 10
    ratios = kept[eligible]/count[eligible]
    return dict(regions=int(len(count)), sparse_regions=int((~eligible).sum()),
                eligible_regions=int(eligible.sum()),
                partial_regions=int(np.sum((ratios > 0) & (ratios < 1))),
                entirely_removed_regions=int(np.sum(eligible & (ground == count))),
                no_known_retained_regions=int(np.sum(ratios == 0)),
                regions_with_unclassified_points=int(np.sum(eligible & (unknown > 0))),
                entirely_retained_regions=int(np.sum(ratios == 1)),
                retention_histogram=np.histogram(ratios, [0, .05, .25, .75, .95, 1.000001])[0].tolist())


def actor_shape(points, mask):
    out = proportions(mask)
    out["sparse"] = len(points) < 10
    z = np.asarray(points)[:, 2]
    lo, hi = float(z.min()), float(z.max())
    band = np.minimum(2, np.floor(3 * (z-lo)/max(hi-lo, 1e-9)).astype(int))
    out["observed_height_bands"] = {name: proportions(np.asarray(mask)[band == i])
                                  for i, name in enumerate(("lower", "middle", "upper"))}
    out["height_basis"] = "visible_leveled_points_not_full_object_box"
    out["input_extent_xyz"] = np.ptp(points, axis=0).tolist()
    selected = points[np.asarray(mask) == 2]
    out["retained_extent_xyz"] = np.ptp(selected, axis=0).tolist() if len(selected) else None
    out["range_m"] = float(np.linalg.norm(np.median(points, axis=0)))
    return out


def topology(points, masks):
    """Construct neighbors BEFORE filtering, reuse them for every method.

    Reports topology relative to observed sampling, never GT object detection.
    No min cluster size, radius fitting, candidate clustering or GT inference.
    """
    points = np.asarray(points)
    n = len(points)
    pairs = cKDTree(points).query_pairs(max(RADII), output_type="ndarray")
    distance = np.linalg.norm(points[pairs[:, 0]]-points[pairs[:, 1]], axis=1) if len(pairs) else np.empty(0)
    result = {name: {} for name in masks}
    for radius in RADII:
        edges = pairs[distance <= radius]
        a, b = edges.T if len(edges) else (np.empty(0, int), np.empty(0, int))
        matrix = coo_matrix((np.ones(2*len(edges), np.uint8),
                            (np.concatenate((a,b)), np.concatenate((b,a)))), shape=(n,n)).tocsr()
        original_n, original_id = connected_components(matrix, directed=False)
        degree = np.diff(matrix.indptr)
        for name, mask in masks.items():
            mask = np.asarray(mask)
            retained = mask == 2
            selected = np.flatnonzero(retained)
            sub = matrix[selected][:, selected]
            remain_n, remain_id = connected_components(sub, directed=False)
            sizes = np.bincount(remain_id, minlength=remain_n)
            split = 0
            if len(selected):
                links = np.unique(np.column_stack((original_id[selected], remain_id)), axis=0)
                pieces = np.bincount(links[:, 0], minlength=original_n)
                split = int(np.maximum(pieces-1, 0).sum())
            known_edges = np.isin(mask[a], (1,2)) & np.isin(mask[b], (1,2))
            transition = known_edges & (mask[a] != mask[b])
            new_isolated = int(np.sum((np.diff(sub.indptr) == 0) & (degree[selected] > 0)))
            result[name][str(radius)] = dict(
                input_components=int(original_n), input_isolated_points=int((degree == 0).sum()),
                retained_components=int(remain_n), retained_points=int(retained.sum()),
                retained_isolated_points=int((np.diff(sub.indptr) == 0).sum()),
                newly_isolated_points=new_isolated, added_split_parts=split,
                largest_retained_component_points=int(sizes.max()) if len(sizes) else 0,
                components_1_to_3_points=int(np.sum(sizes <= 3)),
                comparable_edges=int(known_edges.sum()), deletion_preservation_edges=int(transition.sum()),
                boundary_fraction=float(transition.sum()/known_edges.sum()) if known_edges.any() else None)
    return result


def frame_metrics(cloud, semantic, instance, masks, pitch):
    sizes = [len(cloud), len(semantic), len(instance), *(len(x) for x in masks.values())]
    if len(set(sizes)) != 1:
        raise ValueError("point/GT/mask length mismatch")
    roi = next(iter(masks.values())) != 0
    if any(not np.array_equal(mask != 0, roi) for mask in masks.values()):
        raise ValueError("method ROI mismatch")
    xyz = cloud[:, :3].astype(float).copy()
    c, s = np.cos(np.float32(pitch)), np.sin(np.float32(pitch))
    xyz[:, 0] = c*cloud[:, 0]+s*cloud[:, 2]
    xyz[:, 2] = -s*cloud[:, 0]+c*cloud[:, 2]
    result = {m: {"classes": {}, "objects": []} for m in masks}
    class_names = np.array([CLASSES.get(int(x), "unmapped_"+str(int(x))) for x in semantic])
    for name in sorted(set(class_names[roi])):
        selected = roi & (class_names == name)
        local_masks = {m: labels[selected] for m, labels in masks.items()}
        # Actor topology must use individual instances, not all cars at once.
        if name not in ACTORS:
            graphs = topology(xyz[selected], local_masks)
        for m in masks:
            entry = proportions(local_masks[m])
            if name not in ACTORS:
                entry["regions"] = regions(xyz[selected], local_masks[m])
                entry["topology"] = graphs[m]
            result[m]["classes"][name] = entry
        if name in ACTORS:
            for identity in np.unique(instance[selected]):
                if int(identity) == 0:
                    for m in masks:
                        result[m]["classes"][name]["unresolved_instance_points"] = int(np.sum(selected & (instance == 0)))
                    continue
                actor = selected & (instance == identity)
                actor_masks = {m: labels[actor] for m, labels in masks.items()}
                graphs = topology(xyz[actor], actor_masks)
                for m in masks:
                    obj = actor_shape(xyz[actor], actor_masks[m])
                    obj.update(semantic=name, instance_id=int(identity), topology=graphs[m])
                    result[m]["objects"].append(obj)
    return result
