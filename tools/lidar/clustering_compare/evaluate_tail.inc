        const auto off_result = L1B4Harness::Recluster(pipeline, reference, reference.labels, protect);
        result["candidate_off"] = Serialize(off_result, protect, ambiguous, available, protected_boxes, protection_ms, 0, 0);
        Cloud::Ptr raw_level(new Cloud);
        ApplyRotation(*raw, reference.leveling, raw_level.get());
        std::vector<std::uint8_t> raw_labels(raw->size(), 255), raw_protect(raw->size(), 0), raw_ambiguous(raw->size(), 0);
        for (std::size_t i = 0; i < raw->size(); ++i) if (lineage.raw_to_voxel[i] >= 0) {
            const auto vi = lineage.raw_to_voxel[i]; raw_labels[i] = reference.labels[vi];
            raw_protect[i] = protect[vi]; raw_ambiguous[i] = ambiguous[vi];
        }
        result["raw_path"] = result["input_id"].as<std::string>() + ".raw.bin";
        WriteCloud(*raw_level, clouds / result["raw_path"].as<std::string>());
        result["raw_leveled_xyzi_fnv1a64"] = CloudHash(*raw_level);
        auto add_mode = [&](const char* name, const l1b5::Partition& partition) {
            const auto finished = L1B5Harness::Finish(pipeline, reference, raw_level, raw_labels, partition.clusters, raw_protect);
            auto m = Serialize(finished, raw_protect, raw_ambiguous, available, protected_boxes, protection_ms, 0, 0);
            m["status"] = "evaluated";
            m["projection_reasons"] = Sequence(partition.reasons);
            m["pre_gate_cluster_id"] = Sequence(partition.pre_gate_cluster_id);
            m["rejected_unique_voxel_sizes"] = Sequence(partition.rejected_gate_sizes);
            m["projection_row"] = Sequence(partition.projection_row);
            m["projection_col"] = Sequence(partition.projection_col);
            m["projection_input_points"] = partition.input_points;
            m["projection_kept_points"] = partition.projected_points;
            m["timing_ms"]["projection_adapter"] = partition.projection_ms;
            m["timing_ms"]["linkage_and_core_labels"] = partition.linkage_ms;
            m["timing_ms"]["cluster"] = partition.linkage_ms;
            m["timing_ms"].remove("reference_pipeline");
            m["timing_ms"].remove("pipeline_estimate");
            m["timing_ms"]["unique_voxel_gate"] = partition.observation_gate_ms;
            m["timing_ms"]["clustering_and_obb"] = partition.projection_ms + partition.linkage_ms +
                partition.observation_gate_ms + finished.timings.obb_ms + finished.timings.obb_filter_ms;
            result[name] = m;
        };
        add_mode("raw_euclidean", l1b5::Euclidean(raw_level, raw_labels, lineage.raw_to_voxel, aos_params));
        if (row["projection_admitted"].as<bool>(false))
            add_mode("travel_aos", l1b5::Aos(*raw, raw_labels, lineage.raw_to_voxel, aos_params));
        else {
            result["travel_aos"]["status"] = "blocked_projection_admission";
            result["travel_aos"]["reason"] = "input_projection_contract_unverified";
        }
        Json(out, result); out << '\n';
    }
    return 0;
} catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
