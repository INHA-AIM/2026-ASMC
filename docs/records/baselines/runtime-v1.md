# P1-A Runtime Baseline v1

> **상태:** 동결된 회귀 기준 · 생성 도구: `tools/audit/build_runtime_baseline.py`
> **주의:** 이 문서는 센서셋·네트워크·패키지 경로·HD맵 geometry를 변경하지 않는다.

## 1. 요약

- catkin package: **24개**
- launch file/node 선언: **54개 / 61개**
- local runtime message MD5: **33개**
- HD Map: `kcity2025_route_curation_stage14_20260918` · geometry SHA-256 `01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf`
- HD Map inventory: lane marking **121개**, stopline **7개**
- 공식 checkpoint: **16개**, 반경 **3.0 m**
- 전역경로: **2184.61 m**, 출발 5% station **109.23 m**
- P1-C metadata: checkpoint **16개**, mission zone **15개** (아직 미발행)

현재 stage14 map은 검수 기준이지만 `partial_candidate=true`, `operational=false`다. 이 상태값을
geometry 회귀 기준과 별개로 보존한다.

## 2. HD Map 회귀 기준

| 항목 | 값 |
|---|---|
| Map / build | `kcity2025_competition_route_curation_candidate_v14` / `kcity2025_route_curation_stage14_20260918` |
| Source file SHA-256 | `ffcccb478ab9fea5232631dab49145eb59c9211a88c6d2f460108355fc4c54ee` |
| Geometry-only SHA-256 | `01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf` |
| Lane marking | 121 |
| White / yellow | 94 / 27 |
| Solid / broken | 90 / 31 |
| Stopline | 7 |

## 3. 센서 기준과 승인된 후속 변경

| ID | 센서 | 현재 Hz | UDP port | 위치 (m) |
|---|---|---|---|---|
| 1 | camera_1 | 20.000 | 9290 → 9291 | (1.90, 0.00, 1.20) |
| 2 | camera_2 | 20.000 | 9292 → 9293 | (1.15, 0.65, 1.20) |
| 3 | camera_3 | 20.000 | 9294 → 9295 | (1.15, -0.65, 1.20) |
| 4 | camera_4 | 10.000 | 9296 → 9297 | (0.00, 0.00, 1.40) |
| 5 | lidar3d_1 | 10.000 | 9298 → 9299 | (0.80, 0.00, 1.45) |
| 6 | gps_1 | 5.000 | 9300 → 9301 | (0.35, 0.00, 1.25) |
| 7 | imu_1 | 50.000 | 9302 → 9303 | (0.00, 0.00, 0.00) |

LiDAR는 현재 **10 Hz**가 회귀 기준이다. 손재호 담당 코드의
요구에 따른 **15 Hz** 변경은 3단계의 승인된 요구지만 아직 적용하지 않았다.
MORAI UI에서 15 Hz로 설정·재저장하여 `sensorPeriod`와 `frequencyIndex`를 확인한 뒤
30초 이상 `/velodyne_points` 실측과 tracking/resource 비교를 통과해야 한다.

현장 제출 `Autonav_sensor.json`은 provenance 전용이다. v2와의 semantic 차이는
자유 카메라 pitch(`328°` 대 제출본 `340°`) 하나뿐이며 실행 정본은 v2다.

## 4. 대표 bag 토픽

### raw

`$ASMC_DATA/rosbags/raw/20260913_195606_kcity2025_shadow_static_avoid_return_sunny_1pm_wheel/bags/20260913_195606_kcity2025_shadow_static_avoid_return_sunny_1pm_wheel_0.bag` · 28.260 s · SHA-256 `a2f1b41be24ecd4eaf79eeadc6c5a32c2ae126f9b68f349e06478fa242559ebc`

| 토픽 | 타입 | MD5 | 메시지 | 평균 Hz |
|---|---|---|---|---|
| `/Ego_topic` | `morai_msgs/EgoVehicleStatus` | `a54480c8f4c41293a77cf7a0eb3925c9` | 1378 | 48.762 |
| `/cam_left/image_jpeg/compressed` | `sensor_msgs/CompressedImage` | `8f7a12909da2c9d3332d540a0977563f` | 564 | 19.958 |
| `/cam_right/image_jpeg/compressed` | `sensor_msgs/CompressedImage` | `8f7a12909da2c9d3332d540a0977563f` | 563 | 19.922 |
| `/gps` | `morai_msgs/GPSMessage` | `6ab9511aaf9d33b721630ddffc1f7c5f` | 142 | 5.025 |
| `/image_jpeg/compressed` | `sensor_msgs/CompressedImage` | `8f7a12909da2c9d3332d540a0977563f` | 563 | 19.922 |
| `/imu` | `sensor_msgs/Imu` | `6a62c6daae103f4ff57a132d6f95cec2` | 1396 | 49.399 |
| `/morai_sensor_bridge/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `60810da900de1dd6ddd437c3503511da` | 6 | 0.212 |
| `/morai_udp_bridge/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `60810da900de1dd6ddd437c3503511da` | 6 | 0.212 |
| `/tf_static` | `tf2_msgs/TFMessage` | `94810edda583a504dfda3829e70d7eec` | 1 | 0.035 |

### result

`$ASMC_DATA/rosbags/results/20260913_195606_kcity2025_shadow_static_avoid_return_sunny_1pm_wheel/twinlite_v6_shadow_boundary_family_candidate_v3_profile_v1_20260915/bags/twinlite_v6_shadow_boundary_family_candidate_v3_profile_v1_20260915_0.bag` · 28.230 s · SHA-256 `7ab5c78147651c5bcdd77272589988309d3ba653237729e09ab6ed253872ded8`

| 토픽 | 타입 | MD5 | 메시지 | 평균 Hz |
|---|---|---|---|---|
| `/perception/bev/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `60810da900de1dd6ddd437c3503511da` | 16 | 0.567 |
| `/perception/camera/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `60810da900de1dd6ddd437c3503511da` | 14 | 0.496 |
| `/perception/lane/boundary_family` | `asmc_msgs/LaneBoundaryFamily` | `70ac79f98ca1244c782ea4067becd6af` | 293 | 10.379 |
| `/perception/lane/boundary_observations` | `asmc_msgs/LaneBoundaryObservationArray` | `f4c14183f7aa8631b2893327df0651f1` | 293 | 10.379 |
| `/perception/lane/debug/shared_family/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `60810da900de1dd6ddd437c3503511da` | 293 | 10.379 |
| `/perception/lane/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `60810da900de1dd6ddd437c3503511da` | 16 | 0.567 |
| `/perception/lane/pipeline_audit` | `diagnostic_msgs/DiagnosticArray` | `60810da900de1dd6ddd437c3503511da` | 293 | 10.379 |

## 5. 팀 메시지 MD5

| 타입 | ROS MD5 | 소스 SHA-256 |
|---|---|---|
| `Msg/TrackedObject` | `6c96135eb03a3ee93bba474fa835f57d` | `c1cba43089d4909873650f92cde14d86964376449559f1c76b7e7fb397b5265a` |
| `Msg/TrackedObjectArray` | `e127b2aabd7494ad01a5a56b005bf07e` | `e3ae24a4bc2f200044be54f8f219504fa2e721f58b846c188b4ce38ed111251a` |
| `Msg/Yolo_bbox` | `605f5dd6f8f921f2dc15199c760840f9` | `e18df2183cb2d19ebfcf5e03fd01aec48a93fe83f156197460b8f848eb1bf4b8` |
| `Msg/Yolo_bbox_arr` | `9604a9d8104d38c3bd9830382a3571d0` | `24f263d3ccbb64879554db75104f6e1283f12e875478a48982af60c39371d214` |
| `asmc_msgs/BehaviorCommand` | `fad196232c8f54cbbe2a1bced9d3972a` | `4d1eae77eceeb23ce4e6453340e57a7e44d5b96251af88de391c9ad64e493d5b` |
| `asmc_msgs/BehaviorContext` | `450f8f3ab65064fbb3622550828cadc5` | `364e6bd6c949d9eb30200c47f5439c173cb837dd214058c36cb1359f6790a576` |
| `asmc_msgs/CompetitionCheckpoint` | `794b79e3d48ac7fef6493abadad2fc47` | `0d2a14ec11ff92106b461fb98d64d08b4958345b586f0f70ba8474d10244ee4b` |
| `asmc_msgs/CompetitionCheckpointArray` | `2a9f1f28734c005b499ce4993b20931e` | `528ed5154faa0b3624647c7eaafce4753ca883839909c500f75c92a01671125e` |
| `asmc_msgs/DetectedObject` | `a965285ae904ae3ade687b9c54c2c515` | `0137ff768f5cad6e5e2c3784dd6af818dcd4888a647b4f08a73a60a3d6ff630e` |
| `asmc_msgs/DetectedObjectArray` | `14fbd9679c2d87ae7498e4269d9678da` | `b9d4660cd540fb36ee9ac7673e5d25a64cab9416b031128922d145f277a5fcad` |
| `asmc_msgs/DrivableArea` | `3f988da146dba2b6bd2348b32ba3fb7d` | `6c2b8f838ac50d927e2489b5477989c9715226641d9ae609d68c68cf9175f8d7` |
| `asmc_msgs/DrivableAreaArray` | `672c8c1123790ada26596de6aab5edcb` | `d6d143bab65d2f5b7598e21f011e8f2b35e19981e699144c546e8ed43b1c7f45` |
| `asmc_msgs/FrenetFeedback` | `072b15bb1dfe3f5b99d17c1487408114` | `e68970875d9120c1d845de237b8b9f488b8d6f20f11de54d02291061c51b400f` |
| `asmc_msgs/HdMapMeta` | `adb0d98c5dbd41055ebb929fcdf3af19` | `e8bf223ccd185df17e8c6c2601a9aa8d7913f9235f6590325b694ea489278e2c` |
| `asmc_msgs/Lane` | `380340d72bbc206438d6cf73629c5fa4` | `c842278e9a430499e723b050b7cb84119f2f3e8c5b74877795252fa55f8bc761` |
| `asmc_msgs/LaneArray` | `95ad93ef58c8ff22f4069b8ca2388395` | `a199edb1a688ef6f60c87348017fe5142da648f396164e006d77ce9d2f181ecd` |
| `asmc_msgs/LaneBoundaryFamily` | `70ac79f98ca1244c782ea4067becd6af` | `6bc3ca95dba989daa017db8cb57ca8061b9c73c5e273335fb40300c826d8611f` |
| `asmc_msgs/LaneBoundaryFamilyMember` | `516612ff4e656e2e61852056fda78969` | `18764087479b4a0da230c3cd749a01408092c9b19afeea25ccd404c7f0229d69` |
| `asmc_msgs/LaneBoundaryObservation` | `6ae63de01bc8e43ba30dbb4add2129c3` | `752dea18724055c250e69770c55b9141af77a950721312b790558d7748d26687` |
| `asmc_msgs/LaneBoundaryObservationArray` | `f4c14183f7aa8631b2893327df0651f1` | `e9b5f165668069858fab289f149fe7b17a40812a1dc0f02e0fcf0011db86b081` |
| `asmc_msgs/LaneMarking` | `65d4f9e4cbb4f60a65b5d7c3702e49fa` | `024758c33675a9a3954b6568bb28be719af29c55b41a9a0f6f94af885332fb5a` |
| `asmc_msgs/LaneMarkingArray` | `2eae16c9d4bf3385ac35eba4b3d1b823` | `c2ddbcd1379f10b04f79ccda07ddd20c3071acef11ec382a46d6cb35f15d10b0` |
| `asmc_msgs/MissionZone` | `902a8d5532787290e6471aa52a8a0d9c` | `d06e3ec39bd8c0292c22ce9b09e388ff7bf084d62fa5f8b552a63f01da8334e6` |
| `asmc_msgs/MissionZoneArray` | `04325427bb7aedffd9a6ec4881ed46ce` | `8c7f097982bf18eb465b509bd5e65811e5f71ae731e36a89f5931f6c629b77af` |
| `asmc_msgs/PlanFeedback` | `b5132603c037f3cf42de5a795df54d52` | `17278e09283ac563bf4880b0192d3dab3f20b816479f6e43572feb5969428c2e` |
| `asmc_msgs/Trajectory` | `38657ba74f761a20afb310b0265cb085` | `1fe76fb79468ba3579c18ccc96d33a971b40dd17704509e854625b554c43406d` |
| `asmc_msgs/TrajectoryPoint` | `f3382a560d156ee4a98b9951e48b3f3e` | `9bdce6e444790723cf69f664a698cde1a8b848fb917a92e58d3e0ce11812ac04` |
| `asmc_msgs/VehicleState` | `7b5b81807cda33ae90638895dcf749f1` | `8167c328acb6cdc21270d4fceda53f90686dfc27c1540217ff6d6c6204cb451f` |
| `behavior_planner/BehaviorContext` | `da76030d0c0b268889a850bc8bcce5dd` | `705253df4131921d26bbf7b6d9f0007fff6d8b208cd4bd5d1dfd42a8f6c5ba95` |
| `behavior_planner/PlanFeedback` | `6f4a10e67e2ec54e46e3797b766681f5` | `3740414b11fe11f2d018dcadf85a32f54ce90b4abeba7a31d1c5d0760cb04d4d` |
| `behavior_planner/UrbanFeatureDebug` | `01f6813b8f3711370df9eb54aa2b495c` | `a5eaf12dd7972d720c15220e063fb267f6bdfcdb925fefa623ee67657e91eac9` |
| `velodyne_msgs/VelodynePacket` | `ae4f90a23256f44e82baa08dd45c3456` | `87a00a34f67cb7320d810736dc10b575c63db73dd25a4c46a0dccd0f41916ee0` |
| `velodyne_msgs/VelodyneScan` | `50804fc9533a0e579e6322c04ae70566` | `2ae7c1ca2f72bb060be7bdbd65ec5cd781529ca4be44e2593a522470aa228a1a` |

## 6. 이후 단계에 넘기는 요구사항

| ID | 단계 | 상태 | 요구사항 |
|---|---|---|---|
| lidar_15hz | 3 | approved_not_applied | Change the Autonav VLP16 from the 10 Hz baseline to 15 Hz. |
| field_aligned_udp_ports | 3 | approved_not_applied | Use field ports in both environments: Ctrl 9093/9094 mode 2, Competition Vehicle Status 7603/7502, Collision 9091/9092, v2 sensor ports 9290-9303. |
| package_consolidation | 2-5 | planned | Converge team-owned runtime code on asmc_msgs, asmc_hd_map, asmc_bridge, and asmc_perception without moving planning-owned packages. |
| field_rehearsal | 6 | planned | Run native-Linux Docker LAN2LAN rehearsal on two hosts before deployment freeze. |

## 7. 재생성·검증

```bash
python3 tools/audit/build_runtime_baseline.py --write
python3 tools/audit/build_runtime_baseline.py --check
```

`--check`는 manifest와 보고서를 메모리에서 다시 생성해 커밋된 파일과 byte-for-byte 비교한다.
대표 bag과 현장 자료는 복사하지 않고 상대 경로·크기·SHA만 저장한다. 로컬 runtime message
MD5는 generated header를 우선 사용한다. 아직 build되지 않은 `asmc_msgs`만 P1-B에서 기존
generated MD5와 대조한 ROS1 source 계산기를 fallback으로 사용하며, 그 밖의 package는 clean
환경에서 먼저 build해야 한다.
