#include "Frenet/Maneuver/Merge/MergeGapEvaluator.hpp"
#include "Frenet/Maneuver/Merge/HighwayMerge.hpp"
#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"
#include "Frenet/Maneuver/Merge/HighwayRearResponse.hpp"

#include "Frenet/Geometry/PolygonUtils.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

// s=0..total_s, x=s, y=0(직선 경로). conflict/checkpoint 판정은 s만
// 사용하므로 곡률 없는 직선으로 충분하다.
RefLine MakeStraightRefLine(double total_s, double step = 1.0) {
    RefLine ref;
    ref.length=total_s;
    for (double s = 0.0; s <= total_s + 1e-9; s += step) {
        RefPoint p{};
        p.x = s;
        p.y = 0.0;
        p.s = s;
        p.theta = 0.0;
        p.kappa = 0.0;
        p.d_kappa = 0.0;
        ref.points.push_back(p);
    }
    return ref;
}

// 실제 후보 평가가 소비하는 authoritative 8-corner OBB를 직접 구성한다.
// 충돌 재검사는 geometry_valid인 corners를 요구하므로,
// production 합성기 없이 이 fixture가 직접 회전된 직사각형을 두 z 층에
// 채워 넣는다(실제 perception obb_corners 경로와 동일한 authoritative
// geometry 형태).
TrackedObject MakeVehicleAt(int id, double x, double y, double heading,
                            double speed) {
    TrackedObject obj;
    obj.id = id;
    obj.type = 1;
    obj.x = x;
    obj.y = y;
    obj.z = 0.0;
    obj.heading = heading;
    obj.speed = speed;
    obj.vx = speed * std::cos(heading);
    obj.vy = speed * std::sin(heading);
    obj.width = 1.9;
    obj.length = 4.5;
    obj.height = 1.6;
    obj.yaw_rate = 0.0;

    const double half_length = obj.length * 0.5;
    const double half_width = obj.width * 0.5;
    const double cosine = std::cos(heading);
    const double sine = std::sin(heading);
    const double local_xy[4][2] = {
        {-half_length, -half_width},
        { half_length, -half_width},
        { half_length,  half_width},
        {-half_length,  half_width},
    };
    for (std::size_t i = 0; i < 4; ++i) {
        const double corner_x = x + cosine * local_xy[i][0] - sine * local_xy[i][1];
        const double corner_y = y + sine * local_xy[i][0] + cosine * local_xy[i][1];
        obj.corners[i] = {corner_x, corner_y, 0.0};
        obj.corners[i + 4] = {corner_x, corner_y, obj.height};
    }
    obj.geometry_valid = geometry::ValidateCorners8(obj.corners).valid;
    obj.z_valid = true;
    return obj;
}

MergeConfig MakeRoundaboutCfg(double conflict_s) {
    MergeConfig cfg;
    cfg.conflict_s = conflict_s;
    cfg.approach_distance = 45.0;
    cfg.stop_buffer = 3.0;
    cfg.conflict_radius = 6.0;
    cfg.min_object_speed = 0.5;
    cfg.min_front_time_gap = 1.5;
    cfg.min_rear_time_gap = 2.0;
    cfg.ego_clear_time = 1.5;
    cfg.max_wait_time = 10.0;
    cfg.completion_distance = 8.0;
    cfg.commit_distance = 15.0;
    cfg.safe_confirm_cycles = 3;
    cfg.cross_speed_floor = 3.0;
    cfg.prediction_dt = 0.05;
    cfg.entry_search_dt = 0.1;
    cfg.min_travel_speed = 0.5;
    cfg.min_travel_accel = 0.1;
    return cfg;
}

HighwayMergeConfig MakeHighwayCfg() {
    HighwayMergeConfig cfg;
    cfg.enabled = true;
    cfg.approach_distance = 50.0;
    cfg.stop_buffer = 4.0;
    cfg.target_corridor_half_width = 5.5;
    cfg.object_search_distance = 100.0;
    cfg.min_heading_alignment = 0.5;
    cfg.min_front_gap = 8.0;
    cfg.min_rear_gap = 10.0;
    cfg.min_front_ttc = 2.0;
    cfg.min_rear_ttc = 3.0;
    cfg.max_wait_time = 10.0;
    cfg.commit_distance = 20.0;
    cfg.safe_confirm_cycles = 3;
    cfg.cross_speed_floor = 4.0;
    cfg.entry_search_dt = 0.1;
    cfg.collision_check_dt = 0.1;
    cfg.min_travel_speed = 0.5;
    cfg.min_travel_accel = 0.1;
    return cfg;
}

VehicleShape MakeEgoShape() {
    VehicleShape shape;
    shape.width = 1.9;
    shape.length = 4.5;
    shape.rear_overhang = -1.0;
    return shape;
}

CollisionCheckConfig MakeCollisionCfg() {
    CollisionCheckConfig cfg;
    cfg.safety_margin = 0.3;
    cfg.margin_growth_rate = 0.0;
    cfg.reactive_lookahead = 3.0;
    return cfg;
}

}  // namespace

// TEST 1 - 회전교차로 진입 경로에 다른 차량이 없으면 안전하다.
void TestRoundaboutGapSafeWhenNoTraffic() {
    const double conflict_s = 50.0;
    RefLine ref = MakeStraightRefLine(120.0);
    const MergeConfig cfg = MakeRoundaboutCfg(conflict_s);

    FrenetState ego{};
    ego.s = 40.0;
    ego.s_d = 5.0;

    const RoundaboutGap gap = FindRoundaboutGap(
        ref, ego, /*obstacles=*/{}, cfg, /*desired_speed=*/5.0,
        /*max_ego_accel=*/2.0);

    std::cout << "  safe=" << gap.safe << " confirmed=" << gap.confirmed
              << " entry_time=" << gap.entry_time << '\n';
    Expect(gap.confirmed, "TEST1: 빈 회전교차로에서는 gap 판단이 confirmed다");
    Expect(gap.safe, "TEST1: 빈 회전교차로에서는 gap이 안전하다");
}

// TEST 2 - conflict point를 막 통과하는 차량이 있으면 front gap이 부족해
// 그 시각 이후로 entry_time이 밀린다.
void TestRoundaboutGapBlockedByPrecedingVehicle() {
    const double conflict_s = 50.0;
    RefLine ref = MakeStraightRefLine(120.0);
    MergeConfig cfg = MakeRoundaboutCfg(conflict_s);
    cfg.min_front_time_gap = 3.0;  // 넉넉한 여유를 요구해 위반을 명확히 함

    FrenetState ego{};
    ego.s = 45.0;
    ego.s_d = 3.0;

    // 순환차가 conflict point(50, 0) 바로 위(50, 20)에서 -y 방향으로 곧장
    // 접근 중 - 궤적이 conflict point를 정확히 통과한다.
    TrackedObject circulating =
        MakeVehicleAt(/*id=*/7, /*x=*/50.0, /*y=*/20.0, /*heading=*/-M_PI / 2.0,
                     /*speed=*/8.0);
    // OBB yaw가 순간적으로 잘못되어도 map-frame velocity는 conflict point를
    // 향한다. motion prediction이 box orientation을 쓰면 이 차량을 놓친다.
    circulating.heading = 0.0;

    const RoundaboutGap gap = FindRoundaboutGap(
        ref, ego, {circulating}, cfg, /*desired_speed=*/5.0,
        /*max_ego_accel=*/2.0);

    std::cout << "  safe=" << gap.safe << " confirmed=" << gap.confirmed
              << " entry_time=" << gap.entry_time
              << " preceding_id=" << gap.preceding_id << '\n';
    Expect(gap.crossing_vehicle_count >= 1,
           "TEST2: 순환차가 conflict point 통과 차량으로 관측된다");
    // front gap 여유가 부족한 즉시 진입(가장 이른 entry)은 안전하지 않아야
    // 하며, 함수는 그 다음으로 안전한 entry_time을 계속 탐색한다 - 최소한
    // safe 여부가 관측된 차량 존재를 반영해야 한다(무차별 안전 판정 금지).
    Expect(gap.preceding_id == 7 || gap.following_id == 7,
           "TEST2: 관측된 순환차가 preceding 또는 following으로 분류된다");
}

// TEST 3 - ego가 conflict point에서 너무 멀면 confirmed=false다(안전하다는
// 뜻이 아니라 아직 판단할 수 없다는 뜻).
void TestRoundaboutGapUnconfirmedWhenEgoFarAway() {
    const double conflict_s = 500.0;
    RefLine ref = MakeStraightRefLine(600.0);
    MergeConfig cfg = MakeRoundaboutCfg(conflict_s);
    cfg.max_wait_time = 5.0;  // 짧게 잡아 "예측창 밖"을 쉽게 재현

    FrenetState ego{};
    ego.s = 0.0;
    ego.s_d = 0.5;  // 매우 느려 도착까지 오래 걸림

    const RoundaboutGap gap = FindRoundaboutGap(
        ref, ego, /*obstacles=*/{}, cfg, /*desired_speed=*/5.0,
        /*max_ego_accel=*/1.0);

    std::cout << "  confirmed=" << gap.confirmed
              << " entry_time=" << gap.entry_time << '\n';
    Expect(!gap.confirmed,
           "TEST3: ego 도착시각이 예측창 밖이면 confirmed=false다");
}

// The production highway evaluator now consumes the actual candidate. Keep
// the original empty/front-gap/disabled-policy intent and add rear/side cases.
struct HighwayFixture {
    RefLine ref=MakeStraightRefLine(400);
    HighwayMergeConfig cfg=MakeHighwayCfg();
    HighwayMergeRoute::Zone zone;
    std::shared_ptr<const asmc_planner::lane_reference::HdLaneMap> map;
    CartesianPath path;
    HighwayFixture() {
        using namespace asmc_planner::lane_reference;
        LaneMapData data;data.identity={"map","test","test",std::string(64,'a')};data.complete=true;
        LaneData lane;lane.id=lane.link_id="target";lane.traversable=true;
        lane.centerline={{0,0},{400,0}};lane.polygon={{0,-1.75},{400,-1.75},{400,1.75},{0,1.75}};
        data.lanes={lane};data.areas={{"road",{{0,-5},{400,-5},{400,5},{0,5}},true}};
        std::string error;map=HdLaneMap::create(data,error);
        zone.target_region=map->corridor({"target"},error);zone.target_reference=ref;
        zone.config.start_s=60;zone.config.completion_s=120;
        zone.config.checkpoints={{100,120}};zone.entry_s=95;zone.source_stop_s=91;
        cfg.speed_error_mps=0;cfg.post_merge_horizon_s=2;
        for(int i=0;i<=100;++i) {
            path.x.push_back(70+.8*i);path.y.push_back(0);path.yaw.push_back(0);
            path.kappa.push_back(0);path.v.push_back(8);path.a.push_back(0);
        }
    }
    HighwayCandidateCheck evaluate(const std::vector<TrackedObject>& objects) {
        return EvaluateHighwayMergeCandidate(path,zone,ref,objects,cfg,
            MakeEgoShape(),MakeCollisionCfg(),.1,true);
    }
};
void TestHighwayActualCandidate() {
    HighwayFixture f;
    Expect(f.map->laneHeading(100,0,8,0).value_or(1)==0,
        "actor axes use the containing lane's forward centerline direction");
    Expect(!f.map->laneHeading(100,4,8,0)&&!f.map->laneHeading(100,0,-8,0),
        "off-lane or opposite-direction actors do not inherit a target-lane direction");
    auto check=f.evaluate({});
    Expect(check.safe&&check.covers_completion,"TEST4: actual empty-road candidate covers completion and follow-on");
    Expect(check.clear_time_s>6&&check.clear_time_s<7,"completion time comes from the actual 8m/s path");
    auto leader=MakeVehicleAt(9,112,0,0,.3);
    Expect(!f.evaluate({leader}).safe,"TEST5: close leader blocks actual candidate");
    Expect(f.evaluate({MakeVehicleAt(9,200,0,0,8)}).safe,"TEST5: distant leader does not block");
    Expect(!f.evaluate({MakeVehicleAt(10,25,0,0,20)}).safe,"fast rear traffic cannot be ignored");
    Expect(!f.evaluate({MakeVehicleAt(11,75,0,0,8)}).safe,"side-by-side longitudinal overlap is unsafe");
    auto old=f.path;f.path.x.resize(25);f.path.y.resize(25);f.path.yaw.resize(25);
    f.path.kappa.resize(25);f.path.v.resize(25);f.path.a.resize(25);
    check=f.evaluate({});
    Expect(!check.safe&&check.reason=="insufficient_completion_horizon","short candidate cannot certify future merge");
    f.path=old;f.path.v[1]=NAN;
    Expect(!f.evaluate({}).safe,"non-finite candidate fails closed");
    f.path=old;auto malformed=leader;malformed.geometry_valid=false;
    Expect(!f.evaluate({malformed}).safe,"invalid object geometry never means free space");
    malformed=leader;malformed.x=NAN;
    Expect(!f.evaluate({malformed}).safe,"non-finite object center never means free space");
    f.cfg.max_collision_samples=2;
    Expect(f.evaluate({}).reason=="collision_sample_budget","search budget exhaustion cannot approve");
    f.cfg.max_collision_samples=12000;
    const auto timed_out=EvaluateHighwayMergeCandidate(f.path,f.zone,f.ref,{},f.cfg,
        MakeEgoShape(),MakeCollisionCfg(),.1,true,0,std::chrono::steady_clock::now());
    Expect(!timed_out.safe&&timed_out.reason=="planning_time_budget","expired search never approves a partial certificate");
    const auto invalid_time=EvaluateHighwayMergeCandidate(f.path,f.zone,f.ref,{},f.cfg,
        MakeEgoShape(),MakeCollisionCfg(),.1,true,NAN);
    Expect(!invalid_time.safe,"invalid object/ego time alignment fails closed");
    auto stale=MakeVehicleAt(16,50,0,0,8);
    const auto at_epoch=f.evaluate({stale});
    const auto aged=EvaluateHighwayMergeCandidate(f.path,f.zone,f.ref,{stale},f.cfg,
        MakeEgoShape(),MakeCollisionCfg(),.1,true,1.25);
    Expect(at_epoch.safe&&!aged.safe,"source-time offset moves an old rear object to its actual current pose");
    f.cfg.enabled=false;
    Expect(!f.evaluate({}).safe,"TEST6: disabled policy never approves");
}
void TestHighwayContinuationPrefix() {
    HighwayFixture f;
    // A distant actor changes collision sampling density without blocking us.
    const std::vector<TrackedObject> remote={MakeVehicleAt(91,300,50,0,11)};
    const auto full=f.evaluate(remote);
    Expect(full.safe&&full.covers_completion,"dense sampling certifies the full continuation");
    const double end=(full.certified_path_samples-1)*.1;
    Expect(end-full.clear_time_s+1e-9>=f.cfg.post_merge_horizon_s,
           "retained prefix includes two full seconds after checker clearance");
    Expect(end-full.clear_time_s<f.cfg.post_merge_horizon_s+.1+1e-9,
           "unused generated tail is limited to one trajectory-grid rounding step");
    Expect(full.certified_path_samples<f.path.x.size(),"checker returns a shorter certified prefix");

    // The former generator clipped at the first motion-grid clear sample.
    // Its tail is genuinely too short for this denser check and stays rejected.
    const auto original=f.path;
    const auto old_end=static_cast<std::size_t>(std::ceil((120+2.25-70)/8/.1))+20+1;
    for(auto* v:{&f.path.x,&f.path.y,&f.path.yaw,&f.path.kappa,&f.path.v,&f.path.a})v->resize(old_end);
    const auto short_check=f.evaluate(remote);
    Expect(!short_check.safe&&short_check.reason=="insufficient_completion_horizon",
           "a pre-trimmed short continuation never receives an artificial tolerance");
    Expect(short_check.certified_path_samples==0,"failed candidates expose no certified prefix");
    f.path=original;
    // Collision checks include the continued driving after rear clearance.
    auto obstacle=MakeVehicleAt(92,70+8*(full.clear_time_s+1),0,0,0);
    const auto collision=EvaluateHighwayMergeCandidate(f.path,f.zone,f.ref,{obstacle},f.cfg,
        MakeEgoShape(),MakeCollisionCfg(),.1,false);
    Expect(!collision.safe&&collision.reason=="candidate_collision",
           "post-completion collision is checked before any prefix is accepted");
    f.cfg.post_merge_horizon_s=NAN;
    Expect(!f.evaluate({}).safe,"invalid continuation duration fails closed");
}
void TestHighwayPredictionAndLaneScope() {
    auto object=MakeVehicleAt(101,100,4,0,10);
    object.motion_state=ObjectMotionState::DYNAMIC;
    HighwayMergeConfig cfg=MakeHighwayCfg();cfg.speed_error_mps=5;
    cfg.speed_error_floor_mps=1;cfg.speed_error_correlation_s=.5;
    Expect(std::fabs(HighwayPositionError(5,.2,cfg)-1)<1e-9,
           "near-term sensor error remains fully represented");
    Expect(std::fabs(HighwayPositionError(5,10,cfg)-50)<1e-9&&
           HighwayPositionError(5,15,cfg)>HighwayPositionError(5,10,cfg),
           "observed uncertainty persists over the complete path rather than shrinking without evidence");
    auto provisional=PrepareHighwayObjectPrediction(object,nullptr,cfg);
    Expect(provisional.prediction_max_speed==15,
           "new tracks retain the full sensor speed-error ceiling");
    ObservedObjectMotion track;
    for(int i=0;i<6;++i) {
        auto sample=object;sample.x=95+i;sample.vy=-.4;
        track.Observe(sample,100+.1*i,1);
    }
    object=track.last;object.yaw_rate=0;
    const auto lane=MakeStraightRefLine(400);
    const auto fitted=PrepareHighwayObjectPrediction(object,&track,cfg,&lane);
    const double tangent_speed=std::hypot(10,.4);
    Expect(std::fabs(fitted.vx-10)<1e-6&&fitted.vy==object.vy,
           "a position fit refines longitudinal speed without changing observed lateral motion");
    Expect(std::fabs(fitted.prediction_max_speed-(tangent_speed+1))<1e-6,
           "stable fit retains a nonzero residual-speed floor over the full horizon");
    auto jumping=track;jumping.samples[2].y+=2;
    const auto robust=PrepareHighwayObjectPrediction(object,&jumping,cfg,&lane);
    Expect(std::fabs(robust.prediction_max_speed-std::hypot(object.vx,object.vy)-1)<1e-6,
           "a transverse center jump does not erase consistent longitudinal observations");
    auto turned=object;turned.yaw_rate=.2;
    const auto turning=PrepareHighwayObjectPrediction(turned,&track,cfg);
    Expect(std::fabs(turning.prediction_max_speed-std::hypot(turned.vx,turned.vy)-5)<1e-6,
           "an unresolved turn retains the original motion model and ceiling");

    HighwayFixture f;f.cfg.speed_error_mps=5;
    f.cfg.speed_error_floor_mps=5; // isolate lane classification from noise-window reduction
    auto parallel=MakeVehicleAt(102,180,3,0,8);
    parallel.motion_state=ObjectMotionState::DYNAMIC;
    parallel.vx=-8;parallel.vy=-.1;
    // Small physical OBB; speed uncertainty alone reaches into the target
    // polygon, but remains clear of ego throughout the certified prefix.
    for(auto& p:parallel.corners)p.y=parallel.y+(p.y-parallel.y)*(.2/1.9);
    parallel.width=.2;
    Expect(f.evaluate({parallel}).safe,
           "another lane is not target traffic solely because its speed hull touches the target polygon");
    auto crossing=MakeVehicleAt(103,110,5,-M_PI/2,1);
    crossing.motion_state=ObjectMotionState::DYNAMIC;
    const auto blocked=f.evaluate({crossing});
    Expect(!blocked.safe&&blocked.reason=="candidate_collision",
           "a real off-corridor crossing actor still blocks the physical trajectory");
}
void TestHighwayReferenceArrival() {
    HighwayFixture f;
    f.cfg.completion_lateral_m=.5;f.cfg.completion_heading_rad=.12;
    const auto check=[&](const std::vector<TrackedObject>& objects,bool entering=true) {
        return EvaluateHighwayMergeCandidate(f.path,f.zone,f.ref,objects,f.cfg,
            MakeEgoShape(),MakeCollisionCfg(),.1,entering,0,
            std::chrono::steady_clock::time_point::max(),true);
    };
    const auto clear=check({});
    Expect(clear.safe&&clear.entry_time_s==0&&clear.clear_time_s==0,
        "actual target footprint arrival does not wait for the old mission station");
    Expect(clear.certified_path_samples==21,"actual arrival still retains two full seconds of checked continuation");
    auto rear=MakeVehicleAt(201,60,0,0,8);rear.motion_state=ObjectMotionState::DYNAMIC;
    const auto blocked=check({rear});
    Expect(!blocked.safe&&blocked.reason=="rear_gap_ttc_or_required_braking",
        "rear admission policy applies at actual lane entry before the mapped taper");
    Expect(check({rear},false).safe,"committed lane motion does not stop to request the rear gap again");
    auto fast_rear=MakeVehicleAt(203,55,0,0,20);fast_rear.motion_state=ObjectMotionState::DYNAMIC;
    Expect(!check({fast_rear}).safe,"fast following traffic still prevents a new lane-entry approval");
    Expect(check({fast_rear},false).safe,"an established committed vehicle does not stop for rear catch-up alone");
    const auto straight=f.path;
    for(std::size_t i=0;i<f.path.y.size();++i) {
        const double q=std::min(1.0,i*.1);
        f.path.y[i]=-3.5+3.5*(10*q*q*q-15*q*q*q*q+6*q*q*q*q*q);
        f.path.yaw[i]=std::atan(3.5*(30*q*q-60*q*q*q+30*q*q*q*q)/8);
    }
    auto responsive=MakeVehicleAt(206,46,0,0,16);responsive.motion_state=ObjectMotionState::DYNAMIC;
    Expect(check({responsive},false).safe,
        "committed crossing retains forward motion when a separated follower can respond within admitted braking");
    f.path=straight;
    auto cutin=MakeVehicleAt(204,55,4,0,20);cutin.motion_state=ObjectMotionState::DYNAMIC;cutin.vy=-2;
    Expect(!check({cutin},false).safe,"a rear vehicle entering from another lane retains collision checks");
    auto oncoming=MakeVehicleAt(205,90,0,0,-8);oncoming.motion_state=ObjectMotionState::DYNAMIC;
    Expect(!check({oncoming},false).safe,"oncoming traffic never gets a rear-following exemption");
    auto collision=MakeVehicleAt(202,70,0,0,8);collision.motion_state=ObjectMotionState::DYNAMIC;
    Expect(!check({collision},false).safe,"post-commit physical collision remains a hard rejection");
    for(auto* v:{&f.path.x,&f.path.y,&f.path.yaw,&f.path.kappa,&f.path.v,&f.path.a})v->resize(20);
    Expect(!check({}).safe,"short continuation cannot complete reference-transition admission");
}

void TestHighwayBoundedRearResponse() {
    HighwayFixture f;
    f.cfg.completion_lateral_m=.5;f.cfg.completion_heading_rad=.12;
    f.cfg.speed_error_mps=.25;f.cfg.max_rear_decel_mps2=3;
    const auto check=[&](const TrackedObject& object,bool reference=true) {
        return EvaluateHighwayMergeCandidate(f.path,f.zone,f.ref,{object},f.cfg,
            MakeEgoShape(),MakeCollisionCfg(),.1,true,0,
            std::chrono::steady_clock::time_point::max(),reference);
    };
    for(std::size_t i=0;i<f.path.y.size();++i) {
        const double q=std::min(1.0,i*.1/3);
        f.path.y[i]=-3.5+3.5*(10*q*q*q-15*q*q*q*q+6*q*q*q*q*q);
        f.path.yaw[i]=std::atan(3.5/3*(30*q*q-60*q*q*q+30*q*q*q*q)/8);
    }
    auto rear=MakeVehicleAt(301,38,0,0,12);rear.motion_state=ObjectMotionState::DYNAMIC;
    const auto original=check(rear);
    rear.prediction_lane=std::make_shared<const RefLine>(f.ref);rear.prediction_lane_s=38;
    rear.prediction_lane_d=0;rear.prediction_lane_end_s=400;
    const auto responsive=check(rear);
    Expect(!original.safe&&responsive.safe,
        "confirmed target follower uses bounded reaction in BOTH dense collision and rear-gap admission");
    Expect(responsive.minimum_rear_gap_m>=f.cfg.min_rear_gap&&
           responsive.minimum_rear_ttc_s>=f.cfg.min_rear_ttc&&
           responsive.maximum_rear_decel_mps2<=3,
        "responsive admission retains rear gap/TTC and original 3m/s2 braking limits");
    Expect(!check(rear,false).safe,"ordinary/legacy merge evaluation does not inherit highway response");
    auto uncertain=rear;uncertain.prediction_min_speed=7;uncertain.prediction_max_speed=17;
    Expect(!check(uncertain).safe,"wide observed speed intervals cannot be erased by assuming a rear response");
    auto unconfirmed=rear;unconfirmed.motion_state=ObjectMotionState::UNKNOWN;
    Expect(!check(unconfirmed).safe,"unconfirmed motion cannot receive a rear-response hypothesis");
    auto fork=rear;fork.prediction_lane_end_s=55;
    Expect(!check(fork).safe,"an unresolved lane successor restores strict prediction");
    auto close=MakeVehicleAt(302,62,0,0,20);close.motion_state=ObjectMotionState::DYNAMIC;
    close.prediction_lane=rear.prediction_lane;close.prediction_lane_s=62;close.prediction_lane_end_s=400;
    Expect(!check(close).safe,"late close rear traffic still rejects before any assumed reaction");
    auto alongside=MakeVehicleAt(303,70,0,0,12);alongside.motion_state=ObjectMotionState::DYNAMIC;
    alongside.prediction_lane=rear.prediction_lane;alongside.prediction_lane_s=70;alongside.prediction_lane_end_s=400;
    Expect(!check(alongside).safe,"alongside traffic cannot become a responsive follower");
    auto contact=MakeVehicleAt(304,70,-3.5,0,12);contact.motion_state=ObjectMotionState::DYNAMIC;
    contact.prediction_lane=rear.prediction_lane;contact.prediction_lane_s=70;contact.prediction_lane_end_s=400;
    Expect(!check(contact).safe,"current physical contact is never exempted");
    highway_rear_response::State model;
    model.enabled=true;model.start=2;model.low=11.75;model.high=12.25;
    model.floor=8;model.required_decel=1;model.maximum_decel=3;
    const auto coast=model.distances(1.5),react=model.distances(3),speeds=model.speeds(3);
    Expect(std::fabs(coast.first-11.75*1.5)<1e-9&&std::fabs(coast.second-12.25*1.5)<1e-9,
        "the full original speed interval coasts until entry plus one-second response");
    Expect(react.first<react.second&&speeds.first<speeds.second&&speeds.second<12.25,
        "response occupancy keeps both speed uncertainty and bounded braking alternatives");
    Expect(model.speeds(10).first==0&&model.distances(10).first<model.distances(10).second,
        "the maximum-braking bound includes a rear vehicle slowing all the way to rest");
    const auto poly=model.footprint(rear,3);
    Expect(poly.size()>=3,"response retains the measured footprint through the shared lane sweep");
}

void TestObservedFastUnknownFragment() {
    HighwayFixture f;
    auto fragment=MakeVehicleAt(1433,70,2.05,0,10);
    fragment.type=0;fragment.motion_state=ObjectMotionState::DYNAMIC;
    fragment.width=.16;fragment.length=1.6;fragment.height=.05;
    for(int i=0;i<4;++i){
        const double dx=(i==0||i==3)?-.8:.8,dy=i<2?-.08:.08;
        fragment.corners[i]={fragment.x+dx,fragment.y+dy,0};
        fragment.corners[i+4]={fragment.x+dx,fragment.y+dy,.05};
    }
    Expect(!f.evaluate({fragment}).safe,
        "main observed fast-UNKNOWN fragment protection applies to actual admission candidate");
    Expect(fragment.width==.16&&fragment.corners[0].y==fragment.y-.08,
        "admission envelope never changes authoritative source geometry");
}

int main() {
    TestRoundaboutGapSafeWhenNoTraffic();
    TestRoundaboutGapBlockedByPrecedingVehicle();
    TestRoundaboutGapUnconfirmedWhenEgoFarAway();
    TestHighwayActualCandidate();
    TestHighwayContinuationPrefix();
    TestHighwayPredictionAndLaneScope();
    TestHighwayReferenceArrival();
    TestHighwayBoundedRearResponse();
    TestObservedFastUnknownFragment();
    std::cout<<g_failures<<" failures\n";return g_failures?1:0;
}
