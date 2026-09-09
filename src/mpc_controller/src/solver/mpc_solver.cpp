#include "mpc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "math/utils.hpp"
#include "model/vehicle_model.hpp"
#include "cost/cost_function.hpp"
#include "constraints/constraints.hpp"

// ========================================
// 헬퍼: control sequence 전체에 box constraint 투영
// ========================================
static void projectControls(std::vector<MPCControl>& controls,
                            const MPCControl& prev_control,
                            const MPCParams& params)
{
    MPCControl u_prev = prev_control;
    for (auto& u : controls) {
        clipControl(u, params);
        clipSteeringRate(u, u_prev, params.steering_rate_max, params.dt);
        u_prev = u;
    }
}

// ========================================
// 헬퍼: 현재 reference로 stage별 steering feed-forward를 계산한다.
//   기존 fallback 분기(warm-start 없을 때)와 curvature-aware rebase(있을
//   때) 양쪽이 "이번 solve의 feed-forward"를 필요로 하므로 한 곳에서
//   만든다 - 두 곳에서 atan(wheelbase*curvature)를 각자 계산하지 않는다.
// ========================================
std::vector<double> computeCurrentSteeringFeedforward(
    const ReferencePath& ref, int N, const MPCParams& params)
{
    std::vector<double> ff(N, 0.0);
    for (int i = 0; i < N; ++i) {
        const double k_ref_i = (ref.size() > 0)
            ? ref.k_ref[std::min(static_cast<size_t>(i), ref.size() - 1)]
            : 0.0;
        ff[i] = SteeringFeedforward(k_ref_i, params.wheelbase, params.steering_max);
    }
    return ff;
}

// ========================================
// curvature-aware steering warm-start rebase.
//
//   과거(fix/yuntae-frenet-speed-continuity 이전부터) warm-start는 "이전
//   최적 절대 steering을 한 스텝 shift"만 했다. 그 절대값은 이전 reference
//   기준으로 최적화된 값이라, 새 커브가 나타나면(reference curvature가
//   바뀌면) 그 절대값이 새 목표와 무관한 "과거 방향"에 갇혀 있다 - MORAI
//   실측에서 ffpeak가 이미 크게 반대 부호인데 steer_cmd가 여러 cycle 동안
//   이전 값 근처에 머무르는 curve-entry lag로 나타났다.
//
//   그래서 절대값이 아니라 "이전 최적해가 이전 feed-forward 대비 얼마나
//   보정했는가"(feedback residual)만 이어받는다:
//
//     residual_prev[i]  = warm_start[i].delta - warm_start_ff[i]
//     residual_shift[i] = residual_prev[min(i+1, N-1)]   (기존 shift와 동일 인덱싱)
//     delta_init[i]     = current_ff[i] + residual_shift[i]
//
//   새 tuning 파라미터를 넣지 않는다 - blend factor 없이 100% residual을
//   그대로 이어받는다. accel은 이 함수가 건드리지 않는다 - 종방향
//   warm-start는 기존 그대로(호출자가 accel을 별도로 shift한다).
//
//   전제(맞지 않으면 호출자가 폴백해야 한다): warm_start.size() == N이고
//   warm_start_ff.size() == warm_start.size()이며 모든 값이 유한하다.
//   solveMPC 밖에서도(단위 테스트) 쓸 수 있도록 헤더에 선언한다 - ROS 타입
//   의존이 없는 순수 함수라 MPC core 경계를 넘지 않는다.
// ========================================
std::vector<double> RebaseSteeringWarmStart(
    const std::vector<MPCControl>& warm_start,
    const std::vector<double>& warm_start_ff,
    const std::vector<double>& current_ff,
    int N)
{
    std::vector<double> rebased(N, 0.0);
    for (int i = 0; i < N; ++i) {
        const int shifted_index = std::min(i + 1, N - 1);
        const double residual =
            warm_start[shifted_index].delta - warm_start_ff[shifted_index];
        rebased[i] = current_ff[i] + residual;
    }
    return rebased;
}

// warm_start/warm_start_ff가 curvature-aware rebase에 안전하게 쓰일 수
// 있는 조건. 크기 불일치나 non-finite 값이 하나라도 있으면 기존
// feed-forward-only fallback으로 안전하게 수렴한다 - 짝이 맞지 않는 값을
// 섞어 rebase하지 않는다.
bool WarmStartFeedforwardIsValid(
    const std::vector<MPCControl>& warm_start,
    const std::vector<double>& warm_start_ff,
    int N)
{
    if (static_cast<int>(warm_start.size()) < N) return false;
    if (warm_start_ff.size() != warm_start.size()) return false;
    for (const auto& u : warm_start) {
        if (!std::isfinite(u.delta) || !std::isfinite(u.accel)) return false;
    }
    for (double ff : warm_start_ff) {
        if (!std::isfinite(ff)) return false;
    }
    return true;
}

// ========================================
// 헬퍼: 선형 모델로 trajectory 예측
//   x_{k+1} = A_k * x_k + B_k * u_k + c_k
//   predictTrajectory() 대신 사용 → 행렬 곱셈만이라 훨씬 빠름
// ========================================
static std::vector<MPCState> predictTrajectoryLTV(
    const MPCState&                     x0,
    const std::vector<MPCControl>&      U,
    const std::vector<LinearizedModel>& models)
{
    std::vector<MPCState> traj;
    traj.reserve(U.size() + 1);
    traj.push_back(x0);

    double x[4] = { x0.x, x0.y, x0.yaw, x0.vx };

    for (size_t k = 0; k < U.size(); ++k) {
        const auto& lm = models[k];
        double u[2] = { U[k].delta, U[k].accel };

        // x_next = A*x + B*u + c
        double x_next[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            x_next[i] = lm.c[i];
            for (int j = 0; j < 4; ++j) x_next[i] += lm.A[i][j] * x[j];
            for (int j = 0; j < 2; ++j) x_next[i] += lm.B[i][j] * u[j];
        }

        MPCState st;
        st.x   = x_next[0];
        st.y   = x_next[1];
        st.yaw = normalizeAngle(x_next[2]);
        st.vx  = clip(x_next[3], 0.0, 100.0);
        traj.push_back(st);

        x[0] = st.x; x[1] = st.y; x[2] = st.yaw; x[3] = st.vx;
    }
    return traj;
}

// ========================================
// 헬퍼: cost 평가
//   models 있으면 선형 모델로 trajectory 예측 (빠름)
//   없으면 비선형 predictTrajectory() 사용 (fallback)
// ========================================
static double evalCost(
    const MPCState&                     x0,
    const std::vector<MPCControl>&      U,
    const ReferencePath&                ref,
    const MPCControl&                   prev_control,
    const MPCParams&                    params,
    const std::vector<LinearizedModel>* models   = nullptr,
    std::vector<MPCState>*              out_traj = nullptr)
{
    std::vector<MPCState> traj;
    if (models && models->size() == U.size()) {
        // 선형 모델로 trajectory 예측
        traj = predictTrajectoryLTV(x0, U, *models);
    } else {
        // fallback: 비선형 forward sim
        traj = predictTrajectory(x0, U, params.dt, params.wheelbase,
                                 params.vel_min, params.vel_max);
    }
    double c = computeTotalCost(traj, U, ref, prev_control, params);
    if (out_traj) *out_traj = std::move(traj);
    return c;
}

// ========================================
// 헬퍼: 선형 모델 기반 gradient 계산 (central diff)
//
//   기존과 수식은 동일하지만 evalCost 내부에서
//   predictTrajectory() 대신 행렬 곱(predictTrajectoryLTV)을 사용하므로
//   연산량이 대폭 줄어듦
//
//   기존: evalCost 4N번 호출 → 내부에서 predictTrajectory() 4N번 실행
//   변경: evalCost 4N번 호출 → 내부에서 행렬 곱만 (비선형 적분 없음)
// ========================================
static std::vector<MPCControl> computeGradient(
    const MPCState&                     x0,
    const std::vector<MPCControl>&      U,
    const std::vector<LinearizedModel>& models,
    const ReferencePath&                ref,
    const MPCControl&                   prev_control,
    const MPCParams&                    params)
{
    const int N   = static_cast<int>(U.size());
    const double eps = 1e-3;
    std::vector<MPCControl> grad(N);

    for (int k = 0; k < N; ++k) {
        // d/d_delta
        std::vector<MPCControl> U_p = U, U_m = U;
        U_p[k].delta += eps;
        U_m[k].delta -= eps;
        grad[k].delta = (evalCost(x0, U_p, ref, prev_control, params, &models)
                       - evalCost(x0, U_m, ref, prev_control, params, &models))
                       / (2.0 * eps);

        // d/d_accel
        U_p = U; U_m = U;
        U_p[k].accel += eps;
        U_m[k].accel -= eps;
        grad[k].accel = (evalCost(x0, U_p, ref, prev_control, params, &models)
                       - evalCost(x0, U_m, ref, prev_control, params, &models))
                       / (2.0 * eps);
    }
    return grad;
}

// ========================================
// MPC Solver
//   - LTV 1차 Taylor 선형화 기반 gradient 계산
//   - Projected Gradient Descent
//   - Armijo backtracking line search
// ========================================
MPCResult solveMPC(
    const MPCState&                x0,
    const ReferencePath&           ref,
    const MPCControl&              prev_control,
    const std::vector<MPCControl>& warm_start,
    const std::vector<double>&     warm_start_steering_feedforward,
    const MPCParams&               params)
{
    MPCResult result;
    result.success = false;

    if (ref.empty()) {
        result.solver_msg = "Empty reference path";
        result.control.delta = 0.0;
        result.control.accel = std::max(params.accel_min, -1.0);
        return result;
    }

    const int N = params.horizon;
    if (N <= 0) {
        result.solver_msg = "Invalid horizon";
        return result;
    }

    // 이번 solve의 reference로 만든 stage별 feed-forward. fallback
    // 초기화와 curvature-aware rebase 양쪽에 쓰고, 다음 cycle에 이번 해의
    // "feed-forward 대비 residual"을 계산할 수 있도록 result에도 그대로
    // 담아 돌려준다(호출자가 다음 solveMPC 호출의
    // warm_start_steering_feedforward로 넘긴다).
    const std::vector<double> current_ff =
        computeCurrentSteeringFeedforward(ref, N, params);

    // ====================================
    // 1) Warm-start: 이전 해를 한 스텝 shift, 부족분은 prev_control로
    //
    //    warm-start가 없는 첫 solve(또는 fail-safe 직후)에서는 모든 스텝을
    //    prev_control로만 채우면 코너에서 초기 조향 guess가 항상 0에서
    //    출발해 gradient descent가 먼 거리를 걸어야 한다. reference
    //    curvature 기반 feed-forward로 초기값을 주면 최적해에 더 가까운
    //    지점에서 시작한다 (Commit C). prev_control은 실제 이전 actuator
    //    상태이므로 accel은 그대로 이어받고 delta만 feed-forward로 바꾼다 -
    //    그래야 첫 horizon point가 이전 steering에서 순간이동하지 않는다
    //    (아래 clipSteeringRate가 prev_control 기준으로 투영).
    //
    //    curve-tracking stability iteration 3: warm-start가 있을 때도 이전
    //    "절대" steering을 그대로 이어받지 않는다 - 그 절대값은 이전
    //    reference 기준으로 최적화된 값이라 새 커브(reference curvature
    //    변화)와 무관한 과거 방향에 갇힐 수 있다(MORAI curve-entry lag 실측
    //    확인). 대신 이전 최적해가 "그 solve의 feed-forward 대비 얼마나
    //    보정했는가"(feedback residual)만 이어받아 이번 reference의
    //    feed-forward에 얹는다 - RebaseSteeringWarmStart 주석 참고. 이전
    //    feed-forward를 안전하게 재사용할 수 없으면(크기 불일치/비유한값)
    //    기존 순수 feed-forward-only 초기화로 그대로 폴백한다. accel
    //    초기화는 이 변경과 무관하게 그대로다.
    // ====================================
    std::vector<MPCControl> U(N);
    const bool use_curvature_aware_rebase =
        static_cast<int>(warm_start.size()) >= N &&
        WarmStartFeedforwardIsValid(warm_start, warm_start_steering_feedforward, N);
    if (use_curvature_aware_rebase) {
        const std::vector<double> rebased_delta = RebaseSteeringWarmStart(
            warm_start, warm_start_steering_feedforward, current_ff, N);
        for (int i = 0; i < N; ++i) {
            U[i].delta = rebased_delta[i];
            const int shifted_index = std::min(i + 1, N - 1);
            U[i].accel = warm_start[shifted_index].accel;
        }
    } else if (static_cast<int>(warm_start.size()) >= N) {
        // 이전 controls는 있지만 feed-forward 짝이 없거나(구현 전환 첫
        // cycle 등) 무효하다 - accel은 그대로 shift하고 steering만 안전한
        // feed-forward-only로 되돌린다.
        for (int i = 0; i < N; ++i) {
            const int shifted_index = std::min(i + 1, N - 1);
            U[i].delta = current_ff[i];
            U[i].accel = warm_start[shifted_index].accel;
        }
    } else {
        for (int i = 0; i < N; ++i) {
            U[i].delta = current_ff[i];
            U[i].accel = prev_control.accel;
        }
    }
    projectControls(U, prev_control, params);
    // [MPC-WARM] warm_projected0 진단 전용 스냅샷 - 이후 gradient descent가
    // U를 계속 갱신하므로 여기서 값을 미리 떼어 둔다.
    const double initial_guess_delta0 = U.empty() ? 0.0 : U[0].delta;

    // ====================================
    // 2) 현재 U 기반으로 LTV 선형화 모델 계산
    //    warm-start U를 동작점으로 삼아 각 스텝 A_k, B_k, c_k 확보
    //    → 이후 gradient/line search에서 비선형 forward sim 대체
    // ====================================
    std::vector<LinearizedModel> models = buildLTVModels(
        x0, U, params.dt, params.wheelbase, params.vel_min, params.vel_max);

    // ====================================
    // 3) 최적화 루프
    // ====================================
    double cost_cur = evalCost(x0, U, ref, prev_control, params, &models);
    double lr = params.lr_init;

    for (int iter = 0; iter < params.max_iterations; ++iter) {

        // ---- gradient 계산 (선형 모델 기반) ----
        std::vector<MPCControl> grad = computeGradient(
            x0, U, models, ref, prev_control, params);

        // 수렴 체크
        double grad_norm2 = 0.0;
        for (int k = 0; k < N; ++k)
            grad_norm2 += grad[k].delta * grad[k].delta
                        + grad[k].accel * grad[k].accel;
        if (grad_norm2 < params.convergence_eps * params.convergence_eps) break;

        // ---- Armijo backtracking line search ----
        double alpha = lr;
        std::vector<MPCControl> U_new(N);
        double cost_new = cost_cur;
        bool accepted = false;

        for (int ls = 0; ls < params.line_search_steps; ++ls) {
            for (int k = 0; k < N; ++k) {
                double w = 1.0 / (1.0 + 0.1 * k);
                U_new[k].delta = U[k].delta - alpha * w * grad[k].delta;
                U_new[k].accel = U[k].accel - alpha * w * grad[k].accel;
            }
            projectControls(U_new, prev_control, params);

            // U_new 기준으로 선형화 모델 갱신 후 cost 평가
            // U_old 기준 models로 U_new cost 평가 시 선형화 오차 누적 방지
            std::vector<LinearizedModel> models_new = buildLTVModels(
                x0, U_new, params.dt, params.wheelbase, params.vel_min, params.vel_max);
            cost_new = evalCost(x0, U_new, ref, prev_control, params, &models_new);

            if (cost_new < cost_cur - 1e-4 * alpha * grad_norm2) {
                accepted = true;
                models = std::move(models_new);  // accept된 models 반영
                break;
            }
            alpha *= 0.5;
        }

        if (accepted) {
            U = std::move(U_new);
            lr = std::min(params.lr_init, alpha * 1.5);
            // models는 line search에서 이미 U_new 기준으로 갱신 완료

            if (cost_cur - cost_new < params.convergence_eps) {
                cost_cur = cost_new;
                break;
            }
            cost_cur = cost_new;
        } else {
            lr *= 0.5;
            if (lr < params.lr_min) break;
        }
    }

    // ====================================
    // 4) 결과 패키징
    //    최종 trajectory는 비선형 모델로 다시 계산 (근사 오차 제거)
    // ====================================
    std::vector<MPCState> traj;
    evalCost(x0, U, ref, prev_control, params, nullptr, &traj);

    result.controls             = U;
    result.steering_feedforward  = current_ff;
    result.initial_guess_delta0  = initial_guess_delta0;
    result.predicted_states      = traj;
    result.cost                  = cost_cur;
    result.control               = U[0];
    result.success                = true;
    result.solver_msg             = "OK";
    return result;
}