#pragma once
struct MotionObject {
  bool b_IsDynamic = 0;
  double f32_AbsVx = 0;
  double f32_AbsVy = 0;
  double f32_CoreE = 0;
  double f32_CoreN = 0;
  double f32_CorrectedDisplacementSpeedKPH = 0;
  double f32_DecisionSpeedKPH = 0;
  double f32_DecisionVxKPH = 0;
  double f32_DecisionVyKPH = 0;
  double f32_MapDisplacementSpeedKPH = 0;
  double f32_MotionVelocityJumpKPH = 0;
  double f32_PlannerVxKPH = 0;
  double f32_PlannerVyKPH = 0;
  int s32_Id = 0;
  int s32_MissCount = 0;
  int s32_MotionAcceptedSamples = 0;
  int s32_MotionCandidateSamples = 0;
  int s32_MotionState = 0;
  int s32_MotionValidObservations = 0;
  double min_e=0,max_e=0,min_n=0,max_n=0;
};
