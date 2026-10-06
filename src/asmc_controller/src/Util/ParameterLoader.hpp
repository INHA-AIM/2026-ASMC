#pragma once

#include <string>

#include "Global/Global.hpp"

// yaml 파일에서 MPC 파라미터 로드 (params를 채운다. 실패하면 기본값 유지)
void loadMPCParameters(const std::string& yaml_path, MPCParams& params);
