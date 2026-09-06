#pragma once

#include "global/global.hpp"
#include "runtime/datatypes.hpp"

CtrlCmd MakeStopCommand();
CtrlCmd ToControlCommand(const MPCControl& control, const MPCParams& params);
