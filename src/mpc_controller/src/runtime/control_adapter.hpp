#pragma once

#include "global/global.hpp"
#include "udp_network/datatypes.hpp"

CtrlCmd MakeStopCommand();
CtrlCmd ToControlCommand(const MPCControl& control, const MPCParams& params);
