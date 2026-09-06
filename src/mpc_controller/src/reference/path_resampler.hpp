#pragma once

#include "global/global.hpp"

ReferencePath ToReferencePath(const PlannedPath& path, double destination_dt,
                              int steps);
