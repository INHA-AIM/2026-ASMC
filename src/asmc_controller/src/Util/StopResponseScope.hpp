#pragma once

// Separate ordinary STOP and roundabout PREPARE so response estimates reset
// on entry/exit. Committed traversal and other maneuvers retain their mapping.
enum class StopResponseScope { NONE, ORDINARY_STOP, ROUNDABOUT_WAITING };

template<class Command>
StopResponseScope ResolveStopResponseScope(const Command& command) {
    if(command.maneuver_type==Command::STOP && command.merge_type!=Command::MERGE_ROUNDABOUT)
        return StopResponseScope::ORDINARY_STOP;
    if(command.maneuver_type==Command::MERGE && command.merge_type==Command::MERGE_ROUNDABOUT &&
       command.execution_request==Command::PREPARE)
        return StopResponseScope::ROUNDABOUT_WAITING;
    return StopResponseScope::NONE;
}
