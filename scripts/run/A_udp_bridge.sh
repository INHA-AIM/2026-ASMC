#!/usr/bin/env bash
# A: MORAI UDP 브리지 (roscore도 여기서 뜬다. 가장 먼저 실행)
source "$(dirname "$(readlink -f "$0")")/_common.sh"
run_in_container "A_udp_bridge" "./scripts/udp_bridge.sh"
