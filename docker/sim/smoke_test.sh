#!/bin/bash
# Smoke test for a running swarm simulation: topics present, message rates, real-time factor.
# Inside the container:  docker exec -it swarm_sim_cloud smoke-test [--duration 20] [--min-rate depth/image=28] [--min-rtf 0.9]
# Options are documented in smoke_test.py (--help).

source /opt/ros/jazzy/setup.bash
[ -f /root/swarm_ws/install/setup.bash ] && source /root/swarm_ws/install/setup.bash
exec python3 "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/smoke_test.py" "$@"
