#!/usr/bin/env bash
set -euo pipefail

engine="${1:-dts-matrix-engine}"
connector_dir="/code/src/dt-duckiematrix/packages/duckiematrix_engine/connectors"
entity_dir="/code/src/dt-duckiematrix/packages/duckiematrix_engine/entities"

docker cp patches/duckiematrix_binary_mavlink_timing.patch \
    "${engine}:/tmp/duckiematrix_binary_mavlink_timing.patch"
docker cp patches/duckiematrix_binary_decoder_api.patch \
    "${engine}:/tmp/duckiematrix_binary_decoder_api.patch"
docker cp patches/duckiematrix_dd24_start_physics_after_bridge_ready.patch \
    "${engine}:/tmp/duckiematrix_dd24_start_physics_after_bridge_ready.patch"
docker cp patches/duckiematrix_dtps_lockstep_timeout.patch \
    "${engine}:/tmp/duckiematrix_dtps_lockstep_timeout.patch"
docker cp patches/rotorpy_lockstep_poll_granularity.patch \
    "${engine}:/tmp/rotorpy_lockstep_poll_granularity.patch"
docker cp patches/duckiematrix_bridge_counters_info.patch \
    "${engine}:/tmp/duckiematrix_bridge_counters_info.patch"

docker exec "${engine}" sh -lc "
set -eu
cd '${connector_dir}'
if ! grep -q 'MAVLink transport' data_connectors.py; then
    patch --batch --forward -p0 < /tmp/duckiematrix_binary_mavlink_timing.patch
fi
if ! grep -q 'raw_data.get_as_native_object() is None' data_connectors.py; then
    patch --batch --forward -p0 < /tmp/duckiematrix_binary_decoder_api.patch
fi
cd '${entity_dir}'
if ! grep -q 'Bridge routes ready; starting physics loop' dd24_robot_entity.py; then
    patch --batch --forward -p0 < /tmp/duckiematrix_dd24_start_physics_after_bridge_ready.patch
fi
if ! grep -q 'lockstep_timeout = 0.01 if mavlink_mode == "dtps"' dd24_robot_entity.py; then
    patch --batch --forward -p0 < /tmp/duckiematrix_dtps_lockstep_timeout.patch
fi
if ! grep -q 'Keep bridge counters visible for HIL readiness diagnosis' dd24_robot_entity.py; then
    patch --batch --forward -p0 < /tmp/duckiematrix_bridge_counters_info.patch
fi
python3 -m py_compile '${connector_dir}/data_connectors.py' '${entity_dir}/dd24_robot_entity.py'
cd /usr/local/lib/python3.12/dist-packages/rotorpy/vehicles
if ! grep -q 'timeout=0.001' px4_multirotor.py; then
    patch --batch --forward -p0 < /tmp/rotorpy_lockstep_poll_granularity.patch
fi
python3 -m py_compile px4_multirotor.py
"

echo "Matrix bridge fixes installed in ${engine}. Restart the engine before attaching Birdie."
