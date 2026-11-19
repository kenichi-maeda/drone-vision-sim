ARG MATRIX_BASE_IMAGE=duckietown/dt-duckiematrix:ente-arm64v8
FROM ${MATRIX_BASE_IMAGE}

COPY patches/duckiematrix_binary_mavlink_timing.patch /tmp/
COPY patches/duckiematrix_binary_decoder_api.patch /tmp/
COPY patches/duckiematrix_dd24_start_physics_after_bridge_ready.patch /tmp/
COPY patches/duckiematrix_dtps_lockstep_timeout.patch /tmp/
COPY patches/duckiematrix_bridge_counters_info.patch /tmp/
COPY patches/rotorpy_lockstep_poll_granularity.patch /tmp/
COPY patches/duckiematrix_keyboard_hold.patch /tmp/

RUN cd /code/src/dt-duckiematrix/packages/duckiematrix_engine/connectors \
    && patch --batch --forward -p0 < /tmp/duckiematrix_binary_mavlink_timing.patch \
    && patch --batch --forward -p0 < /tmp/duckiematrix_binary_decoder_api.patch \
    && cd /code/src/dt-duckiematrix/packages/duckiematrix_engine/entities \
    && patch --batch --forward -p0 < /tmp/duckiematrix_dd24_start_physics_after_bridge_ready.patch \
    && patch --batch --forward -p0 < /tmp/duckiematrix_dtps_lockstep_timeout.patch \
    && patch --batch --forward -p0 < /tmp/duckiematrix_bridge_counters_info.patch \
    && patch --batch --forward -p0 < /tmp/duckiematrix_keyboard_hold.patch \
    && python3 -m py_compile \
        /code/src/dt-duckiematrix/packages/duckiematrix_engine/entities/quadcopter_entity.py \
        /code/src/dt-duckiematrix/packages/duckiematrix_engine/connectors/data_connectors.py \
        /code/src/dt-duckiematrix/packages/duckiematrix_engine/entities/dd24_robot_entity.py \
    && cd /usr/local/lib/python3.12/dist-packages/rotorpy/vehicles \
    && patch --batch --forward -p0 < /tmp/rotorpy_lockstep_poll_granularity.patch \
    && python3 -m py_compile px4_multirotor.py
