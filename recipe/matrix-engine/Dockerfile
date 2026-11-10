ARG MATRIX_BASE_IMAGE=duckietown/dt-duckiematrix:ente-arm64v8
FROM ${MATRIX_BASE_IMAGE}

COPY patches/duckiematrix_binary_mavlink_timing.patch /tmp/
COPY patches/duckiematrix_binary_decoder_api.patch /tmp/
COPY patches/duckiematrix_dd24_start_physics_after_bridge_ready.patch /tmp/

RUN cd /code/src/dt-duckiematrix/packages/duckiematrix_engine/connectors \
    && patch --batch --forward -p0 < /tmp/duckiematrix_binary_mavlink_timing.patch \
    && patch --batch --forward -p0 < /tmp/duckiematrix_binary_decoder_api.patch \
    && cd /code/src/dt-duckiematrix/packages/duckiematrix_engine/entities \
    && patch --batch --forward -p0 < /tmp/duckiematrix_dd24_start_physics_after_bridge_ready.patch \
    && python3 -m py_compile \
        /code/src/dt-duckiematrix/packages/duckiematrix_engine/connectors/data_connectors.py \
        /code/src/dt-duckiematrix/packages/duckiematrix_engine/entities/dd24_robot_entity.py
