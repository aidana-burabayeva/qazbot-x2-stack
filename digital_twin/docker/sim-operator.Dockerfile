# syntax=docker/dockerfile:1.7
# Thin layer over the existing immutable sim image: ONLY the worker-capsule guard changes (adds the OPERATOR_SIM capsule kind).
# Everything else (libraries, AimDK, supervisor, world generator) is byte-identical to the base image.
ARG BASE=x2-twin-sim:nightly
FROM ${BASE}
USER root
COPY docker/sim/check_authorization.py /opt/twin-sim/check_authorization.py
RUN chmod 0755 /opt/twin-sim/check_authorization.py
USER twin
