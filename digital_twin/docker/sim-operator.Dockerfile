# syntax=docker/dockerfile:1.7
# Thin layer over the existing immutable sim image. The base image `nightly` predates the capsule guard: its sim_supervisor.sh has neither
# the authorization check nor the official-mc.lock flock (verified: no process held the lock). This layer installs the repository versions of
# exactly those two files (guard with the OPERATOR_SIM capsule kind + supervisor that enforces it). Everything else is byte-identical.
ARG BASE=x2-twin-sim:nightly
FROM ${BASE}
USER root
COPY docker/sim/check_authorization.py docker/sim/sim_supervisor.sh /opt/twin-sim/
RUN chmod 0755 /opt/twin-sim/check_authorization.py /opt/twin-sim/sim_supervisor.sh
USER twin
