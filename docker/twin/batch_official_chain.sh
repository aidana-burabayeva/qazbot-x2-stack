#!/bin/bash
# Host-side. N repeated runs of the official-MC chain with a given max_angular (SIM-ONLY experiment; the production value stays 0.5
# until the operator approves a change). Usage: batch_official_chain.sh PREFIX N [MAX_ANGULAR=0.5]
P=$1; N=$2; W=${3:-0.5}
CB="$(cd "$(dirname "$0")/../.." && pwd)"
for i in $(seq 1 $N); do
  QAZBOT_MAX_ANGULAR=$W "$CB/docker/twin/run_official_chain.sh" "${P}$(printf %02d $i)" > "$CB/logs/twin/${P}$(printf %02d $i).log" 2>&1
  echo "${P}$(printf %02d $i) rc=$?"
done
