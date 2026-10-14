#!/bin/bash
# run_batch.sh OUTDIR TAG SEEDS [OVERRIDE] [SHIMDIR] [JOB]  (JOB names the log and cache, so parallel jobs can share a TAG)
# Runs batch_missions in a fresh MATLAB with its own Simulink cache.
A="$(cd "$(dirname "$0")" && pwd)"
SIM="$A/../../egh446_simEnv2026_25b/sim_env"
OUT="$1"; TAG="$2"; SEEDS="$3"; OVR="${4:-[]}"; SHIM="${5:-$A/logging_shims}"
JOB="${6:-$TAG}"; CACHE="$OUT/cache_$JOB"; mkdir -p "$CACHE"
matlab -batch "addpath('$A'); batch_missions('$SIM','$SHIM','$OUT','$CACHE',$SEEDS,'$TAG',$OVR)" > "$OUT/$JOB.log" 2>&1
grep -E "MISSION|BATCH_DONE" "$OUT/$JOB.log"
