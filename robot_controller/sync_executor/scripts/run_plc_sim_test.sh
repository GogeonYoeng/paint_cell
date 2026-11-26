#!/usr/bin/env bash
# PLC 시뮬레이터 + sync_executor(--gantry plc, 로봇 mock) 종단 시험.
# 사용: run_plc_sim_test.sh <out_dir> <motion.csv> <cam.csv> [sim 옵션...] -- [executor 옵션...]
#   예: run_plc_sim_test.sh /tmp/t motion.csv door_cam.csv --drift-ppm 100 --servo-delay-ms 4 -- --gantry-delay-ms 4
# 환경변수: EXE(실행기 경로), PY(파이썬), STOP_AFTER(초, 설정 시 그 시간 뒤 실행기에 SIGINT)
set -u
OUT=$1; MOTION=$2; CAM=$3; shift 3
SIM_ARGS=(); EXE_ARGS=(); sep=0
for a in "$@"; do
  if [[ $a == "--" ]]; then sep=1; continue; fi
  if [[ $sep == 0 ]]; then SIM_ARGS+=("$a"); else EXE_ARGS+=("$a"); fi
done
HERE="$(cd "$(dirname "$0")" && pwd)"
PY=${PY:-python3}
EXE=${EXE:-$HERE/../build/sync_executor}
PORT=${PORT:-12004}
mkdir -p "$OUT"

$PY "$HERE/plc_sim.py" --port "$PORT" --truth-log "$OUT/truth.csv" "${SIM_ARGS[@]}" > "$OUT/sim.out" 2>&1 < /dev/null &
SIM_PID=$!
sleep 1.5
"$EXE" --motion "$MOTION" --gantry plc --cam "$CAM" --plc-ip 127.0.0.1 --plc-port "$PORT" \
  --log "$OUT/exec_log.csv" "${EXE_ARGS[@]}" > "$OUT/exec.out" 2>&1 < /dev/null &
EXE_PID=$!
if [[ -n "${STOP_AFTER:-}" ]]; then
  sleep "$STOP_AFTER"; kill -INT $EXE_PID 2>/dev/null
fi
wait $EXE_PID; EXE_RC=$?
sleep 1
kill -TERM $SIM_PID 2>/dev/null; wait $SIM_PID
echo "=== executor (rc=$EXE_RC)"; cat "$OUT/exec.out"
echo "=== plc_sim"; cat "$OUT/sim.out"
exit $EXE_RC
