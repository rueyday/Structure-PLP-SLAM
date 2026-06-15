#!/bin/bash
# Run Structure-PLP-SLAM with DeepLSD online line detection.
#
# Usage:
#   ./run_deeplsd_slam.sh <config.yaml> <sequence_dir> <output.db> [extra slam args...]
#
# Environment:
#   SLAM_VOCAB  - path to ORB vocab (default: orb_vocab/orb_vocab.dbow2)
#   DEEPLSD_CKPT - path to DeepLSD weights (default: ../weights/deeplsd_md.tar)
#   DEEPLSD_PORT - ZMQ port (default: 5599)
#   PYTHON      - python interpreter (default: python3)
#
# Example:
#   ./run_deeplsd_slam.sh example/tum_rgbd/7Scenes_rgbd.yaml \
#       /data/7scenes/stairs/seq01 results/seq01.db

set -e

CONFIG="${1:?Usage: $0 <config.yaml> <seq_dir> <output.db> [extra args...]}"
SEQ_DIR="${2:?}"
OUT_DB="${3:?}"
shift 3

VOCAB="${SLAM_VOCAB:-orb_vocab/orb_vocab.dbow2}"
CKPT="${DEEPLSD_CKPT:-$(dirname "$0")/../weights/deeplsd_md.tar}"
PORT="${DEEPLSD_PORT:-5599}"
PYTHON="${PYTHON:-python3}"
SLAM_BIN="$(dirname "$0")/build/run_tum_rgbd_slam_with_line"

# Start DeepLSD server
echo "[run_deeplsd_slam] Starting DeepLSD server on port $PORT ..."
"$PYTHON" "$(dirname "$0")/../deeplsd_server.py" \
    --port "$PORT" --ckpt "$CKPT" &
SERVER_PID=$!
trap "kill $SERVER_PID 2>/dev/null; wait $SERVER_PID 2>/dev/null" EXIT

# Wait for server to be ready (it prints "Listening on port" when ready)
echo "[run_deeplsd_slam] Waiting for server to be ready ..."
for i in $(seq 1 30); do
    sleep 1
    if kill -0 "$SERVER_PID" 2>/dev/null; then
        # Try a simple ZMQ ping via Python to confirm it's up
        if "$PYTHON" -c "
import zmq, sys
ctx = zmq.Context()
s = ctx.socket(zmq.REQ)
s.setsockopt(zmq.RCVTIMEO, 500)
s.setsockopt(zmq.SNDTIMEO, 500)
s.connect('tcp://localhost:${PORT}')
# send a tiny 1x1 image
import struct
msg = struct.pack('<ii', 1, 1) + bytes(1)
try:
    s.send(msg)
    s.recv()
    sys.exit(0)
except:
    sys.exit(1)
" 2>/dev/null; then
            echo "[run_deeplsd_slam] Server ready."
            break
        fi
    else
        echo "[run_deeplsd_slam] ERROR: Server process died."
        exit 1
    fi
done

# Run SLAM
echo "[run_deeplsd_slam] Running SLAM on $SEQ_DIR ..."
DEEPLSD_SERVER="tcp://localhost:${PORT}" \
"$SLAM_BIN" \
    -v "$VOCAB" \
    -c "$CONFIG" \
    -d "$SEQ_DIR" \
    --no-sleep --auto-term \
    -p "$OUT_DB" \
    "$@"

echo "[run_deeplsd_slam] Done. Map saved to $OUT_DB"
