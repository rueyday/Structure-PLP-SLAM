#!/usr/bin/env python3
"""
Receives real-time 3D line landmarks from the running Structure PLP-SLAM via ZMQ.

Each message is a MessagePack-encoded list of dicts:
  [{"id": int, "sp": [x,y,z], "ep": [x,y,z], "num_obs": int}, ...]

Usage:
  python line_map_consumer.py [--endpoint tcp://localhost:5557]
"""

import argparse
import time
import zmq
import msgpack
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="ZMQ line map consumer")
    parser.add_argument("--endpoint", default="tcp://localhost:5557",
                        help="ZMQ endpoint to connect to (default: tcp://localhost:5557)")
    args = parser.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(args.endpoint)
    sock.setsockopt(zmq.SUBSCRIBE, b"")  # subscribe to all messages

    print(f"Connected to {args.endpoint}, waiting for line map updates...")

    try:
        while True:
            msg_bytes = sock.recv()
            lines = msgpack.unpackb(msg_bytes, raw=False)

            if not lines:
                print("[update] 0 lines in map")
                continue

            print(f"\n[update] {len(lines)} line landmarks received at {time.strftime('%H:%M:%S')}")

            # Build numpy array: each row = [id, sp_x, sp_y, sp_z, ep_x, ep_y, ep_z, num_obs]
            data = np.array([
                [ln["id"],
                 ln["sp"][0], ln["sp"][1], ln["sp"][2],
                 ln["ep"][0], ln["ep"][1], ln["ep"][2],
                 ln["num_obs"]]
                for ln in lines
            ], dtype=float)

            lengths = np.linalg.norm(data[:, 4:7] - data[:, 1:4], axis=1)
            print(f"  ID range:   {int(data[:,0].min())} – {int(data[:,0].max())}")
            print(f"  Line length min/mean/max: "
                  f"{lengths.min():.3f} / {lengths.mean():.3f} / {lengths.max():.3f} m")
            print(f"  Observations min/mean/max: "
                  f"{int(data[:,7].min())} / {data[:,7].mean():.1f} / {int(data[:,7].max())}")

            # Show a sample of lines with their colors
            print("  Sample lines (id, length, RGB):")
            for ln in lines[:5]:
                r, g, b = ln.get("rgb", [128, 128, 128])
                sp = ln["sp"]
                ep = ln["ep"]
                length = np.linalg.norm(np.array(ep) - np.array(sp))
                print(f"    id={ln['id']:4d}  len={length:.3f}m  "
                      f"rgb=({r:3d},{g:3d},{b:3d})")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
