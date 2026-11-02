#!/usr/bin/env python3
"""Regenerate the example animation JSON files in this directory."""
import json
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rainbow_cube_gui as rc

random.seed(42)
OUT = os.path.dirname(os.path.abspath(__file__))


def dump(name, anim, nframes, fps=15, loop=True):
    anim.reset()
    frames = [anim.next_frame() for _ in range(nframes)]
    data = {"type": "animation", "size": 4, "fps": fps, "loop": loop, "frames": frames}
    path = os.path.join(OUT, name + ".json")
    with open(path, "w") as f:
        json.dump(data, f)
    print("wrote %s (%d frames)" % (path, len(frames)))


if __name__ == "__main__":
    dump("shell_pulse", rc.ShellAnim(), 12, fps=8)
    dump("layer_sweep", rc.SweepAnim(), 8, fps=6)
    dump("hue_fade", rc.HueFadeAnim(), 32, fps=16)
    dump("snake", rc.SnakeAnim(), 64, fps=20)
