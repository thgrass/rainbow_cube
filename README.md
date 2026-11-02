# Rainbow Cube 4x4x4 — PC controller

GUI (Python + Qt) to fully control a Seeed Studio **Rainbow Cube 4x4x4**
(Rainbowduino v3.0 + 64 RGB LEDs) from a PC over USB serial, including
animations.

```
rainbow_cube_gui.py                  the GUI application
firmware/RainbowCubeSerial/          Arduino sketch that must be uploaded
                                     to the Rainbowduino exactly once
firmware/Rainbowduino/               pristine Seeed Rainbowduino v3.0 library
                                     (copy to ~/Arduino/libraries/)
examples/                            sample animation files (JSON)
requirements.txt
```

## 1. One-time device setup (firmware)

The Rainbowduino v3.0 is an Arduino-compatible board: the cube is *not*
controlled by a fixed factory protocol, so you upload a small serial
slave sketch once. After that the PC drives it over `/dev/ttyUSB0`.

1. Install the Arduino IDE and the **Rainbowduino v3.0 library**: copy
   `firmware/Rainbowduino/` (pristine Seeed v3.0 library) into
   `~/Arduino/libraries/Rainbowduino/`.
2. Open `firmware/RainbowCubeSerial/RainbowCubeSerial.ino`.
3. Tools → Board: **"Arduino Duemilanove or Diecimila w/ ATmega328"**,
   select the Rainbowduino's USB port, upload.
4. Slide switch on the Rainbowduino v3.0 must be in the **USB** position.

Equivalent headless commands (arduino-cli):

```bash
arduino-cli core install arduino:avr                 
cp -r firmware/Rainbowduino ~/Arduino/libraries/    # pristine v3.0 library
arduino-cli compile --fqbn arduino:avr:diecimila firmware/RainbowCubeSerial
arduino-cli upload  -p /dev/ttyUSB0 --fqbn arduino:avr:diecimila firmware/RainbowCubeSerial
```

## 2. Run the GUI

```bash
pip install -r requirements.txt          # pyserial + PyQt5 (PyQt6 also OK)
python3 rainbow_cube_gui.py --port /dev/ttyUSB0 --connect
```

Serial port permissions: your user must be in the `dialout` (or `plugdev`)
group, e.g. `sudo usermod -aG dialout $USER` (re-login afterwards).

## 3. Using the GUI

- **Connection** — pick the port (default `/dev/ttyUSB0`) and press
  *Connect*; the baud box defaults to **19200** (the highest reliable
  rate, see below). The app waits out the bootloader reset, then
  auto-detects the baud at which the device answers PING (it tries your
  selection first, then 19200/9600/57600/115200) — so even if you
  type 115200, it will find the device at 19200, warn you, and correct
  the box. If you do pick a different *reliable* baud (≤ 19200), it
  switches the device to it via `SET_BAUD`. On success it flashes the
  whole cube red twice (low brightness) as a visible "hello", then
  pushes the current frame + brightness. *Ping* re-checks the link at any
  time; *Link test* sends 50 frames + 20 single-voxel commands and
  reports how many arrived intact (via the device's STATS counters) —
  use it to verify a new cable/baud setting: it should always report
  `50/50 frames ok, cmds_err delta=0`. The GUI never writes to the port
  before the device has answered PING (opening the port resets the ATmega
  into its bootloader for ~1 s; bytes sent then are lost/garbled and would
  both freeze the GUI on blocking writes and corrupt the command stream).
  The connect handshake runs in a worker thread and the hello-blink /
  test-pattern sequences are scheduled on the event loop, so the UI stays
  responsive at all times.
- **Cube view** — four 4x4 grids, `Z=0` (bottom) … `Z=3` (top); columns
  are X, rows are Y.
  - left click / drag: paint with the brush color
  - right click / drag: erase (black)
  - double click a voxel: choose its color in a dialog
- **Brush** — from color button, from palette selector, or via R/G/B spin boxes.
- **Draw** — *Clear*, *Fill with brush*, *Random*, *Send frame*, and
  *Test pattern* (lights Z0 red, Z1 green, Z2 blue, Z3 white in turn, then
  sweeps a single voxel through all 64 positions — an obvious visual
  self-test). *Live update* sends every edit immediately, otherwise use
  *Send frame*. The *Brightness* slider sends a global brightness (0–255).
- **Raw command** — send arbitrary hex bytes to the device.
- **Built-in animations** — Rain, Snake, Shell pulse, Plasma wave,
  Sparkle, Layer sweep, Bouncing ball, Hue fade. Select one, set FPS /
  Loop, press *Play* (button becomes *Pause*), *Stop* to end.
- **Custom keyframe animation** — paint a frame, *Add current frame*,
  reorder with *Up/Down*, then *Play keyframes*. *Save...* / *Load...*
  store animations as JSON (see `examples/`). Scenes (single frames)
  can be saved/loaded from the *Scene* box.

## 4. Serial protocol v2 (host ↔ device)

Default 19200 baud, 8N1, binary. Every command is
`<cmd> <payload...> <checksum>` where `checksum = XOR(cmd, payload...)`.
The device verifies it; on a bad checksum or unknown command it flushes
its RX buffer (parser resync) and increments `cmds_err`.

Host → device:

| Byte | Name        | Payload                          | Meaning                              |
|------|-------------|----------------------------------|--------------------------------------|
| 0x01 | SET_PIXEL   | `z x y r g b`                    | set voxel (z,x,y ∈ 0..3)             |
| 0x02 | FILL        | `r g b`                          | all 64 voxels to one color           |
| 0x03 | CLEAR       | —                                | all voxels off                       |
| 0x04 | BRIGHTNESS  | `level` (0..255)                 | global brightness, re-applied        |
| 0x05 | FRAME       | 192 bytes                        | whole frame: z→x→y order, each `r g b` |
| 0x06 | PING        | —                                | device replies `PONG` (4 bytes)      |
| 0x07 | STATS       | —                                | replies `STAT`+u32 frames_ok+u32 cmds_err (LE) |
| 0x08 | SET_BAUD    | u32 LE baud                      | switch UART baud at runtime          |

Device → host: `PONG` for PING, `STAT`+8 bytes for STATS.

### Why 19200 baud (important)

The Rainbowduino library's timer ISR bit-bangs the MY9221 LED drivers
with interrupts disabled (~0.25 ms, every 625 µs). While
it runs, the ATmega UART hardware can hold only 2 received bytes, so a
byte is lost whenever two bytes arrive within one ISR window — i.e. the
link is reliable only when `byte_time >= ISR_time`. 

At higher rates a lost byte desyncs the command stream — that is why symptoms
like "setting a few voxels lights up the whole cube" (a stray `0x02`
FILL) or "send frame does nothing" (corrupted 192-byte frame) appear.
19200 is the default for this reason; at 19200 a full frame takes ~51 ms
on the wire, so the GUI caps animation FPS at `baud/2000` (~19 FPS).

## 6. Animation/scene JSON format

```json
{
  "type": "animation",
  "size": 4,
  "fps": 15,
  "loop": true,
  "frames": [ [[r,g,b], ... 64 entries in z->x->y order ...], ... ]
}
```

A scene file is `{"type": "scene", "size": 4, "voxels": [[r,g,b] × 64]}`.
