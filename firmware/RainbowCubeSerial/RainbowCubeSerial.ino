/*
 * RainbowCubeSerial - serial slave firmware for the Seeed Rainbowduino v3.0
 * driving the 4x4x4 RGB LED cube ("Rainbow Cube").
 *
 * Upload ONCE with the Arduino IDE (board: "Duemilanove or Diecimila w/
 * ATmega328", Rainbowduino v3.0 library installed, slide switch on USB).
 * After that the cube is driven entirely from the PC over /dev/ttyUSBx.
 *
 * Link: UART 8N1. Protocol (all multi-byte fields little-endian):
 *
 *   Every command is  <cmd> <payload...> <checksum>
 *   checksum = XOR of cmd and all payload bytes.
 *   On a bad checksum or unknown cmd the device flushes its RX buffer
 *   (parser resync) and increments cmds_err.
 *
 *   0x01 SET_PIXEL   z x y r g b      set one voxel (z,x,y in 0..3)
 *   0x02 FILL        r g b            all 64 voxels one color
 *   0x03 CLEAR                         all voxels off
 *   0x04 BRIGHTNESS  level            global brightness 0..255, re-applied
 *   0x05 FRAME       192 bytes        whole frame, order z->x->y, each r g b
 *   0x06 PING                          -> device replies "PONG"
 *   0x07 STATS                         -> "STAT" + u32 frames_ok + u32 cmds_err
 *   0x08 SET_BAUD    u32 baud          switch UART baud at runtime
 *
 * BAUD RATE: the display refresh ISR in the Rainbowduino library runs with
 * interrupts disabled for ~0.5 ms every 1.25 ms. The ATmega UART hardware
 * buffers only 2 bytes, so a received byte is lost whenever two bytes arrive
 * inside one ISR window. Reliable operation therefore needs
 * byte_time >= ISR_time, i.e. baud <= ~19200 with the stock library.
 * Measured on real hardware (STATS, N frames back-to-back):
 *   38400 -> 0/100 frames, thousands of errors; 19200 -> clean; 9600 -> clean.
 * Default is therefore 19200. The GUI auto-detects the baud anyway.
 */

#include <Arduino.h>
#include <string.h>
#include <Rainbowduino.h>

#define BAUD_RATE         19200
#define SERIAL_TIMEOUT_MS 100

#define CMD_SET_PIXEL  0x01
#define CMD_FILL       0x02
#define CMD_CLEAR      0x03
#define CMD_BRIGHTNESS 0x04
#define CMD_FRAME      0x05
#define CMD_PING       0x06
#define CMD_STATS      0x07
#define CMD_SET_BAUD   0x08

#define N           4                 // cube is N x N x N
#define FRAME_BYTES (N * N * N * 3)   // 192

static uint8_t cube[N][N][N][3];      // [z][x][y][r,g,b]
static uint8_t brightness = 255;
static uint32_t frames_ok = 0;
static uint32_t cmds_err  = 0;

// One voxel through the brightness scaler into the library frame buffer;
// the library's timer ISR scans it continuously, so this is flicker-free.
static void applyVoxel(uint8_t z, uint8_t x, uint8_t y)
{
    uint8_t r = (uint16_t)cube[z][x][y][0] * brightness / 255;
    uint8_t g = (uint16_t)cube[z][x][y][1] * brightness / 255;
    uint8_t b = (uint16_t)cube[z][x][y][2] * brightness / 255;
    Rb.setPixelZXY(z, x, y, r, g, b);
}

static void pushAll(void)
{
    for (uint8_t z = 0; z < N; z++)
        for (uint8_t x = 0; x < N; x++)
            for (uint8_t y = 0; y < N; y++)
                applyVoxel(z, x, y);
}

static void flushRx(void)
{
    while (Serial.available())
        Serial.read();
}

// Read len payload bytes + 1 checksum byte; verify XOR(cmd ^ payload).
// NOTE: len must be a wide type - FRAME_BYTES (192) overflows int8_t.
static bool readCommand(uint8_t cmd, uint8_t *buf, uint16_t len)
{
    if (Serial.readBytes((char *)buf, (size_t)len + 1) != (size_t)len + 1) {
        flushRx();
        return false;
    }
    uint8_t cs = cmd;
    for (uint16_t i = 0; i < len; i++)
        cs ^= buf[i];
    if (cs != buf[len]) {
        flushRx();
        return false;
    }
    return true;
}

static int payloadLen(uint8_t cmd)
{
    switch (cmd) {
    case CMD_SET_PIXEL:  return 6;
    case CMD_FILL:       return 3;
    case CMD_CLEAR:      return 0;
    case CMD_BRIGHTNESS: return 1;
    case CMD_FRAME:      return FRAME_BYTES;
    case CMD_PING:       return 0;
    case CMD_STATS:      return 0;
    case CMD_SET_BAUD:   return 4;
    default:             return -1;
    }
}

void setup()
{
    Serial.begin(BAUD_RATE);
    Serial.setTimeout(SERIAL_TIMEOUT_MS);
    Rb.init();
    Rb.blankDisplay();
    memset(cube, 0, sizeof(cube));
}

void loop()
{
    if (!Serial.available())
        return;

    uint8_t cmd = (uint8_t)Serial.read();
    int len = payloadLen(cmd);
    uint8_t buf[FRAME_BYTES + 1];

    if (len < 0) {
        cmds_err++;
        flushRx();
        return;
    }
    if (!readCommand(cmd, buf, (uint16_t)len)) {
        cmds_err++;
        return;
    }

    switch (cmd) {
    case CMD_SET_PIXEL: {
        uint8_t z = buf[0], x = buf[1], y = buf[2];
        if (z < N && x < N && y < N) {
            cube[z][x][y][0] = buf[3];
            cube[z][x][y][1] = buf[4];
            cube[z][x][y][2] = buf[5];
            applyVoxel(z, x, y);
        }
        break;
    }
    case CMD_FILL:
        for (uint8_t z = 0; z < N; z++)
            for (uint8_t x = 0; x < N; x++)
                for (uint8_t y = 0; y < N; y++) {
                    cube[z][x][y][0] = buf[0];
                    cube[z][x][y][1] = buf[1];
                    cube[z][x][y][2] = buf[2];
                }
        pushAll();
        break;
    case CMD_CLEAR:
        memset(cube, 0, sizeof(cube));
        Rb.blankDisplay();
        break;
    case CMD_BRIGHTNESS:
        brightness = buf[0];
        pushAll();
        break;
    case CMD_FRAME:
        memcpy(cube, buf, sizeof(cube));
        pushAll();
        frames_ok++;
        break;
    case CMD_PING:
        Serial.write("PONG", 4);
        break;
    case CMD_STATS:
        Serial.write("STAT", 4);
        Serial.write((const uint8_t *)&frames_ok, 4);
        Serial.write((const uint8_t *)&cmds_err, 4);
        break;
    case CMD_SET_BAUD: {
        uint32_t b = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                     ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
        if (b >= 9600 && b <= 230400) {
            Serial.begin(b);
            Serial.setTimeout(SERIAL_TIMEOUT_MS);
            flushRx();
        }
        break;
    }
    }
}
