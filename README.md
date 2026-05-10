# Balanc Robot (M5Stick-C / ESP32)

This repository builds firmware for the `m5stick-c` target using PlatformIO + ESP-IDF.

## 1) Prerequisites

- macOS, Linux, or Windows
- Python 3.9+
- Git
- USB cable for your board

You can use either:

- VS Code + PlatformIO IDE extension (recommended), or
- PlatformIO CLI only

## 2) Clone the Project

```bash
git clone https://github.com/daniyal200/Balanc-Robot.git
cd Balanc-Robot
```

## 3) Install PlatformIO

### Option A: VS Code (recommended)

1. Install VS Code.
2. Install the PlatformIO IDE extension.
3. Open this folder in VS Code.
4. Wait for first-time package/index setup to finish.

### Option B: CLI

```bash
python3 -m pip install --user -U platformio
```

If `pio` is still not found, restart your terminal or use:

```bash
python3 -m platformio --version
```

## 4) Build

### In VS Code

- Run the default build task (`PlatformIO: Build m5stick-c`), or
- Open terminal and run:

```bash
~/.platformio/penv/bin/pio run -e m5stick-c
```

### In CLI

```bash
pio run -e m5stick-c
```

## 5) Upload Firmware

Find your serial port first:

```bash
pio device list
```

Upload:

```bash
pio run -e m5stick-c -t upload --upload-port <PORT>
```

Examples of `<PORT>`:
- macOS: `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`
- Linux: `/dev/ttyUSB0` or `/dev/ttyACM0`
- Windows: `COM3`, `COM4`, etc.

## 6) Serial Monitor

```bash
pio device monitor -b 115200 -p <PORT>
```

## 7) Clean Build

```bash
pio run -e m5stick-c -t clean
pio run -e m5stick-c
```

## 8) Common Issues

### `platformio` or `pio` command not found

If you are using VS Code PlatformIO extension, use the extension-managed binary directly:

```bash
~/.platformio/penv/bin/pio run -e m5stick-c
```

### First build takes a long time

Normal. Toolchains and framework packages are downloaded on first run.

### Upload fails

- Check USB cable (must support data, not charge-only)
- Ensure correct port is selected
- Close any other serial monitor connected to the same port

## 9) Project Notes

- Main build environment: `m5stick-c`
- Framework: ESP-IDF (via PlatformIO)
- Main application sources are under `main/`
