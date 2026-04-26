# uav-onboard

Onboard software for the indoor UAV search mission.

This project owns camera processing, mission state management, MAVLink control,
GCS communication, safety handling, and headless logging.

## Layout

- `config/`: runtime TOML configuration
- `src/`: mission application source
- `tools/`: local development and debugging utilities
- `test_data/`: captured images, videos, and logs for repeatable tests
- `tests/`: unit tests
- `scripts/`: build, deploy, and run helpers
- `docs/`: design notes and protocol reference
- `logs/`: runtime logs

## Build

On a clean Raspberry Pi OS Lite 64-bit install, set up build dependencies first:

```bash
bash scripts/setup_rpi_dependencies.sh
```

Optional development tools:

```bash
bash scripts/setup_rpi_dependencies.sh --with-dev-tools
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Camera Bring-Up

```bash
./build/camera_preview --device 0 --frames 30 --save test_data/images/camera_smoke.jpg
```

Expected output includes the captured frame size, measured FPS, and the saved
image path.

## Telemetry Bring-Up

Start `uav-gcs` on the laptop first, then run:

```bash
./build/uav_onboard --config config --count 10
```

By default, telemetry is sent to the IPv4 local broadcast address:

```toml
[gcs]
ip = "255.255.255.255"
telemetry_port = 14550
```

This avoids editing the onboard config whenever the laptop IP changes. Some
guest Wi-Fi networks block local broadcast; if GCS receives nothing there, use
the laptop's explicit IPv4 address with `--gcs-ip` or in `network.toml`.

For a quick local loopback test, override the destination:

```bash
./build/uav_onboard --config config --gcs-ip 127.0.0.1 --count 10
```

Use `--count 0` or omit `--count` to send telemetry until interrupted.

## Bring-Up Order

1. On the laptop, build and start `uav-gcs` telemetry receiver.
2. On the Raspberry Pi, run `bash scripts/setup_rpi_dependencies.sh`.
3. On the Raspberry Pi, build `uav-onboard`.
4. On the Raspberry Pi, validate the camera with `camera_preview`.
5. On the Raspberry Pi, run `uav_onboard` and confirm telemetry appears in GCS.
