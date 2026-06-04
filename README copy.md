# GoPro Control Station

A complete remote control station for a GoPro Hero camera.

The system combines a two-axis motorised mount (so the camera can be aimed
without touching it) with full remote control of the camera itself
(shutter, mode, gallery, live HDMI preview). Everything is driven by a
single ESP32-S3 board running closed-loop FOC for the motors, BLE for
the camera commands, and an HTTPS web server for the user interface.

## Features

- Two-axis motorised mount with absolute angular position
- Magnetic encoders for accurate, drift-free positioning
- Soft-limits with automatic end-stop calibration
- Stall protection with auto-recovery (re-centres the motor on re-enable)
- BLE control of the GoPro (shutter, mode, sleep, keep-alive)
- Wi-Fi proxy to the GoPro REST API (gallery, thumbnails, downloads)
- Browser-based live HDMI preview via a USB-C capture card
- Self-signed HTTPS server with embedded certificate
- Persistent configuration and calibration in NVS
- Status LED strip for camera, motors, recording state and battery

The user interacts with the system through two parallel channels:

- **Physical panel** — five buttons (M1+, M1-, M2+, M2-, shutter) and a
  7-LED status strip.
- **Web interface** — served on the station's own Wi-Fi access point:
  control, gallery, live HDMI preview and configuration in a
  mobile-friendly UI.

## Repository layout

```
gopro-control-station/
├── platformio.ini              PlatformIO build configuration
├── README.md
├── LICENSE
│
├── src/                        Firmware sources (Arduino + ESP-IDF)
│   ├── bridge_main.cpp         setup() / loop() / status LEDs
│   ├── bridge_foc.cpp/.h       motor control, buttons, calibration
│   ├── bridge_current.cpp/.h   current sensing + stall detection
│   ├── bridge_ble.cpp/.h       NimBLE client (OpenGoPro)
│   ├── bridge_wifi.cpp/.h      Wi-Fi AP + STA + watchdog
│   ├── bridge_https.cpp/.h     TLS web server + REST proxy
│   ├── bridge_server.cpp/.h    plain HTTP (cert download only)
│   ├── bridge_config.cpp/.h    persistent configuration (NVS)
│   ├── bridge_led.h            LED constants
│   └── ssl_cert.h              embedded TLS certificate (auto-generated)
│
├── data/
│   └── index.html              single-page web UI (served from LittleFS)
│
├── certs/                      TLS certificate + regeneration toolchain
│   ├── bridge_cert.pem
│   ├── bridge_key.pem
│   ├── openssl.cnf
│   ├── generate.bat            Windows: regenerate cert + key + header
│   └── make_header.py          convert PEM into src/ssl_cert.h
│
├── hardware/
│   ├── cad/                    STL / STEP files for 3D-printed parts
│   │   ├── base.stl
│   │   ├── m1_arm.stl
│   │   ├── m2_arm.stl
│   │   ├── gopro_mount.stl
│   │   ├── electronics_box.stl
│   │   └── button_panel.stl
│   └── schematic/              electrical schematic + PCB
│       ├── schematic.pdf       rendered schematic for quick reference
│       └── kicad/              KiCad project sources
│
├── scripts/                    Python scripts for the report diagrams
│   ├── firmware_diagram.py
│   └── system_diagram.py
│
└── docs/
    ├── repport.tex             reproduction report (English)
    ├── mode_emploi.tex         end-user manual (French)
    └── figures/                generated diagrams + screenshots
```

## Quick start

### 1. Build and flash the firmware

```bash
# Firmware
pio run -e esp32-s3-bridge -t upload

# Web UI (LittleFS image)
pio run -e esp32-s3-bridge -t uploadfs

# Watch the boot log
pio device monitor -e esp32-s3-bridge
```

A clean first boot prints, in order: PCF8574 ready, FOC init OK x2,
Wi-Fi AP up, BLE connected to the camera, HTTPS server started, Wi-Fi
STA connected to the GoPro. See `docs/repport.tex` §5.5 for the
reference serial log.

### 2. Connect

1. Join the Wi-Fi network `GoPro-Bridge` (default password `gopro1234`).
2. Open `https://192.168.4.1` in a browser.
3. Accept the security warning, or download the certificate from the
   *Config* tab and install it to make the warning go away.

### 3. Adapt to your own GoPro

No source edit is required. After the first boot, go to the *Config* tab
in the web UI, replace the GoPro Wi-Fi SSID and password with the ones
printed on your camera (*Connections / Connect Device / GoPro App*),
*Save*, and reboot.

## Documentation

- **`docs/repport.tex`** — reproduction report (English). Step-by-step
  recipe to rebuild a unit from scratch: BOM, mechanical assembly, wiring
  diagram, GPIO map, firmware flash, validation, calibration, recovery.
- **`docs/mode_emploi.tex`** — end-user manual (French). Buttons, LEDs,
  web interface, calibration procedure.
- **`hardware/cad/`** — 3D-printable parts in STL format. Print
  settings and assembly order are documented in §3 of the report.
- **`hardware/schematic/`** — full electrical schematic. The pin
  assignments are also reproduced in §4 of the report and are
  hard-coded in the firmware (see `src/bridge_foc.cpp`).

## Regenerating the TLS certificate

The repository ships with a self-signed development certificate that
works out of the box. For any deployment where the local network is not
trusted, regenerate it:

```bash
cd certs
generate.bat            # Windows
# POSIX equivalent: see docs/repport.tex Appendix D
```

Then re-flash the firmware. Every client that already trusted the
previous certificate has to install the new one (web UI, *Config* tab).

## Hardware overview

- **MCU**: ESP32-S3 (`esp32-s3-devkitc-1`), 8 MB flash, no PSRAM.
- **Motors**: 2x BLDC (11 pole pairs), each driven by a DRV8313.
- **Encoders**: 2x AS5048A magnetic absolute encoders (SPI / HSPI bus).
- **Buttons**: 5x momentary, multiplexed through a PCF8574T (I2C).
- **Current sense**: 2x INA180A1 with 50 mOhm shunts, read on ADC2.
- **Indicators**: WS2815 strip, 7 LEDs.
- **Camera**: GoPro Hero (any model compatible with the OpenGoPro BLE
  specification) with the official Media Mod for HDMI output.
- **HDMI capture**: any UVC-class USB-C card (UGREEN 15389 is
  auto-selected by the web UI).

## License

This project is released under the [MIT License](LICENSE). You can use,
modify and redistribute it freely, including for commercial purposes,
provided that the copyright notice and the license text are kept.

## Acknowledgements

Built on top of [SimpleFOC](https://docs.simplefoc.com/),
[NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino),
[ESPAsyncWebServer](https://github.com/mathieucarbou/ESPAsyncWebServer),
and the [pioarduino fork](https://github.com/pioarduino/platform-espressif32)
of `platform-espressif32`. Camera control follows the
[OpenGoPro](https://gopro.github.io/OpenGoPro/) specification.
