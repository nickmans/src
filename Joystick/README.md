# Robot Joystick Bridge (Pi Hotspot + Web UI + Bluetooth Serial)

This project runs on a Raspberry Pi and bridges:

Phone browser -> Pi web server (WebSocket) -> Pi Bluetooth serial -> STM32

It is designed for joystick driving with robust safety defaults:

- Backend sends commands at a fixed rate (20 Hz default), including zero commands while idle.
- UI sends normalized joystick x/y only; backend computes angle/speed.
- If browser disconnects or joystick input times out, output is forced to zero.
- Bluetooth reconnect is automatic.
- Motion output remains zero until joystick mode is explicitly armed (UI button sends `joy`).
- Default transport is Ethernet UDP to STM32 (`192.168.1.10:9001`).

## Project Tree

    Joystick/
    ├── app/
    │   ├── __init__.py
    │   ├── bluetooth_link.py
    │   ├── command_mapper.py
    │   ├── config.py
    │   └── main.py
    ├── static/
    │   ├── app.js
    │   ├── index.html
    │   └── style.css
    ├── scripts/
    │   ├── setup_hotspot_nmcli.sh
    │   └── start_robot_joystick.sh
    ├── .env.example
    ├── requirements.txt
    ├── robot_joystick.service
    └── README.md

## Behavior and Conventions

### Direction

- x positive = right
- y positive = forward
- angle = atan2(x, y) in degrees
- normalized to integer 0..359
- 0 = forward/up, 90 = right, 180 = back/down, 270 = left

### Speed

- magnitude = sqrt(x^2 + y^2)
- clamped to 0..1
- deadzone applied in backend
- mapped to integer 0..MAX_SPEED (default 100)

### Safety

- Browser disconnect -> immediate dir 0, speed 0 command
- Input timeout -> forced zero output (while loop keeps streaming zeros)
- Backend keeps streaming at fixed rate even when idle

## Prerequisites on Pi

Install packages:

    sudo apt update
    sudo apt install -y python3 python3-venv python3-pip bluetooth bluez avahi-daemon network-manager

Note:
- avahi-daemon enables hostname access like http://raspberrypi.local:8000
- On Raspberry Pi OS, make sure NetworkManager is enabled if you use the hotspot script.

## Setup the Project

From this folder:

    cd /home/nickolas/ros2_ws/src/omni_src/Joystick
    python3 -m venv .venv
    source .venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt

Create your runtime config:

    cp .env.example .env

Edit .env values for your robot, especially:

- TRANSPORT (ethernet or bluetooth)
- ETH_TARGET_IP, ETH_TARGET_PORT
- BT_DEVICE
- BT_BAUDRATE
- CMD_SINGLE_LINE_TEMPLATE or CMD_SPEED_TEMPLATE
- UPDATE_HZ, DEADZONE, INPUT_TIMEOUT_S

## Bluetooth Serial Setup (RFCOMM)

If your STM32 Bluetooth module supports classic SPP, bind an RFCOMM device.

1) Find device MAC:

    bluetoothctl devices

2) Pair/trust/connect once (replace with your MAC):

    bluetoothctl
    power on
    agent on
    default-agent
    pair AA:BB:CC:DD:EE:FF
    trust AA:BB:CC:DD:EE:FF
    connect AA:BB:CC:DD:EE:FF
    quit

3) Bind RFCOMM channel (often channel 1):

    sudo rfcomm bind /dev/rfcomm0 AA:BB:CC:DD:EE:FF 1

4) Verify serial device:

    ls -l /dev/rfcomm0

Set BT_DEVICE=/dev/rfcomm0 in .env.

## Run Manually

    cd /home/nickolas/ros2_ws/src/omni_src/Joystick
    source .venv/bin/activate
    python -m app.main

Open from phone:

- http://<pi-ip>:8000
- or http://<pi-hostname>.local:8000

Health check:

    curl http://127.0.0.1:8000/health

## Hotspot/AP Setup on Pi (NetworkManager)

Use the included script:

    cd /home/nickolas/ros2_ws/src/omni_src/Joystick
    chmod +x scripts/setup_hotspot_nmcli.sh
    sudo ./scripts/setup_hotspot_nmcli.sh OMNI-BOT-PI5 MyStrongPass123 wlan0

This creates an AP profile:

- Connection name: robot-hotspot
- SSID: OMNI-BOT-PI5
- Password: MyStrongPass123
- AP IP: 10.42.0.1/24

Then connect phone to that SSID and open:

    http://10.42.0.1:8000

Useful checks:

    nmcli connection show robot-hotspot
    nmcli device status
    ip addr show wlan0

## Install as Systemd Service (Boot Startup)

1) Copy project to target path expected by sample service:

    sudo mkdir -p /home/pi/robot_joystick
    sudo rsync -a --delete /home/nickolas/ros2_ws/src/omni_src/Joystick/ /home/pi/robot_joystick/
    sudo chown -R pi:pi /home/pi/robot_joystick

2) Prepare runtime env there:

    sudo -u pi bash -lc 'cd /home/pi/robot_joystick && python3 -m venv .venv && source .venv/bin/activate && pip install -U pip && pip install -r requirements.txt'
    sudo -u pi bash -lc 'cd /home/pi/robot_joystick && cp .env.example .env'
    sudo chmod +x /home/pi/robot_joystick/scripts/start_robot_joystick.sh

3) Install service:

    sudo cp /home/pi/robot_joystick/robot_joystick.service /etc/systemd/system/robot_joystick.service
    sudo systemctl daemon-reload
    sudo systemctl enable robot_joystick.service
    sudo systemctl restart robot_joystick.service

4) Check status/logs:

    systemctl status robot_joystick.service --no-pager
    journalctl -u robot_joystick.service -f

## Command Format Configuration

In .env:

- Single line mode (recommended):

      CMD_USE_SINGLE_LINE=true
      CMD_SINGLE_LINE_TEMPLATE=dir {angle} speed {speed}

    This format is supported directly by CM7 joystick mode parser.

- Two line mode:

      CMD_USE_SINGLE_LINE=false
      CMD_DIR_TEMPLATE=dir {angle}
      CMD_SPEED_TEMPLATE=speed {speed}

Joystick mode command (sent once per button press):

    CMD_JOY=joy

If your STM32 command names change, update only these template values.

## Web UI Controls

- Enable Joystick Mode: sends joy once
- Emergency Stop: immediate zero command
- Big touch joystick with center snap-back
- Live display: angle, speed, websocket status, bluetooth status

## Notes for Robust Use

- Keep update rate at 20 Hz initially.
- For noisy hands near center, increase DEADZONE slightly.
- If serial link floods your parser, keep single-line mode enabled.
- If no movement is expected, backend still streams zero commands as a heartbeat.