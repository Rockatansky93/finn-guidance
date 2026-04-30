# FINN Guidance - Linux Installation Guide

This guide covers installing and running FINN Guidance on a Linux field laptop
or tablet. It is written for Ubuntu/Debian-based systems first, because they
are the most common choice for field hardware, but the Rust application itself
is portable to other Linux distributions.

The PC application has no Windows-only dependency. The main Linux differences
are serial port names, serial device permissions, and GUI/OpenGL system
packages.

---

## 1. Recommended Linux setup

### Tested target

| Item | Recommendation |
|------|----------------|
| OS | Ubuntu 22.04/24.04 LTS, Linux Mint, Debian 12, or similar |
| CPU | x86_64 laptop/tablet |
| RAM | 8 GB minimum recommended |
| Storage | 10 GB free for Rust build cache and PlatformIO |
| Display | Touchscreen helpful, not required |
| USB | Two reliable USB ports for sensor ESP32 and motor ESP32 |

### Hardware used by FINN Guidance

| Device | Linux device name usually looks like |
|--------|--------------------------------------|
| Sensor ESP32 | `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `/dev/ttyACM0` |
| Motor ESP32 | `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `/dev/ttyACM1` |
| Direct GPS USB serial, if used | `/dev/ttyUSB*` or `/dev/ttyACM*` |

The app auto-detects serial devices, so exact port names usually do not need to
be configured manually.

---

## 2. Install Linux system packages

Update the package list first:

```bash
sudo apt update
```

Install the build tools and GUI dependencies:

```bash
sudo apt install \
  build-essential \
  curl \
  git \
  pkg-config \
  libudev-dev \
  libx11-dev \
  libxcb1-dev \
  libxcb-render0-dev \
  libxcb-shape0-dev \
  libxcb-xfixes0-dev \
  libxi-dev \
  libxkbcommon-dev \
  libgl1-mesa-dev
```

Install Python tooling for PlatformIO firmware flashing:

```bash
sudo apt install python3 python3-pip python3-venv
```

Optional but useful serial debugging tools:

```bash
sudo apt install minicom screen
```

---

## 3. Install Rust

Install Rust using rustup:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

When prompted, choose the default installation.

Load Rust into the current terminal:

```bash
source "$HOME/.cargo/env"
```

Verify the install:

```bash
rustup --version
cargo --version
rustc --version
```

Use the stable toolchain:

```bash
rustup default stable
```

---

## 4. Allow serial port access

Linux blocks normal users from opening serial devices unless they are in the
right group.

Add your user to the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and log back in. A reboot is fine too.

Verify group membership after logging back in:

```bash
groups
```

You should see `dialout` in the list.

To see connected serial devices:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

To watch devices appear as you plug them in:

```bash
dmesg -w
```

Press `Ctrl+C` to stop watching.

---

## 5. Clone and build FINN Guidance

Clone the repository:

```bash
git clone https://github.com/Rockatansky93/finn-guidance.git
cd finn-guidance
```

Build the PC guidance application:

```bash
cargo build --release -p finn-guidance-pc
```

Run a compile check during development:

```bash
cargo check
```

Run tests:

```bash
cargo test
```

Note: if any steering-control tests fail, treat that as a controller behaviour
issue to investigate before field use. It is not normally a Linux installation
problem.

---

## 6. Run the PC application

From the repository root:

```bash
cargo run --release -p finn-guidance-pc
```

The application will:

- scan available serial ports
- find the GPS/sensor stream
- find the motor ESP32 on the second serial port
- create `data/coverage.db` if needed
- create `logs/` if steer telemetry logging is enabled

Run from the repository root so relative paths like `data/coverage.db` resolve
correctly.

---

## 7. Install PlatformIO for ESP32 firmware

The ESP32 firmware uses Arduino/PlatformIO.

Install PlatformIO:

```bash
python3 -m pip install --user platformio
```

Make sure the local Python bin directory is on your path:

```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
source "$HOME/.bashrc"
```

Verify:

```bash
pio --version
```

The first PlatformIO build downloads the ESP32 Arduino framework. This can take
several minutes.

---

## 8. Flash the ESP32 firmware

### 8.1 Flash the Sensor Module

Plug in only the sensor ESP32.

```bash
cd firmware-sensor-pio
pio run --target upload
```

Open the serial monitor:

```bash
pio device monitor
```

You should see sensor output such as:

```text
$FINNWAS,2048,1650*4A
$FINNIMU,1.2,-0.5,182.3,3,3,2,1*3B
$GPGGA,...
$FINNHB,2000*1C
```

Press `Ctrl+C` to exit the monitor.

### 8.2 Flash the Motor Controller

Unplug the sensor ESP32. Plug in only the motor ESP32.

```bash
cd ../firmware-motor-pio
pio run --target upload
```

Open the serial monitor:

```bash
pio device monitor
```

You should see motor status output:

```text
$FINNMTR,0,0,200*XX
$FINNMTR,0,0,400*XX
```

The motor should stay disabled until it receives valid steer commands from the
PC application.

### 8.3 Specify a port manually

If PlatformIO chooses the wrong port:

```bash
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0
```

Use this to list likely ports:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

---

## 9. Optional: stable serial names with udev

Linux may assign different `/dev/ttyUSB*` numbers depending on plug-in order.
FINN Guidance auto-detects ports, so this is optional. Stable names are still
useful for debugging and manual flashing.

Plug in a device and inspect it:

```bash
udevadm info -a -n /dev/ttyUSB0 | less
```

Look for attributes such as:

- `idVendor`
- `idProduct`
- `serial`
- `manufacturer`

Create a rules file:

```bash
sudo nano /etc/udev/rules.d/99-finn-guidance.rules
```

Example rules:

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="finn-sensor", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="finn-motor", GROUP="dialout", MODE="0660"
```

Reload rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the ESP32s, then check:

```bash
ls -l /dev/finn-*
```

Different ESP32 boards use different USB serial chips, commonly CP210x or
CH340, so adjust vendor/product values for your actual boards.

---

## 10. Bench test on Linux

1. Plug in both ESP32 modules.
2. Confirm Linux can see them:

   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
   ```

3. Run the PC app:

   ```bash
   cargo run --release -p finn-guidance-pc
   ```

4. Check the app status:
   - GPS fix appears in the status bar
   - WAS/IMU readings appear on the Setup page
   - Motor ESP32 shows connected
   - Motor test controls can command the motor

5. Calibrate WAS:
   - Set centre
   - Set left lock
   - Set right lock
   - Confirm calibrated angle is near 0 degrees when wheels are straight

6. Confirm motor direction:
   - Send a small positive motor test command
   - If the wheels steer the wrong way, use the motor direction invert control

---

## 11. Running on boot

For a dedicated tractor laptop, the goal is: power on, no login prompt, app
starts automatically, app restarts itself if it crashes. This section covers
the full setup — autologin, then a systemd user service, plus a simpler
launch-script fallback.

### 11.1 Autologin

GNOME (default Ubuntu desktop) handles autologin through GDM. Edit the GDM
config:

```bash
sudo nano /etc/gdm3/custom.conf
```

Under the `[daemon]` section, add or uncomment:

```ini
[daemon]
AutomaticLoginEnable=true
AutomaticLogin=tom
```

Replace `tom` with your actual Linux username (run `whoami` if you're not
sure). Save, exit, and reboot to test — the laptop should go straight from
boot splash to desktop with no password prompt.

If you're on a lighter desktop (XFCE/LightDM), the file is
`/etc/lightdm/lightdm.conf` instead, and the lines are:

```ini
[Seat:*]
autologin-user=tom
autologin-user-timeout=0
```

### 11.2 Systemd user service

A systemd user service is the cleanest way to autostart `finn-guidance` after
the graphical session is ready, with automatic restart on crash. User services
(rather than system services) are preferred here because the egui window needs
a live graphical session to attach to.

Create the service file:

```bash
mkdir -p "$HOME/.config/systemd/user"
nano "$HOME/.config/systemd/user/finn-guidance.service"
```

Paste this (adjust paths to match your setup):

```ini
[Unit]
Description=FINN Guidance
After=graphical-session.target
PartOf=graphical-session.target

[Service]
Type=simple
WorkingDirectory=/home/tom/finn-guidance
ExecStart=/home/tom/finn-guidance/target/release/finn-guidance-pc
Restart=on-failure
RestartSec=5
Environment="RUST_LOG=info"
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=graphical-session.target
```

Notes:

- `Restart=on-failure` with `RestartSec=5` means if the app crashes (panic,
  serial error, etc.) it will come back automatically after 5 seconds.
- `WorkingDirectory` ensures relative paths like `data/coverage.db` resolve
  correctly.
- `RUST_LOG=info` matches the typical development log level. Bump to `debug`
  for more verbose output.
- `StandardOutput=journal` sends app stdout/stderr to the systemd journal,
  which persists across reboots.

The service expects a release build to exist at
`target/release/finn-guidance-pc`. Build it first if you haven't:

```bash
cd "$HOME/finn-guidance"
cargo build --release -p finn-guidance-pc
```

Enable and start the service:

```bash
systemctl --user daemon-reload
systemctl --user enable finn-guidance.service
systemctl --user start finn-guidance.service
```

Check status:

```bash
systemctl --user status finn-guidance.service
```

Live-tail the logs:

```bash
journalctl --user -u finn-guidance.service -f
```

View logs from the current boot:

```bash
journalctl --user -u finn-guidance.service -b
```

### 11.3 Enable lingering

By default, user services stop when the user logs out and don't start until
the user logs in. With autologin (section 11.1) that's fine in normal
operation, but to make user services start at boot regardless of login
state — useful for recovery and edge cases — enable lingering:

```bash
sudo loginctl enable-linger tom
```

Replace `tom` with your username. This is generally a good idea for a
dedicated appliance machine.

### 11.4 Test the full boot flow

Reboot the laptop. You should see:

1. BIOS/POST
2. Ubuntu boot splash
3. Desktop appears (no login prompt)
4. `finn-guidance` window opens within a few seconds

If the app doesn't appear, check:

```bash
systemctl --user status finn-guidance.service
journalctl --user -u finn-guidance.service -b
```

### 11.5 Alternative: simple launch script

If you don't want a systemd service — for example during early development
where you want to start and stop the app manually — a launch script is
simpler.

Create the script:

```bash
nano "$HOME/finn-guidance/run-finn-guidance.sh"
```

Add:

```bash
#!/usr/bin/env bash
set -e
cd "$HOME/finn-guidance"
source "$HOME/.cargo/env"
cargo run --release -p finn-guidance-pc
```

Make it executable:

```bash
chmod +x "$HOME/finn-guidance/run-finn-guidance.sh"
```

Create a desktop launcher that runs:

```bash
/home/YOUR_USER/finn-guidance/run-finn-guidance.sh
```

Replace `YOUR_USER` with your Linux username.

This approach uses `cargo run` rather than the compiled binary directly, so it
will rebuild on changes — handy during development, slower at startup.

---

## 12. Troubleshooting

### `Permission denied` opening `/dev/ttyUSB0`

Your user is probably not in the `dialout` group.

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in.

### No serial ports found

Check the USB cable. Many USB cables are charge-only and have no data wires.

Check kernel messages:

```bash
dmesg | tail -50
```

Try another USB port.

### ESP32 appears, then disappears

This is often a weak cable, loose connector, or power issue. Use short, good
quality USB cables and secure them against vibration in the tractor cab.

### `cargo build` fails with missing X11, XCB, GL, or udev libraries

Reinstall the Linux dependency packages:

```bash
sudo apt install \
  build-essential pkg-config libudev-dev \
  libx11-dev libxcb1-dev libxcb-render0-dev \
  libxcb-shape0-dev libxcb-xfixes0-dev \
  libxi-dev libxkbcommon-dev libgl1-mesa-dev
```

### App opens but the window is blank or crashes

This is usually a graphics driver/OpenGL issue.

Try:

```bash
sudo apt install mesa-utils
glxinfo | grep "OpenGL"
```

Make sure hardware acceleration is available. On older laptops, update Mesa or
try a newer Ubuntu/Mint release.

### PlatformIO cannot upload

Check that no serial monitor is already using the port. Close `pio device
monitor`, `screen`, `minicom`, or the FINN Guidance app before flashing.

Specify the port manually:

```bash
pio run --target upload --upload-port /dev/ttyUSB0
```

### GPS shows no fix

The GPS needs a clear sky view. Indoor testing usually shows NMEA output but no
position fix. Move the antenna outside and wait for lock.

### Auto-steer button is disabled

The app requires:

- AB line loaded or created
- motor ESP32 connected
- WAS centre calibrated
- WAS left and right lock calibrated

Check each item on the Setup page.

### Auto-steer disengages with GPS or WAS timeout

Secure the USB connections first. Field vibration can create brief disconnects.

GPS timeout normally means no fresh fix is arriving. WAS timeout means the
sensor ESP32 is not sending fresh `$FINNWAS` messages.

---

## 13. Useful commands

List USB devices:

```bash
lsusb
```

List serial devices:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Watch USB/serial kernel messages:

```bash
dmesg -w
```

Monitor a serial port with PlatformIO:

```bash
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

Monitor a serial port with screen:

```bash
screen /dev/ttyUSB0 115200
```

Exit `screen` with `Ctrl+A`, then `K`, then `Y`.

Run FINN Guidance:

```bash
cd "$HOME/finn-guidance"
cargo run --release -p finn-guidance-pc
```

