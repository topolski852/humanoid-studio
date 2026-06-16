# Installation

This page walks through installing Humanoid Studio from a fresh checkout on Ubuntu. These instructions have been tested on Ubuntu 22.04 and 24.04.

---

## Prerequisites

### Python

Python 3.10 or newer is required. Ubuntu 22.04 ships Python 3.10; Ubuntu 24.04 ships Python 3.12. Verify:

```bash
python3 --version
```

### Node.js

Node.js 20.x is required. The project was developed with v20.20.2 via nvm. If you are using nvm:

```bash
export PATH="$HOME/.nvm/versions/node/v20.20.2/bin:$PATH"
node --version  # should show v20.x.x
```

If Node.js is not installed:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
```

### System packages

Install CAN utilities and the sudo tools used by the CAN adapter setup:

```bash
sudo apt-get install can-utils iproute2
```

Install the C++ build tools for the daemon:

```bash
sudo apt-get install build-essential
```

For the Flash Wizard (optional — only needed if reflashing ESC firmware):

```bash
sudo apt-get install gcc-arm-none-eabi make openocd
```

---

## Clone the repository

```bash
git clone https://github.com/topolski852/humanoid-studio.git
cd humanoid-studio
```

---

## Build the C++ daemon

The daemon is a standalone C++ binary that owns all SocketCAN interfaces. It must be built before running the app.

```bash
cd daemon
make -j$(nproc)
```

Expected output ends with:
```
g++ ... -o build/humanoid_daemon
```

The binary is placed at `daemon/build/humanoid_daemon`.

### Real-time scheduling (optional)

The daemon uses SCHED_FIFO for its control loop. On most Linux systems this requires either running as root or setting the `cap_sys_nice` capability on the binary:

```bash
sudo setcap cap_sys_nice+ep daemon/build/humanoid_daemon
```

Without this, the daemon falls back to SCHED_OTHER, which is sufficient for development but may introduce jitter under system load.

---

## Install Python dependencies

```bash
cd backend
pip install -r requirements.txt
```

All five packages are required. `uvicorn[standard]` includes WebSocket support and the uvloop event loop. `python-can` is retained for the Flash Wizard's direct CAN socket access.

Verify:

```bash
python3 -c "import fastapi, uvicorn, can, pydantic, websockets; print('OK')"
```

---

## Install Node dependencies

```bash
cd ../app
npm install
```

This installs Electron, React, Vite, Tailwind, and all other frontend dependencies into `app/node_modules/`. The install takes 1–2 minutes on first run.

---

## Run in development mode

Development mode runs the React UI on Vite's dev server (port 5173). The Electron process spawns the daemon and the Python backend automatically.

```bash
cd humanoid-studio/app
npm run dev
```

Electron starts and:
1. Spawns `daemon/build/humanoid_daemon --config configs/humanoid_lite.json`
2. Waits for the daemon to respond to PING on port 9001 (up to 10 s)
3. Spawns `python3 main.py` from the `backend/` directory
4. Polls `http://localhost:8765/devices` every 500 ms (up to 20 s)
5. Opens the app window

Expected backend log:
```
INFO:     Started server process [...]
INFO:     Waiting for application startup.
INFO:humanoid.daemon_client:DaemonClient: connected to daemon v1.0
INFO:     Application startup complete.
INFO:     Uvicorn running on http://localhost:8765 (Press CTRL+C to quit)
```

In dev mode, the Chrome DevTools panel opens automatically in a detached window. Close it if you do not need it.

### Launching from a VS Code terminal

VS Code sets `ELECTRON_RUN_AS_NODE=1` in its integrated terminal. The `npm run dev` script includes `ELECTRON_RUN_AS_NODE=` in its `cross-env` call to clear this variable. No action needed — this is already handled.

---

## Run in production mode

The production build packages the React app into static files and bundles them inside the Electron AppImage. The backend and daemon are spawned automatically from within the package.

Build:

```bash
cd humanoid-studio/app
npm run build
```

Output: `app/release/Humanoid Studio-0.1.0.AppImage`

Run:

```bash
./release/"Humanoid Studio-0.1.0.AppImage"
```

The AppImage bundles the frontend assets, Python backend source, robot config (`configs/humanoid_lite.json`), and the compiled C++ daemon binary. Python 3 and the Python packages must still be installed on the host — they are not bundled inside the AppImage.

---

## The GLIBCXX_3.4.29 error

On Ubuntu systems where Electron was installed via Snap, you may see:

```
/snap/core20/current/lib/x86_64-linux-gnu/libstdc++.so.6: version 'GLIBCXX_3.4.29' not found
```

This is a library version conflict between Snap's older bundled `libstdc++` and the version Electron requires. The fix is to ensure the system's own `libstdc++` is used instead of Snap's:

```bash
# Find the system libstdc++
find /usr/lib -name "libstdc++.so.6" 2>/dev/null

# Set LD_LIBRARY_PATH before launching
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
npm run dev
```

If you installed `node` via Snap instead of nvm, the cleanest fix is to switch to the nvm-managed Node.js installation:

```bash
nvm install 20
nvm use 20
```

---

## Verify the installation

With the daemon and backend running, confirm each layer:

```bash
# Daemon is alive
echo '{"type":"PING","id":"test"}' | nc -u -w1 127.0.0.1 9001

# Backend is responding
curl http://localhost:8765/devices

# Config loaded correctly (should show 22 joints)
curl http://localhost:8765/robot/config | python3 -m json.tool | grep -c '"joint_name"'

# Flash wizard endpoint is alive
curl http://localhost:8765/flash/status
```

For the WebSocket telemetry stream:

```bash
python3 -c "
import asyncio, websockets, json
async def t():
    async with websockets.connect('ws://localhost:8765/ws/telemetry') as ws:
        msg = json.loads(await ws.recv())
        print('connected:', msg.get('connected'))
asyncio.run(t())"
```

This should print `connected: False` if the robot is not yet connected, or `connected: True` if it is.
