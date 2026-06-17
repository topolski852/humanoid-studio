# Installation

These instructions have been tested on Ubuntu 22.04 and 24.04.

---

## Install from release

Download the AppImage from the [releases page](https://github.com/topolski852/humanoid-studio/releases/latest). The AppImage bundles the frontend, Python backend, robot config, and C++ daemon binary — no compilation needed.

### 1. Install system dependencies

```bash
sudo apt-get install libfuse2 can-utils iproute2
```

`libfuse2` is required to run AppImages on Ubuntu 22.04+. `can-utils` and `iproute2` are needed for CAN adapter setup.

For the Flash Wizard (optional — only needed if reflashing ESC firmware):

```bash
sudo apt-get install gcc-arm-none-eabi make openocd
```

### 2. Install Python dependencies

```bash
pip install fastapi "uvicorn[standard]" python-can "pydantic>=2.7" websockets
```

### 3. Run the AppImage

```bash
chmod +x "Humanoid Studio-0.1.0.AppImage"
./"Humanoid Studio-0.1.0.AppImage"
```

The app will start, automatically spawn the daemon and backend, and open the UI. If you see a `GLIBCXX_3.4.29 not found` error, see [the GLIBCXX error](#the-glibcxx_3429-error) below.

---

## Developer install

Clone the repo and build from source to modify the code, rebuild the daemon, or contribute to the project.

### Prerequisites

**Python** — 3.10 or newer. Ubuntu 22.04 ships Python 3.10; Ubuntu 24.04 ships Python 3.12.

```bash
python3 --version
```

**Node.js** — 20.x required. Install via nvm:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
```

**System packages:**

```bash
sudo apt-get install build-essential can-utils iproute2
```

For the Flash Wizard (optional):

```bash
sudo apt-get install gcc-arm-none-eabi make openocd
```

### Clone the repository

```bash
git clone https://github.com/topolski852/humanoid-studio.git
cd humanoid-studio
```

### Build the C++ daemon

```bash
cd daemon
make -j$(nproc)
```

The binary is placed at `daemon/build/humanoid_daemon`.

**Real-time scheduling (optional):** The daemon uses SCHED_FIFO for its control loop. Grant the capability without running as root:

```bash
sudo setcap cap_sys_nice+ep daemon/build/humanoid_daemon
```

Without this, the daemon falls back to SCHED_OTHER — sufficient for development but may introduce jitter under load.

### Install Python dependencies

```bash
cd backend
pip install -r requirements.txt
```

Verify:

```bash
python3 -c "import fastapi, uvicorn, can, pydantic, websockets; print('OK')"
```

### Install Node dependencies

```bash
cd ../app
npm install
```

Takes 1–2 minutes on first run.

### Run in development mode

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
INFO:humanoid.daemon_client:DaemonClient: connected to daemon v1.0
INFO:     Application startup complete.
INFO:     Uvicorn running on http://localhost:8765 (Press CTRL+C to quit)
```

> **VS Code terminal:** VS Code sets `ELECTRON_RUN_AS_NODE=1`. The `npm run dev` script clears this automatically via `cross-env` — no action needed.

### Build the AppImage

```bash
cd humanoid-studio/app
npm run build
```

Output: `app/release/Humanoid Studio-0.1.0.AppImage`

The AppImage bundles the frontend, backend source, config, and daemon binary. Python 3 and the Python packages must still be installed on the host.

### Verify the installation

With the app running, confirm each layer from a terminal:

```bash
# Daemon is alive
echo '{"type":"PING","id":"test"}' | nc -u -w1 127.0.0.1 9001

# Backend is responding
curl http://localhost:8765/devices

# Config loaded correctly (should show 22 joints)
curl http://localhost:8765/robot/config | python3 -m json.tool | grep -c '"joint_name"'
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

---

## The GLIBCXX_3.4.29 error

On Ubuntu systems where Node or Electron was installed via Snap, you may see:

```
/snap/core20/current/lib/x86_64-linux-gnu/libstdc++.so.6: version 'GLIBCXX_3.4.29' not found
```

**Fix 1 — use nvm instead of snap node:**

```bash
nvm install 20
nvm use 20
```

**Fix 2 — override LD_LIBRARY_PATH:**

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

Then relaunch the app or run `npm run dev`.
