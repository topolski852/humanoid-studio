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

## Install Python dependencies

```bash
cd backend
pip install fastapi "uvicorn[standard]" python-can "pydantic>=2.7.0" websockets
```

All five packages are required. `uvicorn[standard]` includes the WebSocket support and the uvloop event loop. `python-can` provides the SocketCAN interface used on Linux.

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

Development mode runs the React UI on Vite's dev server (port 5173) and spawns the Python backend from `backend/main.py`. Both are started by a single command.

Open two terminals:

**Terminal 1 — backend:**
```bash
cd humanoid-studio/backend
python3 main.py
```

Expected output:
```
INFO:     Started server process [...]
INFO:     Waiting for application startup.
INFO:humanoid.can_monitor:CanMonitor started
INFO:     Application startup complete.
INFO:     Uvicorn running on http://localhost:8765 (Press CTRL+C to quit)
```

**Terminal 2 — frontend:**
```bash
cd humanoid-studio/app
npm run dev
```

Electron opens automatically. It polls `http://localhost:8765/devices` every 500 ms and waits up to 20 seconds for the backend to respond before showing the window.

In dev mode, the Chrome DevTools panel opens automatically in a detached window. Close it if you do not need it.

---

## Run in production mode

The production build packages the React app into static files and bundles them inside the Electron AppImage. The backend is spawned automatically from within the package — you do not need a separate terminal.

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

The AppImage is self-contained. It embeds the frontend assets and the Python backend source. Python and all Python packages must still be installed on the host system — they are not bundled inside the AppImage.

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

With the backend running, confirm each layer:

```bash
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
