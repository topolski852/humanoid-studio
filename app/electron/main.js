const { app, BrowserWindow, dialog, shell } = require('electron')
const path = require('path')
const { spawn } = require('child_process')
const http = require('http')
const dgram = require('dgram')

const IS_DEV = !app.isPackaged
const BACKEND_PORT = 8765
const DAEMON_CMD_PORT = 9001

const BACKEND_DIR = IS_DEV
  ? path.resolve(__dirname, '../../backend')
  : path.join(process.resourcesPath, 'backend')

const DAEMON_BINARY = IS_DEV
  ? path.resolve(__dirname, '../../daemon/build/humanoid_daemon')
  : path.join(process.resourcesPath, 'daemon/humanoid_daemon')

const DAEMON_CONFIG = IS_DEV
  ? path.resolve(__dirname, '../../configs/humanoid_lite.json')
  : path.join(process.resourcesPath, 'configs/humanoid_lite.json')

let mainWindow = null
let backendProcess = null
let daemonProcess = null
app.isQuitting = false

// ── Poll daemon port 9001 until a PING returns PONG ──────────────────────────
function waitForDaemon(maxAttempts = 20) {
  return new Promise((resolve, reject) => {
    let attempt = 0

    function check() {
      attempt++
      const sock = dgram.createSocket('udp4')
      const msg = Buffer.from(JSON.stringify({ type: 'PING', id: 'startup' }))
      let done = false

      const timer = setTimeout(() => {
        if (!done) { done = true; try { sock.close() } catch (_) {} }
        if (attempt >= maxAttempts) {
          reject(new Error('Daemon did not respond to PING within 10 s'))
        } else {
          setTimeout(check, 500)
        }
      }, 500)

      sock.on('message', () => {
        if (!done) {
          done = true
          clearTimeout(timer)
          try { sock.close() } catch (_) {}
          resolve()
        }
      })

      sock.on('error', () => {
        if (!done) {
          done = true
          clearTimeout(timer)
          try { sock.close() } catch (_) {}
          if (attempt >= maxAttempts) {
            reject(new Error('Daemon UDP socket error'))
          } else {
            setTimeout(check, 500)
          }
        }
      })

      sock.bind(0, () => {
        sock.send(msg, 0, msg.length, DAEMON_CMD_PORT, '127.0.0.1')
      })
    }

    setTimeout(check, 500)
  })
}

// ── Spawn the C++ daemon ──────────────────────────────────────────────────────
function startDaemon() {
  return new Promise((resolve, reject) => {
    daemonProcess = spawn(DAEMON_BINARY, ['--config', DAEMON_CONFIG, '--rt-prio', '0'], {
      stdio: ['ignore', 'pipe', 'pipe'],
    })

    daemonProcess.stdout.on('data', (d) => process.stdout.write(`[daemon] ${d}`))
    daemonProcess.stderr.on('data', (d) => process.stderr.write(`[daemon] ${d}`))

    let started = false

    daemonProcess.on('error', (err) => {
      if (!started) reject(new Error(`Failed to launch daemon: ${err.message}`))
    })

    daemonProcess.on('exit', (code, signal) => {
      if (!started) {
        reject(new Error(`Daemon exited before PING (code ${code ?? signal})`))
      } else if (!app.isQuitting) {
        console.error(`[main] Daemon exited unexpectedly (code ${code ?? signal})`)
      }
    })

    waitForDaemon()
      .then(() => { started = true; resolve() })
      .catch((err) => { if (!started) reject(err) })
  })
}

// ── Poll until the backend is accepting HTTP connections ──────────────────────
function waitForBackend(maxAttempts = 40) {
  return new Promise((resolve, reject) => {
    let attempt = 0

    function check() {
      attempt++
      const req = http.get(`http://localhost:${BACKEND_PORT}/devices`, (res) => {
        res.resume()
        resolve()
      })
      req.on('error', () => {
        if (attempt >= maxAttempts) {
          reject(new Error('Backend did not become ready within 20 seconds'))
        } else {
          setTimeout(check, 500)
        }
      })
      req.setTimeout(1000, () => {
        req.destroy()
        if (attempt >= maxAttempts) {
          reject(new Error('Backend connection timed out after 20 s'))
        } else {
          setTimeout(check, 500)
        }
      })
    }

    // Give Python a moment to start its import phase
    setTimeout(check, 1500)
  })
}

// ── Spawn python3 backend/main.py ─────────────────────────────────────────────
function startBackend() {
  return new Promise((resolve, reject) => {
    const python = process.platform === 'win32' ? 'python' : 'python3'
    backendProcess = spawn(python, ['main.py'], {
      cwd: BACKEND_DIR,
      stdio: ['ignore', 'pipe', 'pipe'],
      env: { ...process.env, PYTHONUNBUFFERED: '1' },
    })

    backendProcess.stdout.on('data', (d) => process.stdout.write(`[backend] ${d}`))
    backendProcess.stderr.on('data', (d) => process.stderr.write(`[backend] ${d}`))

    let started = false
    backendProcess.on('error', (err) => {
      if (!started) {
        reject(new Error(`Failed to launch Python: ${err.message}\n\nCheck that python3 is on PATH and backend dependencies are installed.`))
      } else {
        showBackendCrash(null, err.message)
      }
    })

    backendProcess.on('exit', (code, signal) => {
      if (!started) {
        reject(new Error(`Backend process exited before becoming ready (code ${code ?? signal})`))
        return
      }
      if (!app.isQuitting && code !== 0 && code !== null) {
        showBackendCrash(code)
      }
    })

    waitForBackend()
      .then(() => { started = true; resolve() })
      .catch((err) => { if (!started) reject(err) })
  })
}

function showBackendCrash(code, reason) {
  const detail = reason ?? `Exit code: ${code}`
  if (mainWindow) {
    dialog.showMessageBox(mainWindow, {
      type: 'error',
      title: 'Backend Process Crashed',
      message: 'The Python backend stopped unexpectedly.',
      detail: `${detail}\n\nSave your work and restart Humanoid Studio.`,
      buttons: ['Restart', 'Quit'],
      defaultId: 0,
    }).then(({ response }) => {
      if (response === 0) {
        app.relaunch()
        app.exit(0)
      } else {
        app.quit()
      }
    })
  }
}

// ── Create the renderer window ────────────────────────────────────────────────
function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 960,
    minHeight: 640,
    backgroundColor: '#0f1117',
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js'),
      webSecurity: true,
    },
    titleBarStyle: process.platform === 'darwin' ? 'hiddenInset' : 'default',
    show: false,
    title: 'Humanoid Studio',
    icon: path.join(__dirname, '../public/icon.png'),
  })

  const url = IS_DEV
    ? 'http://localhost:5173'
    : `file://${path.join(__dirname, '../dist/index.html')}`

  mainWindow.loadURL(url)

  if (IS_DEV) {
    mainWindow.webContents.openDevTools({ mode: 'detach' })
  }

  mainWindow.webContents.setWindowOpenHandler(({ url }) => {
    shell.openExternal(url)
    return { action: 'deny' }
  })

  mainWindow.once('ready-to-show', () => mainWindow.show())
  mainWindow.on('closed', () => { mainWindow = null })
}

// ── App lifecycle ─────────────────────────────────────────────────────────────
app.whenReady().then(async () => {
  try {
    console.log('[main] Starting C++ daemon from', DAEMON_BINARY)
    await startDaemon()
    console.log('[main] Daemon ready — starting Python backend from', BACKEND_DIR)
    await startBackend()
    console.log('[main] Backend ready — creating window')
    createWindow()
  } catch (err) {
    dialog.showErrorBox('Failed to Start Humanoid Studio', err.message)
    app.quit()
  }
})

app.on('before-quit', () => {
  app.isQuitting = true
  if (backendProcess && !backendProcess.killed) {
    console.log('[main] Sending SIGTERM to backend...')
    backendProcess.kill('SIGTERM')
    setTimeout(() => {
      if (backendProcess && !backendProcess.killed) {
        console.log('[main] Backend did not exit, sending SIGKILL')
        backendProcess.kill('SIGKILL')
      }
    }, 3000)
  }
  if (daemonProcess && !daemonProcess.killed) {
    console.log('[main] Sending SIGTERM to daemon...')
    daemonProcess.kill('SIGTERM')
    setTimeout(() => {
      if (daemonProcess && !daemonProcess.killed) {
        console.log('[main] Daemon did not exit, sending SIGKILL')
        daemonProcess.kill('SIGKILL')
      }
    }, 3000)
  }
})

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit()
})

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow()
})
