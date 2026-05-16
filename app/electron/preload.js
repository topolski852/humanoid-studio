const { contextBridge, ipcRenderer } = require('electron')

contextBridge.exposeInMainWorld('electron', {
  platform: process.platform,
  // Future IPC hooks go here — renderer currently talks directly to the
  // FastAPI backend via fetch/WebSocket on localhost:8765
})
