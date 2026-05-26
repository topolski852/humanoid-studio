const { contextBridge, ipcRenderer } = require('electron')

contextBridge.exposeInMainWorld('electron', {
  platform: process.platform,
  quit: () => ipcRenderer.send('app-quit'),
  ensureDaemon: () => ipcRenderer.invoke('ensure-daemon'),
})
