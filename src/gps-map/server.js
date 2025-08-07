// server.js
const express = require('express');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const WebSocket = require('ws');

// --- 1) 啟動 Express 服務靜態路由 public/
const app = express();
app.use(express.static('public'));
const server = app.listen(3000, () => {
  console.log('HTTP server running at http://localhost:3000');
});

// --- 2) 啟動 WebSocket server，用於推送 GPS 資料
const wss = new WebSocket.Server({ server });

// 當有新的 WS 連線時，可選擇記錄或立即傳送最新位置
wss.on('connection', ws => {
  console.log('WebSocket client connected');
});

// --- 3) 打開 GPS 的串口（請改成你的 port 名稱）
const port = new SerialPort({
  path: '/dev/ttyUSB0',
  baudRate: 38400
});
const parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }));

// --- 4) 解析 NMEA，拿到 lat/lon 並廣播給所有 WebSocket 客戶端
parser.on('data', line => {
  if (!line.startsWith('$GPGGA') && !line.startsWith('$GPRMC')) return;
  try {
    // 簡易用字串解析，取出 lat/lon
    const parts = line.split(',');
    let lat, lon;
    if (parts[0] === '$GPGGA') {
      lat = parseFloat(parts[2]) / 100.0;
      lon = parseFloat(parts[4]) / 100.0;
      if (parts[3] === 'S') lat = -lat;
      if (parts[5] === 'W') lon = -lon;
    } else if (parts[0] === '$GPRMC') {
      lat = parseFloat(parts[3]) / 100.0;
      lon = parseFloat(parts[5]) / 100.0;
      if (parts[4] === 'S') lat = -lat;
      if (parts[6] === 'W') lon = -lon;
    }
    if (lat && lon) {
      const msg = JSON.stringify({ lat, lon });
      wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(msg);
        }
      });
    }
  } catch (e) {
    console.error('NMEA parse error', e);
  }
});