// 初始化地圖
const map = L.map('map').setView([0, 0], 2);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);

// 建立一個 marker，之後會移動它
const marker = L.marker([0, 0]).addTo(map);

// 建立 WebSocket 連線
const ws = new WebSocket(`ws://${location.host}`);
ws.onopen = () => console.log('WebSocket connected');
ws.onmessage = event => {
  const { lat, lon } = JSON.parse(event.data);
  // 更新 marker 與地圖中心
  marker.setLatLng([lat, lon]);
  map.setView([lat, lon], 16);
};
ws.onerror = err => console.error('WebSocket error', err);