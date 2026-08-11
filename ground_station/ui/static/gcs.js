/* gcs.js
 * -------
 * Frontend logic for the UAV ground control station (FYP report, Ch. 4.3.2):
 *  - Leaflet map: UAV live position marker, mission-center selection
 *  - Socket.IO: telemetry, environmental data, video feed + detection boxes
 *  - Mission control panel: generate/upload/launch mission, RTL, emergency land
 */

// -------------------------------------------------------------------- //
// Map setup
// -------------------------------------------------------------------- //
const map = L.map('map').setView([33.8547, 35.8623], 14); // default: Lebanon
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);

const uavIcon = L.divIcon({ className: 'uav-marker', html: '&#9992;', iconSize: [24, 24] });
let uavMarker = null;
let missionCenterMarker = null;
let missionCenter = null;
let currentMissionId = null;

map.on('click', (e) => {
  missionCenter = e.latlng;
  if (missionCenterMarker) {
    missionCenterMarker.setLatLng(e.latlng);
  } else {
    missionCenterMarker = L.marker(e.latlng, { title: 'Mission center' }).addTo(map);
  }
});

// -------------------------------------------------------------------- //
// Socket.IO
// -------------------------------------------------------------------- //
const socket = io();

socket.on('telemetry:snapshot', (state) => {
  if (state.gps && state.gps.lat) updateUavPosition(state.gps);
  if (state.telemetry) updateTelemetryPanel(state.telemetry);
  if (state.env) updateEnvPanel(state.env);
});

socket.on('telemetry:gps', updateUavPosition);
socket.on('telemetry:state', updateTelemetryPanel);
socket.on('telemetry:battery', (data) => {
  document.getElementById('tm-battery').textContent =
    data.percentage != null ? `${Math.round(data.percentage * 100)}%` : '--';
});
socket.on('telemetry:env', updateEnvPanel);
socket.on('mission:status', (status) => {
  logStatus(`[${status.state}] ${status.detail}`);
  if (status.state === 'launched') {
    document.getElementById('btn-launch-mission').disabled = true;
  }
});
socket.on('video:frame', (data) => {
  document.getElementById('video-feed').src = 'data:image/jpeg;base64,' + data.raw;
  drawDetections(data.human_boxes, data.fire_boxes);
});

function updateUavPosition(gps) {
  if (!gps || gps.lat == null) return;
  const latlng = [gps.lat, gps.lon];
  if (uavMarker) {
    uavMarker.setLatLng(latlng);
  } else {
    uavMarker = L.marker(latlng, { icon: uavIcon }).addTo(map);
  }
  document.getElementById('tm-alt').textContent = gps.alt != null ? `${gps.alt.toFixed(1)} m` : '--';
  const sats = gps.satellites != null ? gps.satellites : '--';
  const hdop = gps.hdop != null ? gps.hdop.toFixed(2) : '--';
  document.getElementById('tm-gps').textContent = `${sats} sats / HDOP ${hdop}`;
}

function updateTelemetryPanel(t) {
  if (t.mode !== undefined) document.getElementById('tm-mode').textContent = t.mode || '--';
  if (t.armed !== undefined) document.getElementById('tm-armed').textContent = t.armed ? 'ARMED' : 'DISARMED';
}

function updateEnvPanel(env) {
  document.getElementById('tm-temp').textContent =
    env.temperature_c != null ? `${env.temperature_c.toFixed(1)} \u00b0C` : '--';
  document.getElementById('tm-humidity').textContent =
    env.humidity_pct != null ? `${env.humidity_pct.toFixed(1)} %` : '--';
  document.getElementById('tm-aqi').textContent =
    env.air_quality_index != null ? env.air_quality_index : '--';
}

// -------------------------------------------------------------------- //
// Video / detection overlay
// -------------------------------------------------------------------- //
let displayMode = 'live';
const overlayCanvas = document.getElementById('detection-overlay');
const overlayCtx = overlayCanvas.getContext('2d');

document.querySelectorAll('.mode-btn').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.mode-btn').forEach((b) => b.classList.remove('active'));
    btn.classList.add('active');
    displayMode = btn.dataset.mode;
    socket.emit('video:set_mode', { mode: displayMode });
  });
});

function drawDetections(humanBoxes, fireBoxes) {
  const img = document.getElementById('video-feed');
  overlayCanvas.width = img.clientWidth;
  overlayCanvas.height = img.clientHeight;
  overlayCtx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);

  if (displayMode === 'human' || displayMode === 'live') {
    drawBoxes(humanBoxes, '#00e676', 'person');
  }
  if (displayMode === 'fire' || displayMode === 'live') {
    drawBoxes(fireBoxes, '#ff1744', 'fire');
  }
}

function drawBoxes(boxes, color, defaultLabel) {
  overlayCtx.strokeStyle = color;
  overlayCtx.lineWidth = 2;
  overlayCtx.font = '14px sans-serif';
  overlayCtx.fillStyle = color;

  (boxes || []).forEach((b) => {
    const x = b.x * overlayCanvas.width;
    const y = b.y * overlayCanvas.height;
    const w = b.w * overlayCanvas.width;
    const h = b.h * overlayCanvas.height;
    overlayCtx.strokeRect(x, y, w, h);
    overlayCtx.fillText(`${b.label || defaultLabel} ${(b.confidence * 100).toFixed(0)}%`, x, y - 4);
  });
}

// -------------------------------------------------------------------- //
// Mission control
// -------------------------------------------------------------------- //
document.getElementById('btn-generate-mission').addEventListener('click', async () => {
  if (!missionCenter) {
    alert('Click the map first to set the mission center.');
    return;
  }
  const payload = {
    center_lat: missionCenter.lat,
    center_lon: missionCenter.lng,
    scan_radius_m: Number(document.getElementById('mp-radius').value),
    cruise_altitude_m: Number(document.getElementById('mp-cruise').value),
    scan_altitude_m: Number(document.getElementById('mp-scan-alt').value),
  };
  const resp = await fetch('/api/mission/generate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(payload),
  });
  const mission = await resp.json();
  if (mission.error) {
    logStatus(`Error: ${mission.error}`);
    return;
  }
  currentMissionId = mission.mission_id;
  drawMissionPreview(mission);
  document.getElementById('btn-launch-mission').disabled = false;
  logStatus(`Mission ${currentMissionId} generated (${mission.waypoints.length} waypoints)`);
});

document.getElementById('btn-launch-mission').addEventListener('click', async () => {
  if (!currentMissionId) return;
  const resp = await fetch('/api/mission/launch', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mission_id: currentMissionId }),
  });
  const result = await resp.json();
  logStatus(result.error ? `Launch error: ${result.error}` : `Mission sent (${result.status})`);
});

document.getElementById('mp-upload').addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const text = await file.text();
  let mission;
  try {
    mission = JSON.parse(text);
  } catch (err) {
    alert('Invalid JSON file');
    return;
  }
  const resp = await fetch('/api/mission/upload_custom', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(mission),
  });
  const result = await resp.json();
  if (result.error) {
    logStatus(`Upload error: ${result.error}`);
    return;
  }
  currentMissionId = result.mission_id;
  drawMissionPreview(result);
  document.getElementById('btn-launch-mission').disabled = false;
  logStatus(`Custom mission ${currentMissionId} loaded (${result.waypoints.length} waypoints)`);
});

document.getElementById('btn-rtl').addEventListener('click', async () => {
  await fetch('/api/mission/rtl', { method: 'POST' });
  logStatus('Return-to-Launch commanded');
});

document.getElementById('btn-land').addEventListener('click', async () => {
  if (!confirm('Confirm EMERGENCY LANDING now?')) return;
  await fetch('/api/mission/land', { method: 'POST' });
  logStatus('EMERGENCY LANDING commanded');
});

let missionPathLayer = null;
function drawMissionPreview(mission) {
  if (missionPathLayer) map.removeLayer(missionPathLayer);
  const latlngs = mission.waypoints.map((wp) => [wp.lat, wp.lon]);
  missionPathLayer = L.polyline(latlngs, { color: '#2979ff', dashArray: '6 4' }).addTo(map);
  map.fitBounds(missionPathLayer.getBounds(), { padding: [40, 40] });
}

function logStatus(text) {
  const el = document.getElementById('mission-status');
  const line = document.createElement('div');
  const ts = new Date().toLocaleTimeString();
  line.textContent = `[${ts}] ${text}`;
  el.prepend(line);
}
