#!/usr/bin/env python3
"""
app.py
--------
Flask + Socket.IO ground station backend (FYP report, Ch. 4.3.1).

Responsibilities:
  - Serve the web frontend (Leaflet map, telemetry dashboard, video panels).
  - Bridge to the drone's ROS2 stack over ROSBridge (WebSocket, port 9090)
    to receive telemetry, sensor data, and the live camera feed, and to
    push mission files.
  - Generate mission JSON (waypoints, scan patterns) from user input,
    optionally using the OpenRouteService (ORS) API for terrain-aware
    route/elevation planning.
  - Push missions to the Raspberry Pi over SSH/SCP (Paramiko) as a backup
    path when ROSBridge mission upload is not available.
  - Run YOLOv8 human detection and a fire/smoke CNN on incoming video
    frames (see detection.py) and relay annotated results to the frontend.
"""

import json
import os
import time
import threading
import re
from datetime import datetime

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit

from mission_planner import MissionPlanner
from rosbridge_client import RosbridgeClient
from ssh_uploader import SSHMissionUploader
from detection import DetectionPipeline

# --------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------- #
RASPBERRY_PI_HOST = os.environ.get('UAV_PI_HOST', '192.168.1.50')
RASPBERRY_PI_USER = os.environ.get('UAV_PI_USER', 'pi')
RASPBERRY_PI_SSH_KEY = os.environ.get('UAV_PI_SSH_KEY', '~/.ssh/id_rsa')
ROSBRIDGE_WS_URL = os.environ.get(
    'UAV_ROSBRIDGE_URL', f'ws://{RASPBERRY_PI_HOST}:9090'
)
ORS_API_KEY = os.environ.get('ORS_API_KEY', '')
MISSION_STORAGE_PATH = os.path.abspath(os.environ.get('UAV_MISSION_STORAGE', './missions'))
MISSION_ID_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,100}$')

# --------------------------------------------------------------------- #
# App init
# --------------------------------------------------------------------- #
app = Flask(__name__, static_folder='../ui/static', template_folder='../ui/templates')
app.config['SECRET_KEY'] = os.environ.get('UAV_SECRET_KEY') or os.urandom(32).hex()
if not os.environ.get('UAV_SECRET_KEY'):
    app.logger.warning('UAV_SECRET_KEY is unset; using a temporary process-local secret.')
allowed_origins = [origin.strip() for origin in os.environ.get(
    'UAV_CORS_ALLOWED_ORIGINS', 'http://127.0.0.1:5000,http://localhost:5000'
).split(',') if origin.strip()]
socketio = SocketIO(app, cors_allowed_origins=allowed_origins, async_mode='threading')

mission_planner = MissionPlanner(ors_api_key=ORS_API_KEY)
ssh_uploader = SSHMissionUploader(
    host=RASPBERRY_PI_HOST, user=RASPBERRY_PI_USER, key_path=RASPBERRY_PI_SSH_KEY
)
detection_pipeline = DetectionPipeline()

rosbridge = RosbridgeClient(ROSBRIDGE_WS_URL)

# In-memory latest state, pushed to newly-connecting frontend clients
latest_state = {
    'telemetry': {},
    'env': {},
    'mission_status': {},
    'gps': {},
}

# --------------------------------------------------------------------- #
# ROSBridge -> Socket.IO relay callbacks
# --------------------------------------------------------------------- #
def on_global_position(msg):
    latest_state['gps'] = {
        'lat': msg['latitude'],
        'lon': msg['longitude'],
        'alt': msg['altitude'],
        'timestamp': time.time(),
    }
    socketio.emit('telemetry:gps', latest_state['gps'])


def on_mavros_state(msg):
    latest_state['telemetry'].update({
        'armed': msg.get('armed'),
        'mode': msg.get('mode'),
        'connected': msg.get('connected'),
    })
    socketio.emit('telemetry:state', latest_state['telemetry'])


def on_battery(msg):
    payload = {'voltage': msg.get('voltage'), 'percentage': msg.get('percentage')}
    latest_state['telemetry'].update(payload)
    socketio.emit('telemetry:battery', payload)


def on_env_summary(msg):
    try:
        env = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['env'] = env
    socketio.emit('telemetry:env', env)


def on_mission_status(msg):
    try:
        status = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['mission_status'] = status
    socketio.emit('mission:status', status)


def on_camera_frame(msg):
    """msg['data'] is base64-encoded JPEG from sensor_msgs/CompressedImage."""
    frame_b64 = msg.get('data')
    if not frame_b64:
        return
    annotated = detection_pipeline.process_frame_b64(frame_b64)
    socketio.emit('video:frame', {
        'raw': frame_b64,
        'human_boxes': annotated.get('human_boxes', []),
        'fire_boxes': annotated.get('fire_boxes', []),
        'timestamp': time.time(),
    })


def _register_rosbridge_subscriptions():
    rosbridge.subscribe('mavros/global_position/global', 'sensor_msgs/NavSatFix', on_global_position)
    rosbridge.subscribe('mavros/state', 'mavros_msgs/State', on_mavros_state)
    rosbridge.subscribe('mavros/battery', 'sensor_msgs/BatteryState', on_battery)
    rosbridge.subscribe('drone/env/summary', 'std_msgs/String', on_env_summary)
    rosbridge.subscribe('drone/mission/status', 'std_msgs/String', on_mission_status)
    rosbridge.subscribe(
        'drone/camera/image_raw/compressed', 'sensor_msgs/CompressedImage', on_camera_frame
    )


# --------------------------------------------------------------------- #
# HTTP routes
# --------------------------------------------------------------------- #
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    return jsonify({
        'rosbridge_connected': rosbridge.is_connected(),
        'latest_state': latest_state,
    })


@app.route('/api/mission/generate', methods=['POST'])
def api_generate_mission():
    """Builds a mission JSON (scan pattern or manual waypoints) from
    operator-supplied parameters, as described in Ch. 4.3.1."""
    params = request.get_json(silent=True)
    if not isinstance(params, dict):
        return jsonify({'error': 'Request body must be a JSON object'}), 400
    required = {'center_lat', 'center_lon', 'scan_radius_m', 'cruise_altitude_m', 'scan_altitude_m'}
    missing = required - params.keys()
    if missing:
        return jsonify({'error': f'Missing parameters: {sorted(missing)}'}), 400

    try:
        mission = mission_planner.build_scan_mission(
            center_lat=params['center_lat'],
            center_lon=params['center_lon'],
            scan_radius_m=params['scan_radius_m'],
            cruise_altitude_m=params['cruise_altitude_m'],
            scan_altitude_m=params['scan_altitude_m'],
            num_points=params.get('num_points', 12),
        )
    except (TypeError, ValueError) as e:
        return jsonify({'error': str(e)}), 400

    mission_id = mission['mission_id']
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)

    return jsonify(mission)


@app.route('/api/mission/upload_custom', methods=['POST'])
def api_upload_custom_mission():
    """Accepts a user-supplied mission JSON file, validates it, and stores it."""
    mission = request.get_json(silent=True)
    if not isinstance(mission, dict):
        return jsonify({'error': 'Request body must be a JSON object'}), 400
    ok, reason = mission_planner.validate(mission)
    if not ok:
        return jsonify({'error': reason}), 400

    mission_id = mission['mission_id']
    if not MISSION_ID_PATTERN.fullmatch(mission_id):
        return jsonify({'error': 'mission_id may contain only letters, digits, underscores, and hyphens'}), 400
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)
    return jsonify(mission)


@app.route('/api/mission/launch', methods=['POST'])
def api_launch_mission():
    """Pushes the mission to the UAV. Tries ROSBridge first (fast path);
    falls back to SCP + remote service trigger over SSH if ROSBridge is
    unavailable (Ch. 4.3.1)."""
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({'error': 'Request body must be a JSON object'}), 400
    mission_id = payload.get('mission_id')
    if not isinstance(mission_id, str) or not MISSION_ID_PATTERN.fullmatch(mission_id):
        return jsonify({'error': 'A valid mission_id is required'}), 400

    path = os.path.abspath(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'))
    if os.path.commonpath([MISSION_STORAGE_PATH, path]) != MISSION_STORAGE_PATH:
        return jsonify({'error': 'Invalid mission path'}), 400
    if not os.path.exists(path):
        return jsonify({'error': f'Mission {mission_id} not found'}), 404

    with open(path) as f:
        mission_data = f.read()

    if rosbridge.is_connected():
        sent = rosbridge.publish('ground_station/mission/upload', 'std_msgs/String', {'data': mission_data})
        if not sent:
            return jsonify({'error': 'ROSBridge connection exists but the mission payload could not be sent'}), 502
        return jsonify({'status': 'sent_via_rosbridge', 'note': 'Sent to ROSBridge; verify mission status and vehicle state before proceeding.'})

    try:
        ssh_uploader.upload_mission(path)
        ssh_uploader.trigger_mission_start()
        return jsonify({'status': 'sent_via_ssh'})
    except Exception as e:
        return jsonify({'error': f'SSH fallback failed: {e}'}), 500


@app.route('/api/mission/rtl', methods=['POST'])
def api_return_to_launch():
    if not rosbridge.is_connected():
        return jsonify({'error': 'ROSBridge is disconnected; RTL request was not sent'}), 503
    sent = rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'RTL'})
    if not sent:
        return jsonify({'error': 'RTL request could not be sent'}), 502
    return jsonify({'status': 'rtl_request_sent', 'note': 'This confirms only a request was sent, not that the aircraft changed mode.'})


@app.route('/api/mission/land', methods=['POST'])
def api_emergency_land():
    if not rosbridge.is_connected():
        return jsonify({'error': 'ROSBridge is disconnected; LAND request was not sent'}), 503
    sent = rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'LAND'})
    if not sent:
        return jsonify({'error': 'LAND request could not be sent'}), 502
    return jsonify({'status': 'land_request_sent', 'note': 'This confirms only a request was sent, not that the aircraft changed mode.'})


@app.route('/api/mission/elevation_profile')
def api_elevation_profile():
    try:
        lat = float(request.args.get('lat', ''))
        lon = float(request.args.get('lon', ''))
        radius = float(request.args.get('radius_m', 100))
        profile = mission_planner.get_elevation_profile(lat, lon, radius)
        return jsonify(profile)
    except (TypeError, ValueError) as e:
        return jsonify({'error': str(e)}), 400
    except Exception as e:
        app.logger.exception('Elevation profile request failed')
        return jsonify({'error': str(e)}), 502


# --------------------------------------------------------------------- #
# Socket.IO events (frontend <-> backend)
# --------------------------------------------------------------------- #
@socketio.on('connect')
def handle_connect():
    emit('telemetry:snapshot', latest_state)


@socketio.on('video:set_mode')
def handle_video_mode(data):
    """Switches which overlay the frontend wants highlighted: 'live',
    'human', or 'fire'. Detection still runs on all frames regardless;
    this just controls what gets drawn client-side."""
    detection_pipeline.set_display_mode(data.get('mode', 'live'))


# --------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------- #
def main():
    _register_rosbridge_subscriptions()
    rosbridge_thread = threading.Thread(target=rosbridge.run_forever, daemon=True)
    rosbridge_thread.start()

    # Localhost is the safe default. Set UAV_GCS_HOST=0.0.0.0 only when
    # intentionally exposing the UI on a trusted, firewalled network.
    socketio.run(app, host=os.environ.get('UAV_GCS_HOST', '127.0.0.1'),
                 port=int(os.environ.get('UAV_GCS_PORT', 5000)))


if __name__ == '__main__':
    main()
)

os.makedirs(MISSION_STORAGE_PATH, exist_ok=True)

# --------------------------------------------------------------------- #
# App init
# --------------------------------------------------------------------- #
app = Flask(__name__, static_folder='../ui/static', template_folder='../ui/templates')
app.config['SECRET_KEY'] = os.environ.get('UAV_SECRET_KEY', 'change-me-in-production')
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

mission_planner = MissionPlanner(ors_api_key=ORS_API_KEY)
ssh_uploader = SSHMissionUploader(
    host=RASPBERRY_PI_HOST, user=RASPBERRY_PI_USER, key_path=RASPBERRY_PI_SSH_KEY
)
detection_pipeline = DetectionPipeline()

rosbridge = RosbridgeClient(ROSBRIDGE_WS_URL)

# In-memory latest state, pushed to newly-connecting frontend clients
latest_state = {
    'telemetry': {},
    'env': {},
    'mission_status': {},
    'gps': {},
}

# --------------------------------------------------------------------- #
# ROSBridge -> Socket.IO relay callbacks
# --------------------------------------------------------------------- #
def on_global_position(msg):
    latest_state['gps'] = {
        'lat': msg['latitude'],
        'lon': msg['longitude'],
        'alt': msg['altitude'],
        'timestamp': time.time(),
    }
    socketio.emit('telemetry:gps', latest_state['gps'])


def on_mavros_state(msg):
    latest_state['telemetry'].update({
        'armed': msg.get('armed'),
        'mode': msg.get('mode'),
        'connected': msg.get('connected'),
    })
    socketio.emit('telemetry:state', latest_state['telemetry'])


def on_battery(msg):
    payload = {'voltage': msg.get('voltage'), 'percentage': msg.get('percentage')}
    latest_state['telemetry'].update(payload)
    socketio.emit('telemetry:battery', payload)


def on_env_summary(msg):
    try:
        env = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['env'] = env
    socketio.emit('telemetry:env', env)


def on_mission_status(msg):
    try:
        status = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['mission_status'] = status
    socketio.emit('mission:status', status)


def on_camera_frame(msg):
    """msg['data'] is base64-encoded JPEG from sensor_msgs/CompressedImage."""
    frame_b64 = msg.get('data')
    if not frame_b64:
        return
    annotated = detection_pipeline.process_frame_b64(frame_b64)
    socketio.emit('video:frame', {
        'raw': frame_b64,
        'human_boxes': annotated.get('human_boxes', []),
        'fire_boxes': annotated.get('fire_boxes', []),
        'timestamp': time.time(),
    })


def _register_rosbridge_subscriptions():
    rosbridge.subscribe('mavros/global_position/global', 'sensor_msgs/NavSatFix', on_global_position)
    rosbridge.subscribe('mavros/state', 'mavros_msgs/State', on_mavros_state)
    rosbridge.subscribe('mavros/battery', 'sensor_msgs/BatteryState', on_battery)
    rosbridge.subscribe('drone/env/summary', 'std_msgs/String', on_env_summary)
    rosbridge.subscribe('drone/mission/status', 'std_msgs/String', on_mission_status)
    rosbridge.subscribe(
        'drone/camera/image_raw/compressed', 'sensor_msgs/CompressedImage', on_camera_frame
    )


# --------------------------------------------------------------------- #
# HTTP routes
# --------------------------------------------------------------------- #
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    return jsonify({
        'rosbridge_connected': rosbridge.is_connected(),
        'latest_state': latest_state,
    })


@app.route('/api/mission/generate', methods=['POST'])
def api_generate_mission():
    """Builds a mission JSON (scan pattern or manual waypoints) from
    operator-supplied parameters, as described in Ch. 4.3.1."""
    params = request.get_json(force=True)
    required = {'center_lat', 'center_lon', 'scan_radius_m', 'cruise_altitude_m', 'scan_altitude_m'}
    missing = required - params.keys()
    if missing:
        return jsonify({'error': f'Missing parameters: {missing}'}), 400

    try:
        mission = mission_planner.build_scan_mission(
            center_lat=params['center_lat'],
            center_lon=params['center_lon'],
            scan_radius_m=params['scan_radius_m'],
            cruise_altitude_m=params['cruise_altitude_m'],
            scan_altitude_m=params['scan_altitude_m'],
            num_points=params.get('num_points', 12),
        )
    except Exception as e:
        return jsonify({'error': str(e)}), 500

    mission_id = mission['mission_id']
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)

    return jsonify(mission)


@app.route('/api/mission/upload_custom', methods=['POST'])
def api_upload_custom_mission():
    """Accepts a user-supplied mission JSON file, validates it, and stores it."""
    mission = request.get_json(force=True)
    ok, reason = mission_planner.validate(mission)
    if not ok:
        return jsonify({'error': reason}), 400

    mission_id = mission.get('mission_id') or datetime.utcnow().strftime('%Y%m%dT%H%M%S')
    mission['mission_id'] = mission_id
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)
    return jsonify(mission)


@app.route('/api/mission/launch', methods=['POST'])
def api_launch_mission():
    """Pushes the mission to the UAV. Tries ROSBridge first (fast path);
    falls back to SCP + remote service trigger over SSH if ROSBridge is
    unavailable (Ch. 4.3.1)."""
    mission_id = request.get_json(force=True).get('mission_id')
    if not mission_id:
        return jsonify({'error': 'mission_id is required'}), 400

    path = os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json')
    if not os.path.exists(path):
        return jsonify({'error': f'Mission {mission_id} not found'}), 404

    with open(path) as f:
        mission_data = f.read()

    if rosbridge.is_connected():
        rosbridge.publish('ground_station/mission/upload', 'std_msgs/String', {'data': mission_data})
        return jsonify({'status': 'sent_via_rosbridge'})

    try:
        ssh_uploader.upload_mission(path)
        ssh_uploader.trigger_mission_start()
        return jsonify({'status': 'sent_via_ssh'})
    except Exception as e:
        return jsonify({'error': f'SSH fallback failed: {e}'}), 500


@app.route('/api/mission/rtl', methods=['POST'])
def api_return_to_launch():
    rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'RTL'})
    return jsonify({'status': 'rtl_commanded'})


@app.route('/api/mission/land', methods=['POST'])
def api_emergency_land():
    rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'LAND'})
    return jsonify({'status': 'land_commanded'})


@app.route('/api/mission/elevation_profile')
def api_elevation_profile():
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    radius = float(request.args.get('radius_m', 100))
    try:
        profile = mission_planner.get_elevation_profile(lat, lon, radius)
        return jsonify(profile)
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# --------------------------------------------------------------------- #
# Socket.IO events (frontend <-> backend)
# --------------------------------------------------------------------- #
@socketio.on('connect')
def handle_connect():
    emit('telemetry:snapshot', latest_state)


@socketio.on('video:set_mode')
def handle_video_mode(data):
    """Switches which overlay the frontend wants highlighted: 'live',
    'human', or 'fire'. Detection still runs on all frames regardless;
    this just controls what gets drawn client-side."""
    detection_pipeline.set_display_mode(data.get('mode', 'live'))


# --------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------- #
def main():
    _register_rosbridge_subscriptions()
    rosbridge_thread = threading.Thread(target=rosbridge.run_forever, daemon=True)
    rosbridge_thread.start()

    socketio.run(app, host='0.0.0.0', port=int(os.environ.get('UAV_GCS_PORT', 5000)))


if __name__ == '__main__':
    main()
)os.makedirs(MISSION_STORAGE_PATH, exist_ok=True)

# --------------------------------------------------------------------- #
# App init
# --------------------------------------------------------------------- #
app = Flask(__name__, static_folder='../ui/static', template_folder='../ui/templates')
app.config['SECRET_KEY'] = os.environ.get('UAV_SECRET_KEY') or os.urandom(32).hex()
if not os.environ.get('UAV_SECRET_KEY'):
    app.logger.warning('UAV_SECRET_KEY is unset; using a temporary process-local secret.')
allowed_origins = [origin.strip() for origin in os.environ.get(
    'UAV_CORS_ALLOWED_ORIGINS', 'http://127.0.0.1:5000,http://localhost:5000'
).split(',') if origin.strip()]
socketio = SocketIO(app, cors_allowed_origins=allowed_origins, async_mode='threading')

mission_planner = MissionPlanner(ors_api_key=ORS_API_KEY)
ssh_uploader = SSHMissionUploader(
    host=RASPBERRY_PI_HOST, user=RASPBERRY_PI_USER, key_path=RASPBERRY_PI_SSH_KEY
)
detection_pipeline = DetectionPipeline()

rosbridge = RosbridgeClient(ROSBRIDGE_WS_URL)

# In-memory latest state, pushed to newly-connecting frontend clients
latest_state = {
    'telemetry': {},
    'env': {},
    'mission_status': {},
    'gps': {},
}

# --------------------------------------------------------------------- #
# ROSBridge -> Socket.IO relay callbacks
# --------------------------------------------------------------------- #
def on_global_position(msg):
    latest_state['gps'] = {
        'lat': msg['latitude'],
        'lon': msg['longitude'],
        'alt': msg['altitude'],
        'timestamp': time.time(),
    }
    socketio.emit('telemetry:gps', latest_state['gps'])


def on_mavros_state(msg):
    latest_state['telemetry'].update({
        'armed': msg.get('armed'),
        'mode': msg.get('mode'),
        'connected': msg.get('connected'),
    })
    socketio.emit('telemetry:state', latest_state['telemetry'])


def on_battery(msg):
    payload = {'voltage': msg.get('voltage'), 'percentage': msg.get('percentage')}
    latest_state['telemetry'].update(payload)
    socketio.emit('telemetry:battery', payload)


def on_env_summary(msg):
    try:
        env = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['env'] = env
    socketio.emit('telemetry:env', env)


def on_mission_status(msg):
    try:
        status = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['mission_status'] = status
    socketio.emit('mission:status', status)


def on_camera_frame(msg):
    """msg['data'] is base64-encoded JPEG from sensor_msgs/CompressedImage."""
    frame_b64 = msg.get('data')
    if not frame_b64:
        return
    annotated = detection_pipeline.process_frame_b64(frame_b64)
    socketio.emit('video:frame', {
        'raw': frame_b64,
        'human_boxes': annotated.get('human_boxes', []),
        'fire_boxes': annotated.get('fire_boxes', []),
        'timestamp': time.time(),
    })


def _register_rosbridge_subscriptions():
    rosbridge.subscribe('mavros/global_position/global', 'sensor_msgs/NavSatFix', on_global_position)
    rosbridge.subscribe('mavros/state', 'mavros_msgs/State', on_mavros_state)
    rosbridge.subscribe('mavros/battery', 'sensor_msgs/BatteryState', on_battery)
    rosbridge.subscribe('drone/env/summary', 'std_msgs/String', on_env_summary)
    rosbridge.subscribe('drone/mission/status', 'std_msgs/String', on_mission_status)
    rosbridge.subscribe(
        'drone/camera/image_raw/compressed', 'sensor_msgs/CompressedImage', on_camera_frame
    )


# --------------------------------------------------------------------- #
# HTTP routes
# --------------------------------------------------------------------- #
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    return jsonify({
        'rosbridge_connected': rosbridge.is_connected(),
        'latest_state': latest_state,
    })


@app.route('/api/mission/generate', methods=['POST'])
def api_generate_mission():
    """Builds a mission JSON (scan pattern or manual waypoints) from
    operator-supplied parameters, as described in Ch. 4.3.1."""
    params = request.get_json(silent=True)
    if not isinstance(params, dict):
        return jsonify({'error': 'Request body must be a JSON object'}), 400
    required = {'center_lat', 'center_lon', 'scan_radius_m', 'cruise_altitude_m', 'scan_altitude_m'}
    missing = required - params.keys()
    if missing:
        return jsonify({'error': f'Missing parameters: {sorted(missing)}'}), 400

    try:
        mission = mission_planner.build_scan_mission(
            center_lat=params['center_lat'],
            center_lon=params['center_lon'],
            scan_radius_m=params['scan_radius_m'],
            cruise_altitude_m=params['cruise_altitude_m'],
            scan_altitude_m=params['scan_altitude_m'],
            num_points=params.get('num_points', 12),
        )
    except (TypeError, ValueError) as e:
        return jsonify({'error': str(e)}), 400

    mission_id = mission['mission_id']
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)

    return jsonify(mission)


@app.route('/api/mission/upload_custom', methods=['POST'])
def api_upload_custom_mission():
    """Accepts a user-supplied mission JSON file, validates it, and stores it."""
    mission = request.get_json(silent=True)
    if not isinstance(mission, dict):
        return jsonify({'error': 'Request body must be a JSON object'}), 400
    ok, reason = mission_planner.validate(mission)
    if not ok:
        return jsonify({'error': reason}), 400

    mission_id = mission['mission_id']
    if not MISSION_ID_PATTERN.fullmatch(mission_id):
        return jsonify({'error': 'mission_id may contain only letters, digits, underscores, and hyphens'}), 400
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)
    return jsonify(mission)


@app.route('/api/mission/launch', methods=['POST'])
def api_launch_mission():
    """Pushes the mission to the UAV. Tries ROSBridge first (fast path);
    falls back to SCP + remote service trigger over SSH if ROSBridge is
    unavailable (Ch. 4.3.1)."""
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({'error': 'Request body must be a JSON object'}), 400
    mission_id = payload.get('mission_id')
    if not isinstance(mission_id, str) or not MISSION_ID_PATTERN.fullmatch(mission_id):
        return jsonify({'error': 'A valid mission_id is required'}), 400

    path = os.path.abspath(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'))
    if os.path.commonpath([MISSION_STORAGE_PATH, path]) != MISSION_STORAGE_PATH:
        return jsonify({'error': 'Invalid mission path'}), 400
    if not os.path.exists(path):
        return jsonify({'error': f'Mission {mission_id} not found'}), 404

    with open(path) as f:
        mission_data = f.read()

    if rosbridge.is_connected():
        sent = rosbridge.publish('ground_station/mission/upload', 'std_msgs/String', {'data': mission_data})
        if not sent:
            return jsonify({'error': 'ROSBridge connection exists but the mission payload could not be sent'}), 502
        return jsonify({'status': 'sent_via_rosbridge', 'note': 'Sent to ROSBridge; verify mission status and vehicle state before proceeding.'})

    try:
        ssh_uploader.upload_mission(path)
        ssh_uploader.trigger_mission_start()
        return jsonify({'status': 'sent_via_ssh'})
    except Exception as e:
        return jsonify({'error': f'SSH fallback failed: {e}'}), 500


@app.route('/api/mission/rtl', methods=['POST'])
def api_return_to_launch():
    if not rosbridge.is_connected():
        return jsonify({'error': 'ROSBridge is disconnected; RTL request was not sent'}), 503
    sent = rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'RTL'})
    if not sent:
        return jsonify({'error': 'RTL request could not be sent'}), 502
    return jsonify({'status': 'rtl_request_sent', 'note': 'This confirms only a request was sent, not that the aircraft changed mode.'})


@app.route('/api/mission/land', methods=['POST'])
def api_emergency_land():
    if not rosbridge.is_connected():
        return jsonify({'error': 'ROSBridge is disconnected; LAND request was not sent'}), 503
    sent = rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'LAND'})
    if not sent:
        return jsonify({'error': 'LAND request could not be sent'}), 502
    return jsonify({'status': 'land_request_sent', 'note': 'This confirms only a request was sent, not that the aircraft changed mode.'})


@app.route('/api/mission/elevation_profile')
def api_elevation_profile():
    try:
        lat = float(request.args.get('lat', ''))
        lon = float(request.args.get('lon', ''))
        radius = float(request.args.get('radius_m', 100))
        profile = mission_planner.get_elevation_profile(lat, lon, radius)
        return jsonify(profile)
    except (TypeError, ValueError) as e:
        return jsonify({'error': str(e)}), 400
    except Exception as e:
        app.logger.exception('Elevation profile request failed')
        return jsonify({'error': str(e)}), 502


# --------------------------------------------------------------------- #
# Socket.IO events (frontend <-> backend)
# --------------------------------------------------------------------- #
@socketio.on('connect')
def handle_connect():
    emit('telemetry:snapshot', latest_state)


@socketio.on('video:set_mode')
def handle_video_mode(data):
    """Switches which overlay the frontend wants highlighted: 'live',
    'human', or 'fire'. Detection still runs on all frames regardless;
    this just controls what gets drawn client-side."""
    detection_pipeline.set_display_mode(data.get('mode', 'live'))


# --------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------- #
def main():
    _register_rosbridge_subscriptions()
    rosbridge_thread = threading.Thread(target=rosbridge.run_forever, daemon=True)
    rosbridge_thread.start()

    # Localhost is the safe default. Set UAV_GCS_HOST=0.0.0.0 only when
    # intentionally exposing the UI on a trusted, firewalled network.
    socketio.run(app, host=os.environ.get('UAV_GCS_HOST', '127.0.0.1'),
                 port=int(os.environ.get('UAV_GCS_PORT', 5000)))


if __name__ == '__main__':
    main()
)

os.makedirs(MISSION_STORAGE_PATH, exist_ok=True)

# --------------------------------------------------------------------- #
# App init
# --------------------------------------------------------------------- #
app = Flask(__name__, static_folder='../ui/static', template_folder='../ui/templates')
app.config['SECRET_KEY'] = os.environ.get('UAV_SECRET_KEY', 'change-me-in-production')
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

mission_planner = MissionPlanner(ors_api_key=ORS_API_KEY)
ssh_uploader = SSHMissionUploader(
    host=RASPBERRY_PI_HOST, user=RASPBERRY_PI_USER, key_path=RASPBERRY_PI_SSH_KEY
)
detection_pipeline = DetectionPipeline()

rosbridge = RosbridgeClient(ROSBRIDGE_WS_URL)

# In-memory latest state, pushed to newly-connecting frontend clients
latest_state = {
    'telemetry': {},
    'env': {},
    'mission_status': {},
    'gps': {},
}

# --------------------------------------------------------------------- #
# ROSBridge -> Socket.IO relay callbacks
# --------------------------------------------------------------------- #
def on_global_position(msg):
    latest_state['gps'] = {
        'lat': msg['latitude'],
        'lon': msg['longitude'],
        'alt': msg['altitude'],
        'timestamp': time.time(),
    }
    socketio.emit('telemetry:gps', latest_state['gps'])


def on_mavros_state(msg):
    latest_state['telemetry'].update({
        'armed': msg.get('armed'),
        'mode': msg.get('mode'),
        'connected': msg.get('connected'),
    })
    socketio.emit('telemetry:state', latest_state['telemetry'])


def on_battery(msg):
    payload = {'voltage': msg.get('voltage'), 'percentage': msg.get('percentage')}
    latest_state['telemetry'].update(payload)
    socketio.emit('telemetry:battery', payload)


def on_env_summary(msg):
    try:
        env = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['env'] = env
    socketio.emit('telemetry:env', env)


def on_mission_status(msg):
    try:
        status = json.loads(msg['data']) if isinstance(msg, dict) else json.loads(msg)
    except (TypeError, json.JSONDecodeError):
        return
    latest_state['mission_status'] = status
    socketio.emit('mission:status', status)


def on_camera_frame(msg):
    """msg['data'] is base64-encoded JPEG from sensor_msgs/CompressedImage."""
    frame_b64 = msg.get('data')
    if not frame_b64:
        return
    annotated = detection_pipeline.process_frame_b64(frame_b64)
    socketio.emit('video:frame', {
        'raw': frame_b64,
        'human_boxes': annotated.get('human_boxes', []),
        'fire_boxes': annotated.get('fire_boxes', []),
        'timestamp': time.time(),
    })


def _register_rosbridge_subscriptions():
    rosbridge.subscribe('mavros/global_position/global', 'sensor_msgs/NavSatFix', on_global_position)
    rosbridge.subscribe('mavros/state', 'mavros_msgs/State', on_mavros_state)
    rosbridge.subscribe('mavros/battery', 'sensor_msgs/BatteryState', on_battery)
    rosbridge.subscribe('drone/env/summary', 'std_msgs/String', on_env_summary)
    rosbridge.subscribe('drone/mission/status', 'std_msgs/String', on_mission_status)
    rosbridge.subscribe(
        'drone/camera/image_raw/compressed', 'sensor_msgs/CompressedImage', on_camera_frame
    )


# --------------------------------------------------------------------- #
# HTTP routes
# --------------------------------------------------------------------- #
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    return jsonify({
        'rosbridge_connected': rosbridge.is_connected(),
        'latest_state': latest_state,
    })


@app.route('/api/mission/generate', methods=['POST'])
def api_generate_mission():
    """Builds a mission JSON (scan pattern or manual waypoints) from
    operator-supplied parameters, as described in Ch. 4.3.1."""
    params = request.get_json(force=True)
    required = {'center_lat', 'center_lon', 'scan_radius_m', 'cruise_altitude_m', 'scan_altitude_m'}
    missing = required - params.keys()
    if missing:
        return jsonify({'error': f'Missing parameters: {missing}'}), 400

    try:
        mission = mission_planner.build_scan_mission(
            center_lat=params['center_lat'],
            center_lon=params['center_lon'],
            scan_radius_m=params['scan_radius_m'],
            cruise_altitude_m=params['cruise_altitude_m'],
            scan_altitude_m=params['scan_altitude_m'],
            num_points=params.get('num_points', 12),
        )
    except Exception as e:
        return jsonify({'error': str(e)}), 500

    mission_id = mission['mission_id']
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)

    return jsonify(mission)


@app.route('/api/mission/upload_custom', methods=['POST'])
def api_upload_custom_mission():
    """Accepts a user-supplied mission JSON file, validates it, and stores it."""
    mission = request.get_json(force=True)
    ok, reason = mission_planner.validate(mission)
    if not ok:
        return jsonify({'error': reason}), 400

    mission_id = mission.get('mission_id') or datetime.utcnow().strftime('%Y%m%dT%H%M%S')
    mission['mission_id'] = mission_id
    with open(os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json'), 'w') as f:
        json.dump(mission, f, indent=2)
    return jsonify(mission)


@app.route('/api/mission/launch', methods=['POST'])
def api_launch_mission():
    """Pushes the mission to the UAV. Tries ROSBridge first (fast path);
    falls back to SCP + remote service trigger over SSH if ROSBridge is
    unavailable (Ch. 4.3.1)."""
    mission_id = request.get_json(force=True).get('mission_id')
    if not mission_id:
        return jsonify({'error': 'mission_id is required'}), 400

    path = os.path.join(MISSION_STORAGE_PATH, f'{mission_id}.json')
    if not os.path.exists(path):
        return jsonify({'error': f'Mission {mission_id} not found'}), 404

    with open(path) as f:
        mission_data = f.read()

    if rosbridge.is_connected():
        rosbridge.publish('ground_station/mission/upload', 'std_msgs/String', {'data': mission_data})
        return jsonify({'status': 'sent_via_rosbridge'})

    try:
        ssh_uploader.upload_mission(path)
        ssh_uploader.trigger_mission_start()
        return jsonify({'status': 'sent_via_ssh'})
    except Exception as e:
        return jsonify({'error': f'SSH fallback failed: {e}'}), 500


@app.route('/api/mission/rtl', methods=['POST'])
def api_return_to_launch():
    rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'RTL'})
    return jsonify({'status': 'rtl_commanded'})


@app.route('/api/mission/land', methods=['POST'])
def api_emergency_land():
    rosbridge.call_service('mavros/set_mode', 'mavros_msgs/SetMode', {'custom_mode': 'LAND'})
    return jsonify({'status': 'land_commanded'})


@app.route('/api/mission/elevation_profile')
def api_elevation_profile():
    lat = float(request.args.get('lat'))
    lon = float(request.args.get('lon'))
    radius = float(request.args.get('radius_m', 100))
    try:
        profile = mission_planner.get_elevation_profile(lat, lon, radius)
        return jsonify(profile)
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# --------------------------------------------------------------------- #
# Socket.IO events (frontend <-> backend)
# --------------------------------------------------------------------- #
@socketio.on('connect')
def handle_connect():
    emit('telemetry:snapshot', latest_state)


@socketio.on('video:set_mode')
def handle_video_mode(data):
    """Switches which overlay the frontend wants highlighted: 'live',
    'human', or 'fire'. Detection still runs on all frames regardless;
    this just controls what gets drawn client-side."""
    detection_pipeline.set_display_mode(data.get('mode', 'live'))


# --------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------- #
def main():
    _register_rosbridge_subscriptions()
    rosbridge_thread = threading.Thread(target=rosbridge.run_forever, daemon=True)
    rosbridge_thread.start()

    socketio.run(app, host='0.0.0.0', port=int(os.environ.get('UAV_GCS_PORT', 5000)))


if __name__ == '__main__':
    main()
