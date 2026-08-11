#!/usr/bin/env python3
"""
rosbridge_client.py
----------------------
Thin WebSocket client for rosbridge_suite (rosbridge_server), giving the
Flask ground station a way to subscribe to ROS2 topics, publish to them,
and call services on the drone's Raspberry Pi -- without needing a full
ROS2 install on the ground-station machine.

Protocol reference: https://github.com/RobotWebTools/rosbridge_suite
"""

import json
import threading
import time
import uuid

try:
    import websocket  # websocket-client package
except ImportError:  # pragma: no cover
    websocket = None


class RosbridgeClient:
    def __init__(self, url: str, reconnect_delay=3.0):
        self.url = url
        self.reconnect_delay = reconnect_delay
        self._ws = None
        self._connected = False
        self._subscriptions = {}  # topic -> callback
        self._lock = threading.Lock()

    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------ #
    # Connection lifecycle
    # ------------------------------------------------------------------ #
    def run_forever(self):
        if websocket is None:
            print('[rosbridge_client] websocket-client not installed, skipping connection')
            return
        while True:
            try:
                self._ws = websocket.WebSocketApp(
                    self.url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_close=self._on_close,
                    on_error=self._on_error,
                )
                self._ws.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as e:
                print(f'[rosbridge_client] connection error: {e}')
            self._connected = False
            time.sleep(self.reconnect_delay)

    def _on_open(self, ws):
        self._connected = True
        print(f'[rosbridge_client] connected to {self.url}')
        # Re-subscribe to everything registered before/after reconnects
        with self._lock:
            for topic, (msg_type, _cb) in self._subscriptions.items():
                self._send({
                    'op': 'subscribe', 'topic': topic, 'type': msg_type,
                })

    def _on_close(self, ws, *_args):
        self._connected = False
        print('[rosbridge_client] disconnected')

    def _on_error(self, ws, error):
        print(f'[rosbridge_client] error: {error}')

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            return
        if data.get('op') == 'publish':
            topic = data.get('topic')
            with self._lock:
                entry = self._subscriptions.get(topic)
            if entry:
                _msg_type, cb = entry
                try:
                    cb(data.get('msg', {}))
                except Exception as e:
                    print(f'[rosbridge_client] callback error on {topic}: {e}')

    def _send(self, payload: dict):
        if self._ws is not None and self._connected:
            try:
                self._ws.send(json.dumps(payload))
            except Exception as e:
                print(f'[rosbridge_client] send error: {e}')

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def subscribe(self, topic: str, msg_type: str, callback):
        with self._lock:
            self._subscriptions[topic] = (msg_type, callback)
        if self._connected:
            self._send({'op': 'subscribe', 'topic': topic, 'type': msg_type})

    def unsubscribe(self, topic: str):
        with self._lock:
            self._subscriptions.pop(topic, None)
        self._send({'op': 'unsubscribe', 'topic': topic})

    def publish(self, topic: str, msg_type: str, msg: dict):
        self._send({
            'op': 'publish', 'topic': topic, 'type': msg_type, 'msg': msg,
        })

    def call_service(self, service: str, srv_type: str, args: dict, timeout=5.0):
        request_id = str(uuid.uuid4())
        self._send({
            'op': 'call_service', 'service': service, 'type': srv_type,
            'args': args, 'id': request_id,
        })
        # Fire-and-forget for simplicity: mission-critical calls (RTL/LAND)
        # are also mirrored through direct MAVROS REST-like set_mode calls
        # so UI feedback does not depend on a service response round-trip.
