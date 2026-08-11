#!/usr/bin/env python3
"""
ssh_uploader.py
------------------
Backup mission-delivery path used when ROSBridge is unreachable: securely
copies the mission JSON to the Raspberry Pi over SCP and remotely triggers
the mission-start ROS2 service via SSH (Paramiko), as described in the
FYP report Ch. 4.3.1.
"""

import os
import posixpath

try:
    import paramiko
except ImportError:  # pragma: no cover
    paramiko = None

REMOTE_MISSION_DIR = '/home/pi/missions'
# BUGFIX: this used to target /navigator_node/start_mission, a service that
# was never implemented anywhere (navigator_node only ever exposed a topic
# subscription, not a service server). The real service now lives on
# mission_receiver_node -- see drone_pkg/mission_receiver_node.py -- which
# loads the just-SCP'd file and republishes it on drone/mission/active
# exactly like the normal ROSBridge path, so navigator_node itself needed
# no changes for this fallback to work.
REMOTE_START_SERVICE_CMD = (
    "ros2 service call /mission_receiver_node/start_mission std_srvs/srv/Trigger '{}'"
)


class SSHMissionUploader:
    def __init__(self, host: str, user: str, key_path: str = '~/.ssh/id_rsa', port: int = 22):
        self.host = host
        self.user = user
        self.key_path = os.path.expanduser(key_path)
        self.port = port

    def _connect(self):
        if paramiko is None:
            raise RuntimeError('paramiko is not installed (pip install paramiko)')
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(
            hostname=self.host, port=self.port, username=self.user,
            key_filename=self.key_path, timeout=8,
        )
        return client

    def upload_mission(self, local_path: str) -> str:
        client = self._connect()
        try:
            sftp = client.open_sftp()
            try:
                sftp.mkdir(REMOTE_MISSION_DIR)
            except IOError:
                pass  # already exists
            remote_path = posixpath.join(REMOTE_MISSION_DIR, os.path.basename(local_path))
            sftp.put(local_path, remote_path)
            sftp.close()
            return remote_path
        finally:
            client.close()

    def trigger_mission_start(self):
        client = self._connect()
        try:
            stdin, stdout, stderr = client.exec_command(REMOTE_START_SERVICE_CMD, timeout=10)
            exit_status = stdout.channel.recv_exit_status()
            if exit_status != 0:
                err = stderr.read().decode(errors='ignore')
                raise RuntimeError(f'Remote mission-start command failed: {err}')
        finally:
            client.close()
