#!/usr/bin/env python3
"""
SSH fallback for delivering a mission file to the Raspberry Pi.

The remote host must already be present in the user's SSH known_hosts file.
Do not automatically trust unknown host keys on a mission-control channel.
"""

import os
import posixpath

try:
    import paramiko
except ImportError:  # pragma: no cover
    paramiko = None

REMOTE_MISSION_DIR = "/home/pi/missions"
REMOTE_START_SERVICE_CMD = (
    "ros2 service call /mission_receiver_node/start_mission "
    "std_srvs/srv/Trigger '{}'"
)


class SSHMissionUploader:
    def __init__(
        self,
        host: str,
        user: str,
        key_path: str = "~/.ssh/id_rsa",
        port: int = 22,
    ):
        self.host = host
        self.user = user
        self.key_path = os.path.expanduser(key_path)
        self.port = port

    def _connect(self):
        if paramiko is None:
            raise RuntimeError("paramiko is not installed (pip install paramiko)")

        client = paramiko.SSHClient()
        # Load trusted host keys and reject unknown hosts. Provision the Pi's
        # verified host key in ~/.ssh/known_hosts before using this fallback.
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        client.connect(
            hostname=self.host,
            port=self.port,
            username=self.user,
            key_filename=self.key_path,
            timeout=8,
            allow_agent=True,
            look_for_keys=True,
        )
        return client

    def upload_mission(self, local_path: str) -> str:
        if not os.path.isfile(local_path):
            raise FileNotFoundError(f"Mission file not found: {local_path}")

        client = self._connect()
        try:
            with client.open_sftp() as sftp:
                try:
                    sftp.stat(REMOTE_MISSION_DIR)
                except IOError:
                    sftp.mkdir(REMOTE_MISSION_DIR)

                remote_path = posixpath.join(
                    REMOTE_MISSION_DIR, os.path.basename(local_path)
                )
                sftp.put(local_path, remote_path)
                return remote_path
        finally:
            client.close()

    def trigger_mission_start(self):
        client = self._connect()
        try:
            _stdin, stdout, stderr = client.exec_command(
                REMOTE_START_SERVICE_CMD, timeout=10
            )
            exit_status = stdout.channel.recv_exit_status()
            if exit_status != 0:
                err = stderr.read().decode(errors="replace").strip()
                raise RuntimeError(
                    f"Remote mission-start command failed (exit {exit_status}): {err}"
                )
        finally:
            client.close()
