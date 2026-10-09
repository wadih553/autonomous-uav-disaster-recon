# Security policy and operational safety

This is an academic UAV prototype. It is not certified flight-control software, and its source code should not be treated as safe for unsupervised or emergency-critical operations.

## Before running the ground station

- Run it only on a trusted, isolated network. Do not expose Flask or ROSBridge directly to the public internet.
- Configure a strong, unique secret and keep API keys, passwords, SSH private keys, and local environment files out of Git.
- Use SSH host-key verification. Provision the Raspberry Pi's verified host key in the operator account's `known_hosts` file before using the SSH fallback.
- Restrict access to the ground station and the ROSBridge endpoint at the network/firewall level. The current app does not yet implement complete operator authentication and authorization for every mission-control endpoint.
- Review CORS, session settings, mission file storage permissions, and all control routes before connecting a real aircraft.
- Validate the mission in simulation and use a propeller-free bench setup before any flight. Check failsafes, geofence, altitude limits, battery reserve, RTL behaviour, and manual override on the actual airframe.

## Reporting a vulnerability

Please do not publish exploitable details involving a live aircraft or operational network in a public issue. Contact the repository maintainer privately with a concise description and safe reproduction steps.

## Scope

Security hardening in this repository is incremental. A passing unit test or CI run does not prove flight safety, hardware compatibility, or end-to-end system correctness.
