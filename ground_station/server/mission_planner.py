#!/usr/bin/env python3
"""
mission_planner.py
---------------------
Generates mission JSON files: circular scan patterns for area-coverage
missions (wildfire perimeter, SAR grid search) and, optionally, terrain-
aware elevation profiles via the OpenRouteService (ORS) API, as described
in the FYP report Ch. 4.3.1.
"""

import math
import uuid
from datetime import datetime, timezone

try:
    import requests
except ImportError:  # pragma: no cover
    requests = None

ORS_ELEVATION_URL = 'https://api.openrouteservice.org/elevation/point'
EARTH_RADIUS_M = 6371000.0

REQUIRED_TOP_LEVEL_KEYS = {'mission_id', 'waypoints'}
REQUIRED_WAYPOINT_KEYS = {'seq', 'lat', 'lon', 'alt'}


class MissionPlanner:
    def __init__(self, ors_api_key: str = ''):
        self.ors_api_key = ors_api_key

    # ------------------------------------------------------------------ #
    # Scan pattern generation
    # ------------------------------------------------------------------ #
    def build_scan_mission(self, center_lat, center_lon, scan_radius_m,
                            cruise_altitude_m, scan_altitude_m, num_points=12):
        """Builds a circular scan pattern mission: climb to cruise altitude,
        descend to scan altitude at the perimeter, sweep the circle at
        `num_points` evenly-spaced headings, then RTL."""
        mission_id = str(uuid.uuid4())
        waypoints = []
        seq = 0

        # Waypoint 0: climb straight up to cruise altitude over home
        waypoints.append({
            'seq': seq, 'lat': center_lat, 'lon': center_lon,
            'alt': cruise_altitude_m, 'action': 'waypoint',
        })
        seq += 1

        # Perimeter sweep at scan altitude
        for i in range(num_points):
            bearing = (2 * math.pi * i) / num_points
            lat, lon = self._destination_point(center_lat, center_lon, scan_radius_m, bearing)
            waypoints.append({
                'seq': seq, 'lat': lat, 'lon': lon,
                'alt': scan_altitude_m,
                'yaw': math.degrees(bearing),
                'action': 'waypoint',
            })
            seq += 1

        # Return to center, climb back to cruise altitude before RTL
        waypoints.append({
            'seq': seq, 'lat': center_lat, 'lon': center_lon,
            'alt': cruise_altitude_m, 'action': 'waypoint',
        })
        seq += 1

        waypoints.append({
            'seq': seq, 'lat': center_lat, 'lon': center_lon,
            'alt': 0, 'action': 'rtl',
        })

        mission = {
            'mission_id': mission_id,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'home': {'lat': center_lat, 'lon': center_lon, 'alt': 0},
            'cruise_altitude_m': cruise_altitude_m,
            'scan_altitude_m': scan_altitude_m,
            'scan_radius_m': scan_radius_m,
            'waypoints': waypoints,
        }
        return mission

    def _destination_point(self, lat, lon, distance_m, bearing_rad):
        """Great-circle destination point given a start point, distance, and
        bearing (standard spherical earth approximation, sufficient at the
        scan radii used here)."""
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        ang_dist = distance_m / EARTH_RADIUS_M

        lat2 = math.asin(
            math.sin(lat1) * math.cos(ang_dist)
            + math.cos(lat1) * math.sin(ang_dist) * math.cos(bearing_rad)
        )
        lon2 = lon1 + math.atan2(
            math.sin(bearing_rad) * math.sin(ang_dist) * math.cos(lat1),
            math.cos(ang_dist) - math.sin(lat1) * math.sin(lat2),
        )
        return math.degrees(lat2), math.degrees(lon2)

    # ------------------------------------------------------------------ #
    # ORS elevation profile
    # ------------------------------------------------------------------ #
    def get_elevation_profile(self, center_lat, center_lon, radius_m, num_samples=12):
        """Samples elevation around the scan perimeter via ORS, so the
        operator can sanity-check terrain clearance before launch."""
        if not self.ors_api_key or requests is None:
            raise RuntimeError('ORS_API_KEY not configured or `requests` not installed')

        profile = []
        headers = {'Authorization': self.ors_api_key}
        for i in range(num_samples):
            bearing = (2 * math.pi * i) / num_samples
            lat, lon = self._destination_point(center_lat, center_lon, radius_m, bearing)
            resp = requests.get(
                ORS_ELEVATION_URL,
                params={'geometry': f'{lon},{lat}'},
                headers=headers,
                timeout=5,
            )
            resp.raise_for_status()
            data = resp.json()
            elevation = data.get('geometry', {}).get('coordinates', [None, None, None])[2]
            profile.append({'lat': lat, 'lon': lon, 'elevation_m': elevation})
        return profile

    # ------------------------------------------------------------------ #
    # Validation (mirrors drone_pkg/mission_receiver_node.py so both ends
    # reject malformed missions consistently)
    # ------------------------------------------------------------------ #
    def validate(self, mission: dict):
        missing = REQUIRED_TOP_LEVEL_KEYS - mission.keys()
        if missing:
            return False, f'Missing top-level keys: {missing}'
        if not isinstance(mission['waypoints'], list) or not mission['waypoints']:
            return False, 'waypoints must be a non-empty list'
        for i, wp in enumerate(mission['waypoints']):
            missing_wp = REQUIRED_WAYPOINT_KEYS - wp.keys()
            if missing_wp:
                return False, f'Waypoint {i} missing keys: {missing_wp}'
            if not (-90 <= wp['lat'] <= 90) or not (-180 <= wp['lon'] <= 180):
                return False, f'Waypoint {i} has out-of-range lat/lon'
        return True, 'ok'
