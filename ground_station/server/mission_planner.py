#!/usr/bin/env python3
"""Mission generation and validation helpers for the UAV ground station."""

import math
import uuid
from datetime import datetime, timezone

try:
    import requests
except ImportError:  # pragma: no cover
    requests = None

ORS_ELEVATION_URL = "https://api.openrouteservice.org/elevation/point"
EARTH_RADIUS_M = 6_371_000.0
REQUIRED_TOP_LEVEL_KEYS = {"mission_id", "waypoints"}
REQUIRED_WAYPOINT_KEYS = {"seq", "lat", "lon", "alt"}
MAX_WAYPOINTS = 500
MAX_SCAN_POINTS = 100


def _is_finite_number(value):
    """True for finite numeric values, excluding booleans."""
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class MissionPlanner:
    def __init__(self, ors_api_key: str = ""):
        self.ors_api_key = ors_api_key

    def build_scan_mission(
        self, center_lat, center_lon, scan_radius_m,
        cruise_altitude_m, scan_altitude_m, num_points=12
    ):
        """Generate a circular reconnaissance pattern; this is planning, not a flight-safety check."""
        numeric = {
            "center_lat": center_lat, "center_lon": center_lon,
            "scan_radius_m": scan_radius_m, "cruise_altitude_m": cruise_altitude_m,
            "scan_altitude_m": scan_altitude_m,
        }
        for name, value in numeric.items():
            if not _is_finite_number(value):
                raise ValueError(f"{name} must be a finite number")
        if not -90 <= center_lat <= 90:
            raise ValueError("center_lat must be between -90 and 90")
        if not -180 <= center_lon <= 180:
            raise ValueError("center_lon must be between -180 and 180")
        if not 0 < scan_radius_m <= 100_000:
            raise ValueError("scan_radius_m must be greater than 0 and no more than 100000 m")
        if cruise_altitude_m < 0 or scan_altitude_m < 0:
            raise ValueError("altitudes must be non-negative")
        if isinstance(num_points, bool) or not isinstance(num_points, int) or not 3 <= num_points <= MAX_SCAN_POINTS:
            raise ValueError(f"num_points must be an integer between 3 and {MAX_SCAN_POINTS}")

        mission_id = str(uuid.uuid4())
        waypoints = [{
            "seq": 0, "lat": center_lat, "lon": center_lon,
            "alt": cruise_altitude_m, "action": "waypoint",
        }]
        for i in range(num_points):
            bearing = (2 * math.pi * i) / num_points
            lat, lon = self._destination_point(center_lat, center_lon, scan_radius_m, bearing)
            waypoints.append({
                "seq": len(waypoints), "lat": lat, "lon": lon,
                "alt": scan_altitude_m, "yaw": math.degrees(bearing),
                "action": "waypoint",
            })
        waypoints.append({
            "seq": len(waypoints), "lat": center_lat, "lon": center_lon,
            "alt": cruise_altitude_m, "action": "waypoint",
        })
        waypoints.append({
            "seq": len(waypoints), "lat": center_lat, "lon": center_lon,
            "alt": 0, "action": "rtl",
        })
        return {
            "mission_id": mission_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "home": {"lat": center_lat, "lon": center_lon, "alt": 0},
            "cruise_altitude_m": cruise_altitude_m,
            "scan_altitude_m": scan_altitude_m,
            "scan_radius_m": scan_radius_m,
            "waypoints": waypoints,
        }

    @staticmethod
    def _destination_point(lat, lon, distance_m, bearing_rad):
        """Great-circle destination point using a spherical Earth approximation."""
        lat1, lon1 = math.radians(lat), math.radians(lon)
        ang_dist = distance_m / EARTH_RADIUS_M
        lat2 = math.asin(
            math.sin(lat1) * math.cos(ang_dist)
            + math.cos(lat1) * math.sin(ang_dist) * math.cos(bearing_rad)
        )
        lon2 = lon1 + math.atan2(
            math.sin(bearing_rad) * math.sin(ang_dist) * math.cos(lat1),
            math.cos(ang_dist) - math.sin(lat1) * math.sin(lat2),
        )
        # Normalize longitude to the conventional [-180, 180] range.
        lon2 = (lon2 + 3 * math.pi) % (2 * math.pi) - math.pi
        return math.degrees(lat2), math.degrees(lon2)

    def get_elevation_profile(self, center_lat, center_lon, radius_m, num_samples=12):
        """Query OpenRouteService for elevations around a scan perimeter."""
        for name, value in {
            "center_lat": center_lat, "center_lon": center_lon, "radius_m": radius_m
        }.items():
            if not _is_finite_number(value):
                raise ValueError(f"{name} must be a finite number")
        if not -90 <= center_lat <= 90 or not -180 <= center_lon <= 180:
            raise ValueError("center latitude/longitude are out of range")
        if not 0 < radius_m <= 100_000:
            raise ValueError("radius_m must be greater than 0 and no more than 100000 m")
        if isinstance(num_samples, bool) or not isinstance(num_samples, int) or not 3 <= num_samples <= MAX_SCAN_POINTS:
            raise ValueError(f"num_samples must be an integer between 3 and {MAX_SCAN_POINTS}")
        if not self.ors_api_key or requests is None:
            raise RuntimeError("ORS_API_KEY not configured or requests not installed")

        profile = []
        headers = {"Authorization": self.ors_api_key}
        for i in range(num_samples):
            bearing = (2 * math.pi * i) / num_samples
            lat, lon = self._destination_point(center_lat, center_lon, radius_m, bearing)
            response = requests.get(
                ORS_ELEVATION_URL,
                params={"geometry": f"{lon},{lat}"},
                headers=headers,
                timeout=5,
            )
            response.raise_for_status()
            coordinates = response.json().get("geometry", {}).get("coordinates", [])
            elevation = coordinates[2] if len(coordinates) >= 3 else None
            if elevation is None or not _is_finite_number(elevation):
                raise ValueError("OpenRouteService response did not contain a valid elevation")
            profile.append({"lat": lat, "lon": lon, "elevation_m": elevation})
        return profile

    def validate(self, mission: dict):
        """Validate mission structure and finite, in-range waypoint values."""
        if not isinstance(mission, dict):
            return False, "mission must be a JSON object"
        missing = REQUIRED_TOP_LEVEL_KEYS - mission.keys()
        if missing:
            return False, f"Missing top-level keys: {sorted(missing)}"
        mission_id = mission.get("mission_id")
        if not isinstance(mission_id, str) or not mission_id.strip() or len(mission_id) > 100:
            return False, "mission_id must be a non-empty string of at most 100 characters"
        waypoints = mission.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            return False, "waypoints must be a non-empty list"
        if len(waypoints) > MAX_WAYPOINTS:
            return False, f"waypoints cannot exceed {MAX_WAYPOINTS} entries"

        for i, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                return False, f"Waypoint {i} must be an object"
            missing_wp = REQUIRED_WAYPOINT_KEYS - waypoint.keys()
            if missing_wp:
                return False, f"Waypoint {i} missing keys: {sorted(missing_wp)}"
            seq = waypoint["seq"]
            if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
                return False, f"Waypoint {i} seq must be a non-negative integer"
            for key in ("lat", "lon", "alt"):
                if not _is_finite_number(waypoint[key]):
                    return False, f"Waypoint {i} {key} must be a finite number"
            if not -90 <= waypoint["lat"] <= 90:
                return False, f"Waypoint {i} latitude is out of range"
            if not -180 <= waypoint["lon"] <= 180:
                return False, f"Waypoint {i} longitude is out of range"
            if waypoint["alt"] < 0:
                return False, f"Waypoint {i} altitude must be non-negative"
            if "yaw" in waypoint and not _is_finite_number(waypoint["yaw"]):
                return False, f"Waypoint {i} yaw must be a finite number"
        return True, "ok"
