from collections import namedtuple
from threading import Lock
import math

from . import constants

# Mirrors robocommand.common.v1.LatLng: decimal degrees, WGS84.
LatLng = namedtuple("LatLng", ["latitude", "longitude"])

# WGS84 semi-major axis, for the local metric plane the inset is computed in.
EARTH_RADIUS_M = 6_378_137.0



def inset_polygon(boundary: list[LatLng], inset_m: float = constants.GEOFENCE_INSET_M) -> list[LatLng]:
    """
    Shrink a closed lat/lng polygon inward by inset_m. Returns it closed (first point == last).
    """
    vertices = boundary[:-1] if boundary[0] == boundary[-1] else list(boundary)

    lat0 = sum(vertex.latitude for vertex in vertices) / len(vertices)
    lon0 = sum(vertex.longitude for vertex in vertices) / len(vertices)
    m_per_deg_lat = math.radians(1) * EARTH_RADIUS_M
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(lat0))

    points = [((vertex.longitude - lon0) * m_per_deg_lon, (vertex.latitude - lat0) * m_per_deg_lat) for vertex in vertices]
    edges = list(zip(points, points[1:] + points[:1]))

    # Shoelace area is positive for counter-clockwise winding, which decides which side is inward.
    inward = 1 if sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in edges) > 0 else -1

    # Inward unit normal of each edge.
    normals = []
    for (x1, y1), (x2, y2) in edges:
        length = math.hypot(x2 - x1, y2 - y1)
        normals.append((-(y2 - y1) / length * inward, (x2 - x1) / length * inward))

    # Vertex i joins edge i-1 (arriving) and edge i (leaving). Moving it by k * (n_arriving + n_leaving)
    # with k = inset_m / (1 + n_arriving . n_leaving) puts it exactly inset_m inside both edges.
    inset = []
    for (x, y), (ax, ay), (lx, ly) in zip(points, normals[-1:] + normals[:-1], normals):
        k = inset_m / (1 + ax * lx + ay * ly)
        inset.append(LatLng(lat0 + (y + (ay + ly) * k) / m_per_deg_lat, lon0 + (x + (ax + lx) * k) / m_per_deg_lon))

    return inset + inset[:1]


class Geofence:
    """
    The course boundary received from RoboCommand (RxCourse) and the UAV geofence derived from it.
    Both are closed lat/lng polygons (first point == last). Populated at runtime and replaced each
    run, since the boundary differs per competition course. Neither is static config.
    """

    def __init__(self):
        self.lock = Lock()
        
        # Original boundary from RoboCommand
        self.course_boundary: list[LatLng] = []

        # The inset polygon for the UAV (provides padding for GPS error)
        self.geofence: list[LatLng] = []

    def set_course_boundary(self, corners):
        """
        Store the course boundary and derive the geofence from it. corners is any sequence of objects
        with .latitude and .longitude in degrees, such as RxCourse.corners.
        """
        boundary = [LatLng(float(corner.latitude), float(corner.longitude)) for corner in corners]
        geofence = inset_polygon(boundary)

        with self.lock:
            self.course_boundary, self.geofence = boundary, geofence

    def clear(self):
        """
        Forget the course boundary. A stale boundary from a previous course is worse than none.
        """
        with self.lock:
            self.course_boundary, self.geofence = [], []

    @property
    def uav_geofence(self) -> list[LatLng]:
        """
        The closed geofence for RunDeclaration.uav_geofence. Empty until a boundary arrives.
        """
        with self.lock:
            return list(self.geofence)

    @property
    def fence_vertices(self) -> list[LatLng]:
        """
        The geofence without its closing point, for the ArduPilot upload, which closes it implicitly.
        """
        with self.lock:
            return self.geofence[:-1]
