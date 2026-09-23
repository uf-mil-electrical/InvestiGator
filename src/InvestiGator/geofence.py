from collections import namedtuple
from threading import Lock

# Mirrors robocommand.common.v1.LatLng: decimal degrees, WGS84.
LatLng = namedtuple("LatLng", ["latitude", "longitude"])

# Unused: the fence is the course boundary itself, with FENCE_MARGIN on the flight controller.
# EARTH_RADIUS_M = 6_378_137.0
# M_PER_DEG_LAT = math.radians(1) * EARTH_RADIUS_M
#
#
# def inset_polygon(boundary: list[LatLng], inset_m: float = constants.GEOFENCE_INSET_M) -> list[LatLng]:
#     """
#     Shrink a closed lat/lng polygon inward by inset_m. Returns it closed (first point == last).
#     """
#     vertices = boundary[:-1] if boundary[0] == boundary[-1] else list(boundary)
#
#     lat0 = sum(vertex.latitude for vertex in vertices) / len(vertices)
#     lon0 = sum(vertex.longitude for vertex in vertices) / len(vertices)
#     m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat0))
#
#     points = [((vertex.longitude - lon0) * m_per_deg_lon, (vertex.latitude - lat0) * M_PER_DEG_LAT) for vertex in vertices]
#     edges = list(zip(points, points[1:] + points[:1]))
#
#     # Shoelace area is positive for counter-clockwise winding, which decides which side is inward.
#     inward = 1 if sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in edges) > 0 else -1
#
#     # Inward unit normal of each edge.
#     normals = []
#     for (x1, y1), (x2, y2) in edges:
#         length = math.hypot(x2 - x1, y2 - y1)
#         normals.append((-(y2 - y1) / length * inward, (x2 - x1) / length * inward))
#
#     # Vertex i joins edge i-1 (arriving) and edge i (leaving). Moving it by k * (n_arriving + n_leaving)
#     # with k = inset_m / (1 + n_arriving . n_leaving) puts it exactly inset_m inside both edges.
#     inset = []
#     for (x, y), (ax, ay), (lx, ly) in zip(points, normals[-1:] + normals[:-1], normals):
#         k = inset_m / (1 + ax * lx + ay * ly)
#         inset.append(LatLng(lat0 + (y + (ay + ly) * k) / M_PER_DEG_LAT, lon0 + (x + (ax + lx) * k) / m_per_deg_lon))
#
#     return inset + inset[:1]


class Geofence:
    """
    The course boundary received from RoboCommand (RxCourse), used directly as the UAV geofence.
    A closed lat/lng polygon (first point == last). Populated at runtime and replaced each run,
    since the boundary differs per competition course. Not static config.
    The margin inside the boundary comes from FENCE_MARGIN on the flight controller.
    """

    def __init__(self):
        self.lock = Lock()
        self.course_boundary: list[LatLng] = []

    def set_course_boundary(self, corners):
        """
        Store the course boundary. corners is RxCourse.corners objs or similar.
        """
        boundary = [LatLng(float(corner.latitude), float(corner.longitude)) for corner in corners]

        with self.lock:
            self.course_boundary = boundary

    def clear(self):
        """
        Forget the course boundary. A stale boundary from a previous course is worse than none.
        """
        with self.lock:
            self.course_boundary = []

    @property
    def uav_geofence(self) -> list[LatLng]:
        """
        The closed geofence for RunDeclaration.uav_geofence. Empty until a boundary arrives.
        """
        with self.lock:
            return list(self.course_boundary)

    @property
    def fence_vertices(self) -> list[LatLng]:
        """
        The geofence without its closing point, for the ArduPilot upload, which closes it implicitly.
        """
        with self.lock:
            return self.course_boundary[:-1]
