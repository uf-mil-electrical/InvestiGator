import math
from collections import namedtuple
from threading import Lock

from . import constants

# Mirrors robocommand.common.v1.LatLng: decimal degrees, WGS84.
LatLng = namedtuple("LatLng", ["latitude", "longitude"])

EARTH_RADIUS_M = 6_378_137.0
M_PER_DEG_LAT = math.radians(1) * EARTH_RADIUS_M


def inset_polygon(boundary: list[LatLng], inset_m: float = constants.GEOFENCE_INSET_M) -> list[LatLng]:
    """
    Shrink a closed lat/lng polygon inward by inset_m. Returns it closed (first point == last),
    or [] if it has a repeated vertex, a hairpin, or is too small to shrink that far.
    """
    vertices = boundary[:-1] if boundary and boundary[0] == boundary[-1] else list(boundary)
    if len(vertices) < MIN_POLYGON_VERTICES:
        return []

    lat0 = sum(vertex.latitude for vertex in vertices) / len(vertices)
    lon0 = sum(vertex.longitude for vertex in vertices) / len(vertices)
    m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat0))

    points = [((vertex.longitude - lon0) * m_per_deg_lon, (vertex.latitude - lat0) * M_PER_DEG_LAT) for vertex in vertices]
    edges = list(zip(points, points[1:] + points[:1]))

    # Shoelace area is positive for counter-clockwise winding, which decides which side is inward.
    inward = 1 if sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in edges) > 0 else -1

    # Inward unit normal of each edge.
    normals = []
    for (x1, y1), (x2, y2) in edges:
        length = math.hypot(x2 - x1, y2 - y1)
        if length == 0:
            return []
        normals.append((-(y2 - y1) / length * inward, (x2 - x1) / length * inward))

    # Vertex i joins edge i-1 (arriving) and edge i (leaving). Moving it by k * (n_arriving + n_leaving)
    # with k = inset_m / (1 + n_arriving . n_leaving) puts it exactly inset_m inside both edges.
    moved = []
    for (x, y), (ax, ay), (lx, ly) in zip(points, normals[-1:] + normals[:-1], normals):
        dot = ax * lx + ay * ly
        if 1 + dot < 1e-9:
            return []
        k = inset_m / (1 + dot)
        moved.append((x + (ax + lx) * k, y + (ay + ly) * k))

    # Too small: an edge shrank past zero and reversed.
    for ((x1, y1), (x2, y2)), (m1, m2) in zip(edges, zip(moved, moved[1:] + moved[:1])):
        if (m2[0] - m1[0]) * (x2 - x1) + (m2[1] - m1[1]) * (y2 - y1) <= 0:
            return []

    inset = [LatLng(lat0 + y / M_PER_DEG_LAT, lon0 + x / m_per_deg_lon) for x, y in moved]
    return inset + inset[:1]


# ArduPilot's per-polygon vertex limits.
MIN_POLYGON_VERTICES = 3
MAX_POLYGON_VERTICES = 255


class Geofence:
    """
    The course boundary received from RoboCommand (RxCourse), and the UAV geofence: that boundary
    inset GEOFENCE_INSET_M. Both closed lat/lng polygons (first point == last). Populated at runtime
    and replaced each run, since the boundary differs per competition course. Not static config.
    """

    def __init__(self):
        self.lock = Lock()
        self.course_boundary: list[LatLng] = []
        self.fence_boundary: list[LatLng] = []
        self.exclusion_zones: list[list[LatLng]] = []
        # KeepOutZone areas (circles) as (center, radius_m). Kept until AllClear, across boundary changes.
        self.keep_out_areas: list[tuple[LatLng, float]] = []

    def set_course_boundary(self, corners) -> bool:
        """
        Store the course boundary and the geofence inset inside it. corners is RxCourse.corners objs or similar.
        Returns False, storing nothing, if the boundary can't be inset.
        """
        boundary = [LatLng(float(corner.latitude), float(corner.longitude)) for corner in corners]

        fence = inset_polygon(boundary)
        if not fence:
            print(f"Course boundary can't be inset {constants.GEOFENCE_INSET_M} m: a repeated vertex, a hairpin, or too small.")
            return False

        with self.lock:
            self.course_boundary = boundary
            self.fence_boundary = fence
        return True

    def set_exclusion_zones(self, zones) -> bool:
        """
        Replace the exclusion zones. Each zone is a list of lat/lng vertices, closed or open.
        Returns False, storing nothing, if any zone is outside the vertex limits.
        """
        polygons = []

        for index, zone in enumerate(zones):
            vertices = [LatLng(float(vertex.latitude), float(vertex.longitude)) for vertex in zone]

            # Stored open, like fence_vertices.
            if len(vertices) > 1 and vertices[0] == vertices[-1]:
                vertices = vertices[:-1]

            if not MIN_POLYGON_VERTICES <= len(vertices) <= MAX_POLYGON_VERTICES:
                print(f"Exclusion zone {index} has {len(vertices)} vertices. Needs {MIN_POLYGON_VERTICES} to {MAX_POLYGON_VERTICES}.")
                return False

            polygons.append(vertices)

        with self.lock:
            self.exclusion_zones = polygons
        return True

    def clear(self):
        """
        Forget the course boundary and exclusion zones. A stale boundary from a previous course is worse than none.
        """
        with self.lock:
            self.course_boundary = []
            self.fence_boundary = []
            self.exclusion_zones = []

    def add_keep_out(self, center: LatLng, radius_m: float) -> bool:
        """
        Add a keep-out area. False if the same area is already active.
        """
        with self.lock:
            if (center, radius_m) in self.keep_out_areas:
                return False
            self.keep_out_areas.append((center, radius_m))
            return True

    def clear_keep_outs(self) -> bool:
        """
        Remove every keep-out area. False if there were none.
        """
        with self.lock:
            had_areas = bool(self.keep_out_areas)
            self.keep_out_areas = []
            return had_areas

    @property
    def exclusion_areas(self) -> list[tuple[LatLng, float]]:
        with self.lock:
            return list(self.keep_out_areas)

    def contains(self, latitude: float, longitude: float) -> bool:
        """
        True if the point is inside the geofence (ray casting in degrees, fine over a course).
        """
        with self.lock:
            vertices = self.fence_boundary[:-1]
        inside = False
        for (lat1, lon1), (lat2, lon2) in zip(vertices, vertices[1:] + vertices[:1]):
            if (lat1 > latitude) != (lat2 > latitude):
                if longitude < lon1 + (latitude - lat1) * (lon2 - lon1) / (lat2 - lat1):
                    inside = not inside
        return inside

    @property
    def uav_geofence(self) -> list[LatLng]:
        """
        The closed geofence for RunDeclaration.uav_geofence. Empty until a boundary arrives.
        """
        with self.lock:
            return list(self.fence_boundary)

    @property
    def fence_vertices(self) -> list[LatLng]:
        """
        The geofence without its closing point, for the ArduPilot upload, which closes it implicitly.
        """
        with self.lock:
            return self.fence_boundary[:-1]

    @property
    def exclusion_vertices(self) -> list[list[LatLng]]:
        """
        Exclusion zones without closing points, for the ArduPilot upload.
        """
        with self.lock:
            return [list(zone) for zone in self.exclusion_zones]
