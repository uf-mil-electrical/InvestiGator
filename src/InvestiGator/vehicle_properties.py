from dataclasses import dataclass
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from collections import namedtuple
from threading import Lock
from pymavlink import mavutil
import math
import time

from . import constants

MavFrameGlobal = namedtuple("MavFrameGlobal", ["lattitude_int", "longitude_int", "altitude_m"])
MavFrameLocalNed = namedtuple("LocalNED", ["x_north_m", "y_east_m", "z_down_m"])
MavFrameGlobalRel = namedtuple("GlobalRelative",["lattitude_int", "longitude_int", "altitude_rel_m"])
MavFrameLocalENU = namedtuple("LocalENU", ["x_east_m", "y_north_m", "z_up_m"])
MavFrameLocalOffsetNED = namedtuple("LocalOffsetNED", ["x_north_m", "y_east_m", "z_down_m"])
MavFrameBodyFRD = namedtuple("BodyFRD", ["x_forward_m", "y_right_m", "z_down_m"])
MavFrameLocalFRD = namedtuple("LocalFRD", ["x_forward_m", "y_right_m", "z_down_m"])
MavFrameLocalFLU = namedtuple("LocalFLU", ["x_forward_m", "y_left_m", "z_up_m"])
Attitude = namedtuple("Attitude", ["roll_rad", "pitch_rad", "yaw_rad", "rollspeed_rad_s", "pitchspeed_rad_s", "yawspeed_rad_s"])
# Mirrors robocommand.common.v1.LatLng: decimal degrees, WGS84.
LatLng = namedtuple("LatLng", ["latitude", "longitude"])

# ArduCopter modes that count as autonomous for RobotState STATE_AUTO.
AUTONOMOUS_MODES = {"GUIDED"}
# ArduCopter has no mode called MANUAL: these are the modes a pilot flies. The autopilot flies the rest.
MANUAL_MODES = {"STABILIZE", "ALT_HOLD", "ACRO", "LOITER", "POSHOLD", "DRIFT", "SPORT", "FLOWHOLD"}
# ArduPilot fixes HEARTBEAT at 1 Hz and ignores SET_MESSAGE_INTERVAL for it, so 3.0 s tolerates two dropped
FC_HEARTBEAT_STALE_S = 3.0
# WGS84 semi-major axis, used for the local tangent plane the geofence inset is computed in.
EARTH_RADIUS_M = 6378137.0

class Location:
    """
    Represents location of the vehicle and provides methods to return location wrapped in different location types.
    """

    def __init__(self, vehicle):
        self.lock = Lock()

        self.lat_int = None
        self.lon_int = None
        self.alt_m = None
        self.relative_alt_m = None
        self.vx_cm_s = None
        self.vy_cm_s = None
        self.vz_cm_s = None
        self.hdg_cdeg = None
        self.alt_ellipsoid_mm = None

        self.x_north_m = None
        self.y_east_m = None
        self.z_down_m = None

        self.roll_rad = None
        self.pitch_rad = None
        self.yaw_rad = None
        self.rollspeed_rad_s = None
        self.pitchspeed_rad_s = None
        self.yawspeed_rad_s = None

        @vehicle.subscribe(mavlink.MAVLink_global_position_int_message.msgname)
        def update_global_position(message: mavlink.MAVLink_global_position_int_message):
            with self.lock:
                self.lat_int = message.lat # Divide by 1E7 to convert to degrees
                self.lon_int = message.lon # Divide by 1E7 to convert to degrees
                self.alt_m = message.alt / 1E3 # Given in mm
                self.relative_alt_m = message.relative_alt / 1E3  # Given in mm
                self.vx_cm_s = message.vx  # Ground X speed (latitude, positive north)
                self.vy_cm_s = message.vy  # Ground Y speed (longitude, positive east)
                self.vz_cm_s = message.vz  # Ground Z speed (altitude, positive down)
                self.hdg_cdeg = message.hdg  # Centidegrees, 65535 if unknown

        @vehicle.subscribe(mavlink.MAVLink_gps_raw_int_message.msgname)
        def update_gps_raw(message: mavlink.MAVLink_gps_raw_int_message):
            if message.get_srcSystem() != 1:
                return
            with self.lock:
                # Height above WGS84 ellipsoid in mm. MAVLink2 extension field, 0 on MAVLink1.
                self.alt_ellipsoid_mm = message.alt_ellipsoid

        @vehicle.subscribe(mavlink.MAVLink_local_position_ned_message.msgname)
        def update_local_position(message: mavlink.MAVLink_local_position_ned_message):
            with self.lock:
                self.x_north_m = message.x
                self.y_east_m = message.y
                self.z_down_m = message.z  # Negative altitude

        @vehicle.subscribe(mavlink.MAVLink_attitude_message.msgname)
        def update_attitude(message: mavlink.MAVLink_attitude_message):
            with self.lock:
                self.roll_rad = message.roll
                self.pitch_rad = message.pitch
                self.yaw_rad = message.yaw
                self.rollspeed_rad_s = message.rollspeed
                self.pitchspeed_rad_s = message.pitchspeed
                self.yawspeed_rad_s = message.yawspeed

    @property
    def global_frame(self):
        """
        Returns location as a MAV_FRAME_GLOBAL frame. Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
        """
        with self.lock:
            return MavFrameGlobal(self.lat_int, self.lon_int, self.alt_m)
        
    @property
    def global_frame_relative(self):
        """
        Returns location as a MAV_FRAME_GLOBAL frame. Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
        """
        with self.lock:
            return MavFrameGlobalRel(self.lat_int, self.lon_int, self.relative_alt_m)

    @property
    def local_ned(self):
        """"
        NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
        """
        with self.lock:
            return MavFrameLocalNed(self.x_north_m, self.y_east_m, self.z_down_m)
    
    @property
    def ground_speed_mps(self):
        """
        Horizontal speed over ground in m/s from GLOBAL_POSITION_INT velocity.
        """
        with self.lock:
            if self.vx_cm_s is None or self.vy_cm_s is None:
                return None
            return math.hypot(self.vx_cm_s, self.vy_cm_s) / 100.0

    @property
    def heading_deg(self):
        """
        Vehicle heading in degrees [0, 360) from GLOBAL_POSITION_INT. None if unknown.
        """
        with self.lock:
            if self.hdg_cdeg is None or self.hdg_cdeg == 65535:  # 65535 is the MAVLink "unknown" sentinel
                return None
            return self.hdg_cdeg / 100.0

    @property
    def altitude_hae_m(self):
        """
        Altitude above the WGS84 ellipsoid (HAE) in meters from GPS_RAW_INT. None if not reported.
        Deliberately no fallback to GLOBAL_POSITION_INT.alt, which is AMSL (a different datum).
        """
        with self.lock:
            if not self.alt_ellipsoid_mm:
                return None
            return self.alt_ellipsoid_mm / 1000.0

    @property
    def body_frd(self):
        """
        FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle.
        """
        #TODO: Create a subscriber for attitude. Calculate FRD position from Local NED and Orientatio.
        pass

    @property
    def attitude(self):
        """
        The attitude in the aeronautical frame (right-handed, Z-down, Y-right, X-front, ZYX, intrinsic)
        Roll, Pitch, Yaw in (-pi, pi)
        """
        with self.lock:
            return Attitude(self.roll_rad, self.pitch_rad, self.yaw_rad, self.rollspeed_rad_s, self.pitchspeed_rad_s, self.yawspeed_rad_s)


class Status:
    """
    Information received from the vehicle's heartbeat and system status messages.
    """

    def __init__(self, vehicle):
        self.lock = Lock()

        # Heartbeat Attributes
        self.mode_map_bynumber = mavutil.mode_mapping_bynumber(mavlink.MAV_TYPE_QUADROTOR)
        self.mode_map_byname = mavutil.mode_mapping_byname(mavlink.MAV_TYPE_QUADROTOR)
        self.type = None
        self.autopilot = None
        self.base_mode = None
        self.custom_mode = None
        self.system_status = None
        self.heartbeat_t = None  # Receipt time of the last autopilot heartbeat

        # System Status Attributes
        self.onboard_control_sensors_present = None
        self.onboard_control_sensors_enabled = None
        self.onboard_control_sensors_health = None
        self.load = None
        self.voltage_battery = None
        self.current_battery = None
        self.battery_remaining = None
        self.drop_rate_comm = None
        self.errors_comm = None
        self.errors_count1 = None
        self.errors_count2 = None
        self.errors_count3 = None
        self.errors_count4 = None

        # Extended System State Attributes
        self.landed_state = None

        @vehicle.subscribe(mavlink.MAVLink_heartbeat_message.msgname)
        def subscription_update(message: mavlink.MAVLink_heartbeat_message):
            # TODO: Make this reflect the configurable source system from vehiclemanager
            if message.get_srcSystem() != 1:
                return
            
            with self.lock:
                self.type = message.type
                self.autopilot = message.autopilot
                self.base_mode = message.base_mode
                self.custom_mode = message.custom_mode
                self.system_status = message.system_status
                if message.get_srcComponent() == mavlink.MAV_COMP_ID_AUTOPILOT1:
                    self.heartbeat_t = time.monotonic()

        @vehicle.subscribe(mavlink.MAVLink_sys_status_message.msgname)
        def on_sys_status(message: mavlink.MAVLink_sys_status_message):
            if message.get_srcSystem() != 1:
                return
            with self.lock:
                self.onboard_control_sensors_present = message.onboard_control_sensors_present
                self.onboard_control_sensors_enabled = message.onboard_control_sensors_enabled
                self.onboard_control_sensors_health = message.onboard_control_sensors_health
                self.load = message.load
                self.voltage_battery = message.voltage_battery
                self.current_battery = message.current_battery
                self.battery_remaining = message.battery_remaining
                self.drop_rate_comm = message.drop_rate_comm
                self.errors_comm = message.errors_comm
                self.errors_count1 = message.errors_count1
                self.errors_count2 = message.errors_count2
                self.errors_count3 = message.errors_count3
                self.errors_count4 = message.errors_count4

        @vehicle.subscribe(mavlink.MAVLink_extended_sys_state_message.msgname)
        def update_landed_state(message: mavlink.MAVLink_extended_sys_state_message):
            if message.get_srcSystem() != 1:
                return
            with self.lock:
                self.landed_state = message.landed_state


    @property
    def armed(self):
        with self.lock:
            if self.base_mode is None:
                return False
            return bool(self.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    
    def disarmed(self):
        with self.lock:
            return not self.armed()

    @property
    def prearmed(self):
        with self.lock:
            if self.onboard_control_sensors_health is None:
                return False
            return bool(self.onboard_control_sensors_health & mavlink.MAV_SYS_STATUS_PREARM_CHECK)
        
    @property
    def mode_string(self):
        with self.lock:
            if self.custom_mode is None or self.mode_map_bynumber is None:
                return None
            return self.mode_map_bynumber.get(self.custom_mode)

    @property
    def flight_phase(self):
        """
        Map MAV_LANDED_STATE to FlightPhase (constants.FLIGHT_PHASE_*).
        """
        with self.lock:
            match self.landed_state:
                case mavlink.MAV_LANDED_STATE_ON_GROUND:
                    return constants.FLIGHT_PHASE_GROUNDED
                case mavlink.MAV_LANDED_STATE_IN_AIR | mavlink.MAV_LANDED_STATE_TAKEOFF | mavlink.MAV_LANDED_STATE_LANDING:
                    return constants.FLIGHT_PHASE_AIRBORNE
                case _:
                    return constants.FLIGHT_PHASE_UNKNOWN

    @property
    def robot_state(self):
        """
        RobotX Heartbeat.state from the latest autopilot heartbeat. Recomputed on every read, never latched.
        KILLED when the autopilot heartbeat is stale, or when disarmed outside a pilot mode.
        MANUAL when the flight mode is one a pilot flies. AUTO when armed and in GUIDED.
        Armed in any other autopilot-flown mode (AUTO, RTL, LAND) falls back to MANUAL, so UNKNOWN is never reported.
        STATE_AUTO gates RunStart, so it must not be reported early.
        """
        with self.lock:
            fresh = self.heartbeat_t is not None and time.monotonic() - self.heartbeat_t < FC_HEARTBEAT_STALE_S
            armed = self.base_mode is not None and bool(self.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = self.mode_map_bynumber.get(self.custom_mode)

            # Checked first so a kill always wins: stale autopilot data, or disarmed outside a pilot mode.
            if not fresh or (not armed and mode not in MANUAL_MODES):
                return constants.ROBOT_STATE_KILLED

            if mode in MANUAL_MODES:
                return constants.ROBOT_STATE_MANUAL

            if armed and mode in AUTONOMOUS_MODES:
                # TODO(open decision): "ready" is armed + GUIDED for now. Any extra check (e.g. 3D GPS fix,
                # holding position) goes here.
                return constants.ROBOT_STATE_AUTO

            # Default to "MANUAL" state if armed in an unrecognized autopilot mode
            # In practice: should not happen!
            return constants.ROBOT_STATE_MANUAL


def open_polygon(polygon: list[LatLng]) -> list[LatLng]:
    """
    Drop the duplicated closing point from a closed polygon.
    RoboCommand sends and expects closed polygons. ArduPilot's polygon fence closes implicitly, so
    sending the duplicate would store a zero-length edge.
    """
    if len(polygon) >= 2 and polygon[0] == polygon[-1]:
        return list(polygon[:-1])
    return list(polygon)


def close_polygon(vertices: list[LatLng]) -> list[LatLng]:
    """
    Append the first point to the end, producing the closed polygon RoboCommand requires.
    """
    if len(vertices) >= 2 and vertices[0] == vertices[-1]:
        return list(vertices)
    return list(vertices) + [vertices[0]]


def validate_closed_polygon(polygon: list[LatLng]):
    """
    Raise ValueError unless polygon is a closed polygon that ArduPilot will accept as a fence.
    Checked before anything is sent to RoboCommand or uploaded to the flight controller.
    """
    if polygon[0] != polygon[-1]:
        raise ValueError(f"Polygon is not closed: first point {polygon[0]} != last point {polygon[-1]}.")

    vertices = open_polygon(polygon)

    if len(vertices) < constants.FENCE_MIN_VERTICES:
        raise ValueError(f"Polygon has {len(vertices)} distinct vertices, fewer than the {constants.FENCE_MIN_VERTICES} ArduPilot requires.")

    if len(vertices) > constants.FENCE_MAX_VERTICES:
        raise ValueError(f"Polygon has {len(vertices)} vertices, more than the {constants.FENCE_MAX_VERTICES} ArduPilot can store.")

    for index, vertex in enumerate(vertices):
        if vertex == vertices[index - 1]:
            raise ValueError(f"Polygon repeats vertex {vertex} at index {index}.")

    if _signed_area_m2(_to_local_m(vertices)) == 0.0:
        raise ValueError("Polygon is degenerate: all vertices are collinear.")


def _to_local_m(vertices: list[LatLng]) -> list[tuple[float, float]]:
    """
    Project lat/lng onto a local ENU tangent plane in meters, centered on the mean of the vertices.
    Equirectangular, which is accurate to well under the 2 m inset over a RobotX-sized course.
    """
    lat0_deg = sum(vertex.latitude for vertex in vertices) / len(vertices)
    lon0_deg = sum(vertex.longitude for vertex in vertices) / len(vertices)
    cos_lat0 = math.cos(math.radians(lat0_deg))

    return [(math.radians(vertex.longitude - lon0_deg) * EARTH_RADIUS_M * cos_lat0,
             math.radians(vertex.latitude - lat0_deg) * EARTH_RADIUS_M) for vertex in vertices]


def _to_latlng(points_m: list[tuple[float, float]], reference: list[LatLng]) -> list[LatLng]:
    """
    Inverse of _to_local_m. reference must be the vertex list the plane was built from.
    """
    lat0_deg = sum(vertex.latitude for vertex in reference) / len(reference)
    lon0_deg = sum(vertex.longitude for vertex in reference) / len(reference)
    cos_lat0 = math.cos(math.radians(lat0_deg))

    return [LatLng(lat0_deg + math.degrees(y_m / EARTH_RADIUS_M),
                   lon0_deg + math.degrees(x_m / (EARTH_RADIUS_M * cos_lat0))) for x_m, y_m in points_m]


def _signed_area_m2(points_m: list[tuple[float, float]]) -> float:
    """
    Shoelace area. Positive when the vertices wind counter-clockwise, which gives the inset direction.
    """
    area = 0.0
    for index, (x1_m, y1_m) in enumerate(points_m):
        x2_m, y2_m = points_m[(index + 1) % len(points_m)]
        area += x1_m * y2_m - x2_m * y1_m

    return area / 2.0


def _point_in_polygon(point_m: tuple[float, float], points_m: list[tuple[float, float]]) -> bool:
    """
    Ray casting crossing count. Used to confirm the inset polygon really lies inside the boundary.
    """
    x_m, y_m = point_m
    inside = False

    for index, (x1_m, y1_m) in enumerate(points_m):
        x2_m, y2_m = points_m[(index + 1) % len(points_m)]
        if (y1_m > y_m) != (y2_m > y_m):
            crossing_x_m = x1_m + (y_m - y1_m) * (x2_m - x1_m) / (y2_m - y1_m)
            if x_m < crossing_x_m:
                inside = not inside

    return inside


def _distance_to_edges_m(point_m: tuple[float, float], points_m: list[tuple[float, float]]) -> float:
    """
    Shortest distance from a point to any edge of a polygon, used to measure a realised inset.
    """
    x_m, y_m = point_m
    shortest_m = math.inf

    for index, (x1_m, y1_m) in enumerate(points_m):
        x2_m, y2_m = points_m[(index + 1) % len(points_m)]
        edge_x_m, edge_y_m = x2_m - x1_m, y2_m - y1_m
        # Clamped projection onto the edge, so corners are measured to the vertex and not past it.
        along = ((x_m - x1_m) * edge_x_m + (y_m - y1_m) * edge_y_m) / (edge_x_m ** 2 + edge_y_m ** 2)
        along = max(0.0, min(1.0, along))
        shortest_m = min(shortest_m, math.hypot(x_m - (x1_m + along * edge_x_m), y_m - (y1_m + along * edge_y_m)))

    return shortest_m


def _edges_cross(first_m, second_m) -> bool:
    """
    True if two line segments properly cross. Endpoint touches do not count, so edges that merely
    share a vertex are not treated as a crossing.
    """
    (ax_m, ay_m), (bx_m, by_m) = first_m
    (cx_m, cy_m), (dx_m, dy_m) = second_m

    def side(px_m, py_m, qx_m, qy_m, rx_m, ry_m):
        return (qx_m - px_m) * (ry_m - py_m) - (qy_m - py_m) * (rx_m - px_m)

    d1 = side(ax_m, ay_m, bx_m, by_m, cx_m, cy_m)
    d2 = side(ax_m, ay_m, bx_m, by_m, dx_m, dy_m)
    d3 = side(cx_m, cy_m, dx_m, dy_m, ax_m, ay_m)
    d4 = side(cx_m, cy_m, dx_m, dy_m, bx_m, by_m)

    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _is_simple_polygon(points_m: list[tuple[float, float]]) -> bool:
    """
    True if no two non-adjacent edges cross. A miter inset of a concave polygon can fold over itself,
    and ArduPilot would accept the result without complaint.
    """
    count = len(points_m)
    edges = [(points_m[index], points_m[(index + 1) % count]) for index in range(count)]

    for first in range(count):
        for second in range(first + 1, count):
            # Skip edges that share a vertex: consecutive edges, and the first/last pair.
            if second == first + 1 or (first == 0 and second == count - 1):
                continue
            if _edges_cross(edges[first], edges[second]):
                return False

    return True


def inset_polygon(boundary: list[LatLng], inset_m: float = constants.GEOFENCE_INSET_M) -> list[LatLng]:
    """
    Shrink a closed polygon inward by inset_m and return the result as a closed polygon.

    Each edge is offset toward the interior in a local metric frame and consecutive offset edges are
    intersected, so a vertex stays a vertex (a miter join). Works for any simple polygon; it does not
    assume the boundary is a convex quadrilateral.
    """
    validate_closed_polygon(boundary)

    vertices = open_polygon(boundary)
    points_m = _to_local_m(vertices)
    area_m2 = _signed_area_m2(points_m)
    inward = 1.0 if area_m2 > 0 else -1.0

    # Offset every edge inward, keeping a point on the offset line and the edge's unit direction.
    offsets = []
    for index, (x1_m, y1_m) in enumerate(points_m):
        x2_m, y2_m = points_m[(index + 1) % len(points_m)]
        length_m = math.hypot(x2_m - x1_m, y2_m - y1_m)
        unit_x, unit_y = (x2_m - x1_m) / length_m, (y2_m - y1_m) / length_m
        normal_x, normal_y = -unit_y * inward, unit_x * inward
        offsets.append(((x1_m + normal_x * inset_m, y1_m + normal_y * inset_m), (unit_x, unit_y)))

    # Each new vertex is where the offsets of the two edges meeting at that vertex cross.
    inset_points_m = []
    for index in range(len(points_m)):
        (arriving_x_m, arriving_y_m), (arriving_dx, arriving_dy) = offsets[index - 1]
        (leaving_x_m, leaving_y_m), (leaving_dx, leaving_dy) = offsets[index]

        cross = arriving_dx * leaving_dy - arriving_dy * leaving_dx
        if abs(cross) < 1E-12:
            # The edges are collinear, so the two offset lines are the same line.
            inset_points_m.append((leaving_x_m, leaving_y_m))
            continue

        distance_m = ((leaving_x_m - arriving_x_m) * leaving_dy - (leaving_y_m - arriving_y_m) * leaving_dx) / cross
        inset_points_m.append((arriving_x_m + arriving_dx * distance_m, arriving_y_m + arriving_dy * distance_m))

    # Offset lines still intersect after the polygon has been shrunk past nothing, and for a convex
    # boundary the result is even wound correctly, so the area alone proves nothing. Measure the
    # inset that was actually achieved instead.
    inset_area_m2 = _signed_area_m2(inset_points_m)
    if abs(inset_area_m2) < 1E-6 or abs(inset_area_m2) >= abs(area_m2):
        raise ValueError(f"A {inset_m} m inset collapsed the boundary: area went from {abs(area_m2):.1f} m2 to {abs(inset_area_m2):.1f} m2.")

    for index, inset_point_m in enumerate(inset_points_m):
        if not _point_in_polygon(inset_point_m, points_m):
            raise ValueError(f"A {inset_m} m inset put vertex {index} outside the boundary. The boundary is too narrow or too concave for a miter inset.")

        achieved_m = _distance_to_edges_m(inset_point_m, points_m)
        if achieved_m < inset_m - 1E-3:
            raise ValueError(f"A {inset_m} m inset left vertex {index} only {achieved_m:.2f} m inside the boundary. The boundary is too narrow to inset by {inset_m} m.")

    if not _is_simple_polygon(inset_points_m):
        raise ValueError(f"A {inset_m} m inset made the boundary self-intersecting. It is too concave to inset by {inset_m} m.")

    return close_polygon(_to_latlng(inset_points_m, vertices))


class Geofence:
    """
    The course boundary received from RoboCommand and the UAV geofence derived from it.

    Both are closed polygons in decimal degrees.
    """

    def __init__(self):
        self.lock = Lock()
        self.course_boundary: list[LatLng] = []
        self.geofence: list[LatLng] = []

    def set_course_boundary(self, corners) -> list[LatLng]:
        """
        Store the RxCourse boundary, derive the inset UAV geofence, and return the geofence.
        corners is any sequence with .latitude and .longitude in degrees, including RxCourse.corners.
        Raises ValueError without storing anything if the boundary or the derived geofence is invalid.
        """
        boundary = [LatLng(float(corner.latitude), float(corner.longitude)) for corner in corners]
        validate_closed_polygon(boundary)
        geofence = inset_polygon(boundary)
        validate_closed_polygon(geofence)

        with self.lock:
            self.course_boundary = boundary
            self.geofence = geofence

        return geofence

    def clear(self):
        """
        Forget the course boundary. A stale boundary from a previous course is worse than none.
        """
        with self.lock:
            self.course_boundary = []
            self.geofence = []

    @property
    def uav_geofence(self) -> list[LatLng]:
        """
        The closed geofence polygon for RunDeclaration.uav_geofence. Empty until a boundary arrives.
        """
        with self.lock:
            return list(self.geofence)

    @property
    def fence_vertices(self) -> list[LatLng]:
        """
        The geofence vertices for the ArduPilot polygon upload, closing duplicate removed.
        """
        with self.lock:
            return open_polygon(self.geofence)
