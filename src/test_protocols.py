"""
Ground station check of what actually arrives over the MAVLink link: SITL's UDP port with --sim, or the
RFD900x radio without it. Read-only: it listens, reports message rates and field sanity, then reads back
the fence and FENCE_* parameters stored on the flight controller.

Run it instead of ground_control.py, never alongside it: both open the same connection.
"""
from InvestiGator import MAVConnection, constants
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from config import load_config
from gc_helpers import configure_messages
from queue import Queue, Empty
from threading import Lock
import argparse
import math
import time

AUTOPILOT = (1, mavlink.MAV_COMP_ID_AUTOPILOT1)
RPI = (1, mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
FENCE = mavlink.MAV_MISSION_TYPE_FENCE
M_PER_DEG_LAT = 111_320.0

# (source, message, minimum Hz). Minimums sit about 20% under the requested rate to allow for radio loss.
EXPECTED_RATES = [
    (AUTOPILOT, "HEARTBEAT", 0.8),            # ArduPilot fixes it at 1 Hz
    (RPI, "HEARTBEAT", 1.6),            # 2 Hz, mavconnection.py
    (AUTOPILOT, "GPS_RAW_INT", 1.6),          # 2 Hz, gc_helpers.configure_messages
    (AUTOPILOT, "GLOBAL_POSITION_INT", 4.0),  # 5 Hz
    (AUTOPILOT, "ATTITUDE", 4.0),             # 5 Hz
    (AUTOPILOT, "SYS_STATUS", 4.0),           # 5 Hz
    (AUTOPILOT, "EXTENDED_SYS_STATE", 0.8),   # 1 Hz
]

EXPECTED_PARAMS = {
    "FENCE_ENABLE": 1,
    "FENCE_TYPE": constants.FENCE_TYPE_UAV,
    "FENCE_ACTION": constants.FENCE_ACTION_RTL,
    "FENCE_MARGIN": constants.FENCE_MARGIN_M,
    "FENCE_ALT_MAX": constants.FENCE_ALT_MAX_M,
    "FENCE_ALT_MAX_TP": constants.FENCE_ALT_FRAME_AMSL,
}

failures = 0


def report(passed: bool, text: str):
    global failures
    failures += not passed
    print(f"  {'PASS' if passed else 'FAIL'}  {text}")


def listen(connection: MAVConnection, duration_s: float):
    """
    Record every message from every source for duration_s. Prints autopilot STATUSTEXT as it arrives.
    Returns message counts per (source, type), the last message of each, and landed_state changes.
    """
    lock = Lock()
    counts, latest, landed_states = {}, {}, []
    start = time.monotonic()

    def record(message):
        key = ((message.get_srcSystem(), message.get_srcComponent()), message.get_type())
        with lock:
            counts[key] = counts.get(key, 0) + 1
            latest[key] = message
            if key == (AUTOPILOT, "EXTENDED_SYS_STATE") and (not landed_states or landed_states[-1][1] != message.landed_state):
                landed_states.append((time.monotonic() - start, message.landed_state))
        if key == (AUTOPILOT, "STATUSTEXT"):
            print(f"  [autopilot] {message.text}")

    for message_class in mavlink.mavlink_map.values():
        connection.subscribe(message_class.msgname)(record)

    print(f"\nListening for {duration_s:.0f} s...")
    time.sleep(duration_s)

    with lock:
        return dict(counts), dict(latest), list(landed_states)


def request(connection: MAVConnection, send, message_type: str, match, timeout_s=2.0, retries=3):
    """
    Call send() and return the first message_type from the autopilot that satisfies match, retrying
    send() up to retries times. Returns None if nothing matching arrives.
    """
    replies = Queue()

    def on_reply(message):
        if (message.get_srcSystem(), message.get_srcComponent()) == AUTOPILOT and match(message):
            replies.put(message)

    connection.subscribe(message_type)(on_reply)
    try:
        for _ in range(retries):
            send()
            try:
                return replies.get(timeout=timeout_s)
            except Empty:
                continue
        return None
    finally:
        connection.sub_manager.unsubscribe(message_type, on_reply)


def read_fence(connection: MAVConnection):
    """
    Download the fence stored on the flight controller with the mission protocol: MISSION_REQUEST_LIST,
    then MISSION_REQUEST_INT per item. Returns the MISSION_ITEM_INTs, or None if the download failed.
    """
    count = request(connection, lambda: connection.mav.mission_request_list_send(*AUTOPILOT, mission_type=FENCE),
                    "MISSION_COUNT", lambda message: message.mission_type == FENCE)
    if count is None:
        return None

    items = []
    for seq in range(count.count):
        item = request(connection, lambda: connection.mav.mission_request_int_send(*AUTOPILOT, seq, mission_type=FENCE),
                       "MISSION_ITEM_INT", lambda message: message.mission_type == FENCE and message.seq == seq)
        if item is None:
            return None
        items.append(item)

    connection.mav.mission_ack_send(*AUTOPILOT, mavlink.MAV_MISSION_ACCEPTED, mission_type=FENCE)
    return items


def to_local_m(lat, lon, lat0, lon0):
    """
    Metres east and north of (lat0, lon0). Equirectangular, fine over a course.
    """
    return (lon - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0)), (lat - lat0) * M_PER_DEG_LAT


def check_telemetry(counts, latest, landed_states, duration_s):
    print(f"\n== Messages received in {duration_s:.0f} s ==")
    for (source, message_type), count in sorted(counts.items()):
        print(f"  sys {source[0]:>3} comp {source[1]:>3}  {message_type:<28} {count / duration_s:6.1f} Hz")

    print("\n== Rates ==")
    for source, message_type, minimum_hz in EXPECTED_RATES:
        hz = counts.get((source, message_type), 0) / duration_s
        name = "RPI" if source == RPI else "autopilot"
        report(hz >= minimum_hz, f"{name} {message_type}: {hz:.1f} Hz (need >= {minimum_hz})")

    print("\n== Fields ==")
    heartbeat = latest.get((RPI, "HEARTBEAT"))
    if heartbeat:
        report(constants.RX_TASK_NONE <= heartbeat.custom_mode <= constants.RX_TASK_DYNAMIC_INCIDENT, f"RPI HEARTBEAT.custom_mode is an RxTask: {heartbeat.custom_mode}")
        report(heartbeat.system_status in constants.MIL_STATES, f"RPI HEARTBEAT.system_status: {constants.MIL_STATES.get(heartbeat.system_status, heartbeat.system_status)}")

    gps = latest.get((AUTOPILOT, "GPS_RAW_INT"))
    if gps:
        report(gps.fix_type >= mavlink.GPS_FIX_TYPE_3D_FIX, f"GPS_RAW_INT.fix_type: {mavlink.enums['GPS_FIX_TYPE'][gps.fix_type].name}")
        report(gps.alt_ellipsoid != 0, f"GPS_RAW_INT.alt_ellipsoid (HAE): {gps.alt_ellipsoid / 1E3:.1f} m")

    position = latest.get((AUTOPILOT, "GLOBAL_POSITION_INT"))
    if position:
        report(not (position.lat == 0 and position.lon == 0), f"GLOBAL_POSITION_INT position: {position.lat / 1E7:.7f}, {position.lon / 1E7:.7f}, {position.alt / 1E3:.1f} m AMSL")
        report(position.hdg != 65535, f"GLOBAL_POSITION_INT.hdg: {'unknown' if position.hdg == 65535 else f'{position.hdg / 100:.1f} deg'}")

    extended = latest.get((AUTOPILOT, "EXTENDED_SYS_STATE"))
    if extended:
        report(extended.landed_state != mavlink.MAV_LANDED_STATE_UNDEFINED, f"EXTENDED_SYS_STATE.landed_state: {mavlink.enums['MAV_LANDED_STATE'][extended.landed_state].name}")

    print("\n== landed_state changes (source of FlightPhase) ==")
    for seconds, state in landed_states:
        print(f"  {seconds:6.1f} s  {mavlink.enums['MAV_LANDED_STATE'][state].name}")


def check_fence(connection: MAVConnection, fence_status):
    print("\n== Fence parameters on the flight controller ==")
    for name, expected in EXPECTED_PARAMS.items():
        value = request(connection, lambda: connection.mav.param_request_read_send(*AUTOPILOT, name.encode(), -1),
                        "PARAM_VALUE", lambda message: message.param_id == name)
        if value is None:
            report(False, f"{name}: no reply")
        else:
            report(math.isclose(value.param_value, expected, abs_tol=1E-4), f"{name} = {value.param_value:g} (expect {expected:g})")

    print("\n== Fence status reported by the flight controller ==")
    if fence_status is None:
        report(False, "no FENCE_STATUS received (the autopilot only sends it while the fence is enabled)")
    elif fence_status.breach_status == 0:
        report(True, f"FENCE_STATUS: inside every fence (breach_count {fence_status.breach_count})")
    else:
        hint = ": above the ceiling, arming fails with 'Vehicle breaching Max Alt'" if fence_status.breach_type == mavlink.FENCE_BREACH_MAXALT else ""
        report(False, f"FENCE_STATUS: breached, {mavlink.enums['FENCE_BREACH'][fence_status.breach_type].name}{hint}")

    print("\n== Fence polygon on the flight controller ==")
    items = read_fence(connection)
    if items is None:
        report(False, "fence download did not complete")
        return
    if not items:
        report(False, "no fence stored")
        return

    report(all(item.command == mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION and item.param1 == len(items) for item in items),
           f"{len(items)} inclusion vertices, each carrying the vertex count")

    vertices = [(item.x / 1E7, item.y / 1E7) for item in items]
    for index, (lat, lon) in enumerate(vertices):
        print(f"    {index}: {lat:.7f}, {lon:.7f}")

    lat0, lon0 = vertices[0]
    points = [to_local_m(lat, lon, lat0, lon0) for lat, lon in vertices]
    edges = list(zip(points, points[1:] + points[:1]))
    print("    edges: " + ", ".join(f"{math.dist(a, b):.2f} m" for a, b in edges))


def main():
    parser = argparse.ArgumentParser(description="Check MAVLink telemetry and the stored geofence from the ground station side. Default connection is the RFD900x radio. Use -s/--sim for SITL. Do not run alongside ground_control.py.")
    parser.add_argument("-s", "--sim", action="store_true", help="Use simulation connection string from config.toml")
    parser.add_argument("-d", "--duration", type=float, default=20.0, help="Seconds to listen for telemetry (default 20)")
    args = parser.parse_args()

    if args.sim:
        EXPECTED_PARAMS["FENCE_ALT_MAX"] = constants.FENCE_ALT_MAX_SIM_M

    config = load_config()
    address = config["simulation"].get("ground_control") if args.sim else config["hardware"].get("ground_control")
    baud = config["hardware"].get("ground_control_baud")

    print(f"Connecting with address: {address}")
    connection = MAVConnection(address, mav_type=mavlink.MAV_TYPE_GCS, source_system=254, baud=baud)

    try:
        configure_messages(connection)
        counts, latest, landed_states = listen(connection, args.duration)
        check_telemetry(counts, latest, landed_states, args.duration)
        check_fence(connection, latest.get((AUTOPILOT, "FENCE_STATUS")))
    finally:
        connection.close()

    print(f"\n{'All checks passed.' if failures == 0 else f'{failures} check(s) failed.'}")
    return 1 if failures else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting.")
