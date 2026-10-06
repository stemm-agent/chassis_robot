import ipaddress
import json
import os
import socket
import subprocess
import time
from datetime import datetime, timezone
from typing import Any
from urllib import error, request

DEVICE_ID = os.getenv("DEVICE_REPORT_DEVICE_ID", os.getenv("MQTT_DEVICE_ID", "robot-001"))
REPORT_URL = os.getenv("DEVICE_REPORT_URL", "").strip()
CHECK_INTERVAL = float(os.getenv("DEVICE_REPORT_CHECK_INTERVAL", "5"))
HEARTBEAT_INTERVAL = float(os.getenv("DEVICE_REPORT_HEARTBEAT_INTERVAL", "60"))
HTTP_TIMEOUT = float(os.getenv("DEVICE_REPORT_HTTP_TIMEOUT", "5"))
INCLUDE_WAITING_STATE = os.getenv("DEVICE_REPORT_INCLUDE_WAITING_STATE", "true").lower() in {
    "1",
    "true",
    "yes",
    "on",
}
HOSTNAME = socket.gethostname()


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def run_command(command: list[str]) -> str:
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="ignore",
        )
    except FileNotFoundError:
        return ""

    if completed.returncode != 0:
        return ""
    return completed.stdout.strip()


def list_wireless_interfaces() -> list[str]:
    output = run_command(["iw", "dev"])
    interfaces: list[str] = []
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("Interface "):
            interfaces.append(stripped.split()[1])

    if interfaces:
        return interfaces

    try:
        candidates = os.listdir("/sys/class/net")
    except OSError:
        return interfaces

    for name in candidates:
        wireless_path = os.path.join("/sys/class/net", name, "wireless")
        if os.path.isdir(wireless_path):
            interfaces.append(name)
    return interfaces


def get_default_route_interface() -> str:
    output = run_command(["ip", "route", "show", "default"])
    for line in output.splitlines():
        parts = line.split()
        if "dev" in parts:
            index = parts.index("dev")
            if index + 1 < len(parts):
                return parts[index + 1]
    return ""


def get_interface_mode(interface: str) -> str:
    output = run_command(["iw", "dev", interface, "info"])
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("type "):
            return stripped.split()[1].lower()

    nmcli_output = run_command(["nmcli", "-t", "-f", "GENERAL.TYPE,GENERAL.STATE", "device", "show", interface])
    if "wifi" in nmcli_output.lower():
        return "managed"
    return "unknown"


def get_interface_ipv4(interface: str) -> str:
    output = run_command(["ip", "-4", "addr", "show", "dev", interface, "scope", "global"])
    for line in output.splitlines():
        stripped = line.strip()
        if stripped.startswith("inet "):
            return stripped.split()[1].split("/")[0]
    return ""


def get_interface_ssid(interface: str) -> str:
    output = run_command(["iwgetid", interface, "--raw"])
    if output:
        return output

    nmcli_output = run_command(["nmcli", "-t", "-f", "ACTIVE,SSID,DEVICE", "dev", "wifi"])
    for line in nmcli_output.splitlines():
        parts = line.split(":", 2)
        if len(parts) == 3 and parts[0] == "yes" and parts[2] == interface:
            return parts[1]
    return ""


def is_usable_lan_ipv4(ip_text: str) -> bool:
    if not ip_text:
        return False

    try:
        ip_value = ipaddress.ip_address(ip_text)
    except ValueError:
        return False

    return isinstance(ip_value, ipaddress.IPv4Address) and ip_value.is_private and not ip_value.is_link_local


def detect_network_state() -> dict[str, Any]:
    wireless_interfaces = list_wireless_interfaces()
    default_route_interface = get_default_route_interface()

    interface_infos: list[dict[str, str]] = []
    for interface in wireless_interfaces:
        mode = get_interface_mode(interface)
        ipv4 = get_interface_ipv4(interface)
        ssid = get_interface_ssid(interface)
        interface_infos.append(
            {
                "name": interface,
                "mode": mode,
                "ipv4": ipv4,
                "ssid": ssid,
            }
        )

    if default_route_interface:
        interface_infos.sort(key=lambda item: item["name"] != default_route_interface)

    for info in interface_infos:
        if info["mode"] in {"managed", "station"} and is_usable_lan_ipv4(info["ipv4"]):
            return {
                "status": "ready",
                "ready": True,
                "networkMode": "wifi-client",
                "interface": info["name"],
                "ipv4": info["ipv4"],
                "ssid": info["ssid"],
                "message": "Wi-Fi client mode is ready for direct connection.",
                "defaultRouteInterface": default_route_interface,
                "wirelessInterfaces": interface_infos,
            }

    for info in interface_infos:
        if info["mode"] == "ap":
            return {
                "status": "waiting_ap",
                "ready": False,
                "networkMode": "ap",
                "interface": info["name"],
                "ipv4": info["ipv4"],
                "ssid": info["ssid"],
                "message": "Device is still in AP mode, waiting for Wi-Fi client connection.",
                "defaultRouteInterface": default_route_interface,
                "wirelessInterfaces": interface_infos,
            }

    for info in interface_infos:
        if info["mode"] in {"managed", "station"}:
            return {
                "status": "waiting_ip",
                "ready": False,
                "networkMode": "wifi-client",
                "interface": info["name"],
                "ipv4": info["ipv4"],
                "ssid": info["ssid"],
                "message": "Wi-Fi client mode detected, waiting for a usable LAN IPv4 address.",
                "defaultRouteInterface": default_route_interface,
                "wirelessInterfaces": interface_infos,
            }

    return {
        "status": "waiting_wifi",
        "ready": False,
        "networkMode": "unknown",
        "interface": "",
        "ipv4": "",
        "ssid": "",
        "message": "No wireless client connection detected yet.",
        "defaultRouteInterface": default_route_interface,
        "wirelessInterfaces": interface_infos,
    }


def build_payload(network_state: dict[str, Any], report_type: str) -> dict[str, Any]:
    return {
        "deviceId": DEVICE_ID,
        "hostname": HOSTNAME,
        "timestamp": now_iso(),
        "reportType": report_type,
        "status": network_state["status"],
        "ready": network_state["ready"],
        "networkMode": network_state["networkMode"],
        "directConnectIp": network_state["ipv4"] if network_state["ready"] else "",
        "interface": network_state["interface"],
        "ssid": network_state["ssid"],
        "defaultRouteInterface": network_state["defaultRouteInterface"],
        "message": network_state["message"],
        "wirelessInterfaces": network_state["wirelessInterfaces"],
    }


def should_send_payload(network_state: dict[str, Any]) -> bool:
    if network_state["ready"]:
        return True
    return INCLUDE_WAITING_STATE


def post_payload(payload: dict[str, Any]) -> tuple[bool, str]:
    if not REPORT_URL:
        return False, "DEVICE_REPORT_URL is empty, skipping report."

    body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
    req = request.Request(
        REPORT_URL,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with request.urlopen(req, timeout=HTTP_TIMEOUT) as response:
            response_text = response.read().decode("utf-8", errors="ignore").strip()
            status_text = f"HTTP {response.status}"
            if response_text:
                return True, f"{status_text} {response_text}"
            return True, status_text
    except error.HTTPError as exc:
        response_text = exc.read().decode("utf-8", errors="ignore").strip()
        return False, f"HTTP {exc.code} {response_text}".strip()
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)


def make_state_signature(network_state: dict[str, Any]) -> str:
    signature_source = {
        "status": network_state["status"],
        "ready": network_state["ready"],
        "networkMode": network_state["networkMode"],
        "interface": network_state["interface"],
        "ipv4": network_state["ipv4"],
        "ssid": network_state["ssid"],
        "defaultRouteInterface": network_state["defaultRouteInterface"],
    }
    return json.dumps(signature_source, sort_keys=True, ensure_ascii=True)


def log(message: str) -> None:
    print(f"[device-reporter] {message}", flush=True)


def main() -> None:
    last_signature = ""
    last_heartbeat_at = 0.0

    log(
        "starting with "
        f"deviceId={DEVICE_ID} reportUrl={REPORT_URL or '<empty>'} "
        f"includeWaiting={INCLUDE_WAITING_STATE}"
    )

    while True:
        network_state = detect_network_state()
        signature = make_state_signature(network_state)
        current_time = time.time()

        report_type = ""
        if signature != last_signature:
            report_type = "state-change" if last_signature else "startup"
        elif current_time - last_heartbeat_at >= HEARTBEAT_INTERVAL:
            report_type = "heartbeat"

        if report_type:
            if should_send_payload(network_state):
                payload = build_payload(network_state, report_type)
                success, result = post_payload(payload)
                if success:
                    last_signature = signature
                    last_heartbeat_at = current_time
                    log(
                        f"reported {report_type}: status={network_state['status']} "
                        f"mode={network_state['networkMode']} ip={payload['directConnectIp'] or '-'} {result}"
                    )
                else:
                    log(
                        f"report failed: type={report_type} status={network_state['status']} "
                        f"mode={network_state['networkMode']} ip={network_state['ipv4'] or '-'} error={result}"
                    )
            else:
                last_signature = signature
                last_heartbeat_at = current_time
                log(
                    f"state observed but not reported: status={network_state['status']} "
                    f"mode={network_state['networkMode']} ip={network_state['ipv4'] or '-'}"
                )

        time.sleep(CHECK_INTERVAL)


if __name__ == "__main__":
    main()
