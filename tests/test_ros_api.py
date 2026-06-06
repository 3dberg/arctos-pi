"""The ROS2 endpoints must degrade gracefully when ROS isn't enabled, and the
existing CAN control API must keep working regardless (additive contract)."""
from fastapi.testclient import TestClient

from backend.api import app


def test_existing_api_still_works_without_ros():
    with TestClient(app) as client:
        r = client.get("/api/state")
        assert r.status_code == 200
        assert "axes" in r.json()


def test_ros_status_reports_unavailable():
    with TestClient(app) as client:
        r = client.get("/api/ros/status")
        assert r.status_code == 200
        body = r.json()
        assert body["available"] is False
        assert body["enabled"] is False


def test_ros_action_endpoints_return_503_when_disabled():
    with TestClient(app) as client:
        assert client.post("/api/ros/estop").status_code == 503
        assert client.post("/api/ros/enable", json={"on": True}).status_code == 503
        assert client.post(
            "/api/ros/move", json={"joints_deg": {"J1": 10.0}, "duration_s": 2.0}
        ).status_code == 503
