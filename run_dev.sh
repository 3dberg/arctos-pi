#!/usr/bin/env bash
# Dev runner for laptop. Uses mock CAN bus unless config.yaml overrides.
set -euo pipefail
cd "$(dirname "$0")"
exec python3 -m uvicorn backend.api:app --host 0.0.0.0 --port 8000 --reload
