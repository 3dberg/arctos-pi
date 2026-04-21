#!/usr/bin/env bash
# Dev runner for laptop. Uses mock CAN bus unless config.yaml overrides.
# Always runs via .venv so conda/system pythons can't shadow installed deps.
set -euo pipefail
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
    echo "error: .venv not found — run ./install.sh first" >&2
    exit 1
fi
exec .venv/bin/python -m uvicorn backend.api:app --host 0.0.0.0 --port 8000 --reload
