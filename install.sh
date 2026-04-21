#!/usr/bin/env bash
# Installer for arctos-pi. Works on Raspberry Pi OS (bookworm, aarch64)
# and Ubuntu 22.04+/24.04 (x86_64).
#
# Usage:
#   ./install.sh              # install deps, set up venv, register systemd user unit
#   ./install.sh --kiosk      # also install chromium kiosk autostart (Pi only)
#   ./install.sh --no-service # skip systemd registration (dev mode)
set -euo pipefail

cd "$(dirname "$0")"
REPO_DIR="$(pwd)"
ARCH="$(uname -m)"
KIOSK=0
NO_SERVICE=0
for arg in "$@"; do
    case "$arg" in
        --kiosk) KIOSK=1 ;;
        --no-service) NO_SERVICE=1 ;;
        *) echo "unknown flag: $arg" && exit 2 ;;
    esac
done

echo "==> arctos-pi installer (arch: $ARCH)"

# ---- System packages ----
if command -v apt-get >/dev/null 2>&1; then
    echo "==> apt: python3-venv, can-utils (optional)"
    sudo apt-get update -qq
    sudo apt-get install -y python3-venv python3-pip can-utils
else
    echo "!! non-apt system; install python3-venv + (optional) can-utils manually"
fi

# ---- Python venv ----
if [ ! -d .venv ]; then
    echo "==> creating venv"
    python3 -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
pip install --upgrade pip -q
pip install -e '.[dev]' -q

# ---- Config ----
if [ ! -f config.yaml ]; then
    echo "==> seeding config.yaml from example"
    cp config.example.yaml config.yaml
    echo "   edit config.yaml to set gear ratios, limits, and can.backend"
fi

# ---- udev rule for MKS CANable v1.0 Pro (stable /dev/arctos-canable symlink) ----
if [ -d /etc/udev/rules.d ]; then
    sudo tee /etc/udev/rules.d/60-arctos-canable.rules >/dev/null <<'EOF'
# MKS CANable v1.0 Pro (CANable firmware, slcan). Exposes /dev/arctos-canable
# with user-rw so no chmod is needed.
SUBSYSTEM=="tty", ATTRS{idVendor}=="ad50", ATTRS{idProduct}=="60c4", \
  SYMLINK+="arctos-canable", MODE="0666", GROUP="dialout"
# Alternate VID/PID seen on Canable firmware builds:
SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", \
  SYMLINK+="arctos-canable", MODE="0666", GROUP="dialout"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "==> udev: /dev/arctos-canable symlink installed (replug adapter to activate)"
fi

# ---- systemd user service ----
if [ "$NO_SERVICE" = 0 ]; then
    mkdir -p ~/.config/systemd/user
    sed "s|@@REPO_DIR@@|$REPO_DIR|g" systemd/arctos.service > ~/.config/systemd/user/arctos.service
    systemctl --user daemon-reload
    systemctl --user enable arctos.service
    echo "==> systemd user unit installed: systemctl --user start arctos"
    # Lingering lets the service survive logout (useful on headless Pi).
    if command -v loginctl >/dev/null 2>&1; then
        sudo loginctl enable-linger "$USER" || true
    fi
fi

# ---- Chromium kiosk autostart (Pi OS only) ----
if [ "$KIOSK" = 1 ]; then
    if [ "$ARCH" != "aarch64" ] && [ "$ARCH" != "armv7l" ]; then
        echo "!! kiosk mode only set up automatically on Pi; skipping on $ARCH"
    else
        echo "==> installing chromium kiosk autostart"
        sudo apt-get install -y chromium-browser unclutter || sudo apt-get install -y chromium
        mkdir -p ~/.config/autostart
        cat > ~/.config/autostart/arctos-kiosk.desktop <<EOF
[Desktop Entry]
Type=Application
Name=Arctos Kiosk
Exec=bash -c "sleep 5 && chromium-browser --kiosk --noerrdialogs --disable-infobars --incognito http://localhost:8000"
X-GNOME-Autostart-enabled=true
EOF
    fi
fi

echo
echo "==> done."
echo "   start now:   systemctl --user start arctos"
echo "   status:      systemctl --user status arctos"
echo "   logs:        journalctl --user -u arctos -f"
echo "   URL:         http://$(hostname -I | awk '{print $1}'):8000"
