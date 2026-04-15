#!/usr/bin/env bash
# Installs Nisse onto a Jetson alongside any existing platform software.
# Idempotent — safe to re-run after updates.
#
# What it does:
#   1. Creates the nisse user and /opt/nisse layout.
#   2. Creates a Python venv and installs the six packages in editable mode.
#   3. Drops /etc/profile.d/nisse.sh for interactive shells.
#   4. Installs the two systemd units and enables them.
#   5. Creates /var/log/nisse and the surveys directory tree.
#
# What it does NOT do:
#   - Flash the Jetson or install CUDA / JetPack. Those are prerequisites.
#   - Configure WiFi hotspot fallback (handled separately).
#   - Install the MTT vendor CAN coder (gitignored, operator-provided).

set -euo pipefail

NISSE_ROOT="${NISSE_ROOT:-/opt/nisse}"
ROVER_NAME="${ROVER_NAME:-rover}"
REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "$0")/../../.." && pwd)}"

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "install.sh must run as root (needs to create /opt/nisse and systemd units)." >&2
        exit 1
    fi
}

create_user() {
    if ! id -u nisse >/dev/null 2>&1; then
        useradd --system --home-dir "$NISSE_ROOT" --shell /bin/bash nisse
    fi
}

create_layout() {
    mkdir -p "$NISSE_ROOT"/{bin,etc,surveys/waypoint,surveys/multipass,surveys/cmp,surveys/calibration}
    mkdir -p /var/log/nisse
    chown -R nisse:nisse "$NISSE_ROOT" /var/log/nisse
}

install_venv() {
    if [ ! -d "$NISSE_ROOT/venv" ]; then
        sudo -u nisse python3 -m venv "$NISSE_ROOT/venv"
    fi
    sudo -u nisse "$NISSE_ROOT/venv/bin/pip" install --upgrade pip
    for pkg in rover_sim rover_drive rover_hardware rover_onboard rover_field_boot rover_sim_emulator; do
        sudo -u nisse "$NISSE_ROOT/venv/bin/pip" install -e "$REPO_ROOT/Software/$pkg"
    done
}

install_profile_d() {
    install -m 0644 "$(dirname "$0")/nisse.sh" /etc/profile.d/nisse.sh
}

install_systemd_units() {
    export ROVER_NAME
    envsubst < "$(dirname "$0")/rover-onboard.service"    > /etc/systemd/system/rover-onboard.service
    install -m 0644 "$(dirname "$0")/rover-field-boot.service" /etc/systemd/system/rover-field-boot.service
    systemctl daemon-reload
    systemctl enable rover-onboard.service rover-field-boot.service
}

welcome_text() {
    cat > "$NISSE_ROOT/etc/welcome.txt" <<EOF
Nisse — ${ROVER_NAME}
  rover-field-boot --list-only     list discovered surveys
  rover-onboard --help             platform services CLI
  systemctl status rover-onboard   service status
EOF
    chown nisse:nisse "$NISSE_ROOT/etc/welcome.txt"
}

main() {
    require_root
    create_user
    create_layout
    install_venv
    install_profile_d
    install_systemd_units
    welcome_text
    echo "Nisse installed at $NISSE_ROOT. Rover name: $ROVER_NAME."
    echo "Reboot or run: systemctl start rover-onboard rover-field-boot"
}

main "$@"
