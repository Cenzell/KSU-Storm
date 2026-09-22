#!/usr/bin/env bash
# Sync the repo from the driver station to the robot Pi over LAN via rsync+ssh.
#
# Target resolution, in order:
#   1. explicit argument:        ./scripts/deploy.sh storm@10.42.0.85
#   2. $KSU_ROBOT_HOST env var:  KSU_ROBOT_HOST=ksu-storm.local ./scripts/deploy.sh
#   3. mDNS auto-discovery: finds whichever Pi is *currently running*
#      robot.py (see scripts/find_robot.py) — only works once robot.py has
#      been started at least once on that Pi.
#   4. static fallback: $KSU_ROBOT_USER@raspberrypi.local
#
# $KSU_ROBOT_USER (default: storm) sets the SSH user for cases 2-4.
#
# One-time setup on the Pi first: enable SSH (raspi-config), then
#   ssh-copy-id <target>
# so this doesn't prompt for a password on every deploy.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_USER="${KSU_ROBOT_USER:-storm}"

if [ "${1:-}" != "" ]; then
    TARGET="$1"
elif [ "${KSU_ROBOT_HOST:-}" != "" ]; then
    TARGET="${ROBOT_USER}@${KSU_ROBOT_HOST}"
else
    echo "No target given — looking for a robot via mDNS..." >&2
    DISCOVERED="$(python3 "$REPO_ROOT/scripts/find_robot.py" --timeout 3 2>/dev/null || true)"
    if [ -n "$DISCOVERED" ]; then
        TARGET="${ROBOT_USER}@${DISCOVERED}"
        echo "Found robot at $DISCOVERED" >&2
    else
        TARGET="${ROBOT_USER}@raspberrypi.local"
        echo "No robot found via mDNS (is robot.py running on it? try: python3 scripts/find_robot.py) — falling back to $TARGET" >&2
    fi
fi

REMOTE_DIR="${KSU_ROBOT_DIR:-~/KSU-Storm}"

echo "Deploying $REPO_ROOT -> $TARGET:$REMOTE_DIR"

rsync -avz --delete \
  --exclude '.git/' \
  --exclude '__pycache__/' \
  --exclude '*.py[cod]' \
  --exclude 'venv/' \
  --exclude '.venv/' \
  --exclude '*.pdf' \
  "$REPO_ROOT/" "$TARGET:$REMOTE_DIR/"

echo
echo "Done. Restart the robot process to pick up the changes, e.g.:"
echo "  ssh $TARGET 'sudo systemctl restart ksu-storm-robot'   # if using a systemd unit"
echo "  ssh $TARGET                                             # otherwise, restart it manually"
