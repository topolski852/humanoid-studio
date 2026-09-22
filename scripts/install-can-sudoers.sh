#!/usr/bin/env bash
# Install the passwordless-sudo rule Humanoid Studio needs to assign CAN
# adapters to limbs from the app (Devices → CAN adapters).
#
# Usage:  scripts/install-can-sudoers.sh [username]
#
# Defaults to the user running the script (not root, when run under sudo).
set -euo pipefail

TEMPLATE="$(dirname "$(readlink -f "$0")")/humanoid-can.sudoers"
TARGET=/etc/sudoers.d/humanoid-can
USERNAME="${1:-${SUDO_USER:-$USER}}"

[ -f "$TEMPLATE" ] || { echo "Template not found: $TEMPLATE" >&2; exit 1; }
id "$USERNAME" >/dev/null 2>&1 || { echo "No such user: $USERNAME" >&2; exit 1; }

STAGED="$(mktemp)"
trap 'rm -f "$STAGED"' EXIT
sed "s/__HUMANOID_USER__/$USERNAME/" "$TEMPLATE" > "$STAGED"

# Validate before installing — a malformed sudoers file can lock out sudo.
if ! sudo visudo -cf "$STAGED"; then
    echo "Generated sudoers file failed validation; not installing." >&2
    exit 1
fi

sudo install -m 440 -o root -g root "$STAGED" "$TARGET"
echo "Installed $TARGET for user '$USERNAME'."

# Confirm the rule actually grants what the app needs, with the credential
# cache cleared so a recent password entry can't fake a pass.
sudo -k
if sudo -n ip link set lo txqueuelen 1000 2>/dev/null; then
    echo "Verified: passwordless 'ip link set' is working."
else
    echo "WARNING: 'sudo -n ip link set' still requires a password." >&2
    exit 1
fi
