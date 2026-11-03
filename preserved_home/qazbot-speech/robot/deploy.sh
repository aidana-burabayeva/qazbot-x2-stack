#!/bin/bash
# Laptop -> robot. Usage: robot/deploy.sh <ssh-host> [--keys]
#   copies src + robot scripts to ~/qazbot-speech, installs the systemd --user unit, restarts it.
#   --keys also copies .env to ~/.config/qazbot_speech/env (mode 600). Never in git, never printed.
# Does NOT stop or disable any other service. The vendor assistant and qbot are the operator's call.
set -e
HOST="${1:?ssh host}"; shift || true
cd "$(dirname "$0")/.."
ssh "$HOST" 'mkdir -p ~/qazbot-speech ~/.config/qazbot_speech ~/.config/systemd/user'
tar cf - src robot | ssh "$HOST" 'cd ~/qazbot-speech && tar xf - && sed -i "s/\r\$//" robot/*.sh robot/*.service && chmod +x robot/*.sh'
ssh "$HOST" 'bash ~/qazbot-speech/robot/make_overlay.sh'
ssh "$HOST" 'cd ~/qazbot-speech && [ -d lib ] || pip3 install -q --target lib "websockets<14" 2>&1 | tail -1   # aiohttp is already on the robot'
if [ "$1" = "--keys" ]; then
  cat .env | ssh "$HOST" 'umask 077; cat > ~/.config/qazbot_speech/env'
fi
ssh "$HOST" 'cp ~/qazbot-speech/robot/qazbot-speech.service ~/qazbot-speech/robot/lingxi-off.service ~/.config/systemd/user/ && systemctl --user daemon-reload && systemctl --user enable qazbot-speech.service lingxi-off.service'
echo "installed. start with: ssh $HOST systemctl --user restart qazbot-speech"
# Linger: the user manager, and with it the service, starts at boot with nobody logged in.
ssh "$HOST" 'loginctl show-user "$USER" -p Linger | grep -q "=yes" || loginctl enable-linger "$USER" ||
  echo "!! could not enable linger - the service will not start at boot until agi logs in" >&2'
# Boot check: loud if anything the robot needs at boot is missing.
ssh "$HOST" 'ok=1; loginctl show-user "$USER" -p Linger | grep -q "=yes" || { echo "!! linger is OFF" >&2; ok=0; }
  systemctl --user is-enabled qazbot-speech.service >/dev/null 2>&1 || { echo "!! qazbot-speech is not enabled" >&2; ok=0; }
  systemctl --user cat lingxi-off.service >/dev/null 2>&1 || echo "note: lingxi-off.service is not installed on this robot (the vendor assistant is not silenced)" >&2
  [ "$ok" = 1 ] && echo "boot check: linger on, qazbot-speech enabled (starts at every boot)"'
