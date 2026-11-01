#!/bin/bash
# OLD sandbox (no resolver ro-bind): same flags as HEAD~1 engineering_sandbox
S=$1; W=$2
exec bwrap --die-with-parent --new-session --unshare-pid --unshare-ipc --cap-drop ALL \
 --ro-bind / / --proc /proc --dev /dev --tmpfs /run --tmpfs /tmp \
 --bind $S/old_sb $S/old_sb --bind $W $W --bind $HOME/.codex $HOME/.codex --chdir $W -- \
 codex exec --json --skip-git-repo-check -s workspace-write -C $W -m gpt-6.1-sol -- 'Reply with exactly OLD_SB_OK. Do not run tools.'
