#!/bin/bash
# Remote mode: the SSH key of the Windows package (docs/远程使用指南.md).
#   install_remote_key.sh [AUTHORIZED_KEYS]      default ~/.ssh/authorized_keys
# Makes the ed25519 key pair ~/.config/carla_cosim_studio/remote_key (+ .pub;
# REMOTE_KEY=... for another path) unless it exists, and appends one line for
# it to AUTHORIZED_KEYS:
#   restrict,port-forwarding,permitopen="127.0.0.1:57120",permitopen="127.0.0.1:57121",permitlisten="127.0.0.1:1",command="<repo>/scripts/remote_session.sh" ssh-ed25519 AAAA... carla-cosim-remote
# The key can then only run remote_session.sh and forward those two ports: no
# shell, no other command, no file transfer, no other forwarding. (Without
# permitlisten, port-forwarding would also allow ssh -R to any port; sshd 8.9
# takes no "none" there, and no user can listen on port 1.)
# Running it again changes nothing (the key is already in the file); the other
# lines of the file are never changed. The private key goes into the Windows
# package (build_remote_package.sh) and never into git.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
KEY="${REMOTE_KEY:-$HOME/.config/carla_cosim_studio/remote_key}"
AUTH="${1:-$HOME/.ssh/authorized_keys}"
COMMAND="$COSIM_ROOT/scripts/remote_session.sh"
fail() { echo "$*" >&2; exit 1; }

if [ ! -f "$KEY" ]; then
  mkdir -p "$(dirname "$KEY")" && chmod 700 "$(dirname "$KEY")" || fail "cannot create $(dirname "$KEY")"
  ssh-keygen -q -t ed25519 -N "" -C carla-cosim-remote -f "$KEY" < /dev/null || fail "ssh-keygen failed"
  echo "new key: $KEY"
elif [ ! -f "$KEY.pub" ]; then
  ssh-keygen -y -f "$KEY" > "$KEY.pub" < /dev/null || fail "cannot read the key $KEY"
fi
chmod 600 "$KEY" "$KEY.pub"
read -r TYPE BLOB _ < "$KEY.pub"
[ "$TYPE" = ssh-ed25519 ] && [ -n "$BLOB" ] || fail "$KEY.pub is not an ed25519 public key"
LINE="restrict,port-forwarding,permitopen=\"127.0.0.1:57120\",permitopen=\"127.0.0.1:57121\",permitlisten=\"127.0.0.1:1\",command=\"$COMMAND\" $TYPE $BLOB carla-cosim-remote"
[ -x "$COMMAND" ] || echo "warning: $COMMAND is missing or not executable" >&2

if [ -f "$AUTH" ] && grep -qF -- " $BLOB" "$AUTH"; then
  if grep -qxF -- "$LINE" "$AUTH"; then
    echo "already in $AUTH (unchanged)"
  else
    echo "warning: the key is in $AUTH already, with other options; the file is left unchanged:" >&2
    grep -F -- " $BLOB" "$AUTH" >&2
  fi
  exit 0
fi
if [ ! -e "$AUTH" ]; then
  mkdir -p "$(dirname "$AUTH")" || fail "cannot create $(dirname "$AUTH")"
  [ "$(dirname "$AUTH")" = "$HOME/.ssh" ] && chmod 700 "$HOME/.ssh"
  (umask 077 && : > "$AUTH") || fail "cannot create $AUTH"
fi
# A last line without its newline would otherwise be joined to ours.
if [ -s "$AUTH" ] && [ -n "$(tail -c 1 "$AUTH")" ]; then
  printf '\n' >> "$AUTH" || fail "cannot write $AUTH"
fi
printf '%s\n' "$LINE" >> "$AUTH" || fail "cannot write $AUTH"
# sshd ignores an authorized_keys that others can write to.
[ -n "$(find "$AUTH" -maxdepth 0 -perm /022)" ] && chmod go-w "$AUTH"
echo "added to $AUTH:"
echo "$LINE"
