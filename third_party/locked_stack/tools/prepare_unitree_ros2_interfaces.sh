#!/usr/bin/env bash
set -euo pipefail

readonly REPOSITORY="https://github.com/unitreerobotics/unitree_ros2.git"
readonly COMMIT="668d1ec5a05d1c38d3306bdca7d59f2ba3581a88"

if [[ $# -ne 1 || "$1" != /* ]]; then
  echo "usage: $0 /absolute/unitree_ros2/source" >&2
  exit 2
fi

readonly SOURCE_ROOT="$1"
if [[ -e "$SOURCE_ROOT" && ! -d "$SOURCE_ROOT/.git" ]]; then
  echo "refusing non-git destination: $SOURCE_ROOT" >&2
  exit 3
fi
if [[ ! -e "$SOURCE_ROOT" ]]; then
  git clone --filter=blob:none "$REPOSITORY" "$SOURCE_ROOT"
  git -C "$SOURCE_ROOT" checkout --detach "$COMMIT"
fi

readonly ACTUAL_REMOTE="$(git -C "$SOURCE_ROOT" remote get-url origin)"
readonly ACTUAL_COMMIT="$(git -C "$SOURCE_ROOT" rev-parse HEAD)"
if [[ "$ACTUAL_REMOTE" != "$REPOSITORY" ]]; then
  echo "unitree_ros2 remote mismatch: $ACTUAL_REMOTE" >&2
  exit 4
fi
if [[ "$ACTUAL_COMMIT" != "$COMMIT" ]]; then
  echo "unitree_ros2 must be detached at $COMMIT; found $ACTUAL_COMMIT" >&2
  echo "use a fresh destination so this tool never overwrites local work" >&2
  exit 5
fi

readonly UNITREE_GO="$SOURCE_ROOT/cyclonedds_ws/src/unitree/unitree_go"
test -f "$UNITREE_GO/package.xml"
test -f "$UNITREE_GO/msg/SportModeState.msg"
test -f "$UNITREE_GO/msg/LowState.msg"
echo "$UNITREE_GO"
