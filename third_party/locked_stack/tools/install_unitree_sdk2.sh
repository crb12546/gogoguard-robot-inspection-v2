#!/usr/bin/env bash
set -euo pipefail

readonly REPOSITORY="https://github.com/unitreerobotics/unitree_sdk2.git"
readonly COMMIT="5ea10f3230e41c6ab62aa79f49f84afa685d84aa"

if [[ $# -ne 1 || "$1" != /* ]]; then
  echo "usage: $0 /absolute/install/prefix" >&2
  exit 2
fi

readonly INSTALL_PREFIX="$1"
readonly RECEIPT_DIR="$INSTALL_PREFIX/share/go2-dependency-locks"
readonly RECEIPT="$RECEIPT_DIR/unitree_sdk2.commit"
if [[ -f "$INSTALL_PREFIX/lib/libunitree_sdk2.a" \
   && -f "$INSTALL_PREFIX/lib/libddscxx.so" \
   && -f "$INSTALL_PREFIX/lib/libddsc.so" \
   && -f "$INSTALL_PREFIX/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" \
   && -f "$RECEIPT" \
   && "$(<"$RECEIPT")" = "$COMMIT" ]]; then
  echo "reusing locked unitree_sdk2 $COMMIT at $INSTALL_PREFIX"
  exit 0
fi
readonly WORK_DIR="$(mktemp -d /tmp/go2-unitree-sdk2-install.XXXXXX)"
cleanup() {
  rm -rf -- "$WORK_DIR"
}
trap cleanup EXIT

git clone --filter=blob:none "$REPOSITORY" "$WORK_DIR/source"
git -C "$WORK_DIR/source" checkout --detach "$COMMIT"
test "$(git -C "$WORK_DIR/source" rev-parse HEAD)" = "$COMMIT"

cmake -S "$WORK_DIR/source" -B "$WORK_DIR/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
  -DBUILD_EXAMPLES=OFF
cmake --build "$WORK_DIR/build" --parallel
cmake --install "$WORK_DIR/build"

test -f "$INSTALL_PREFIX/lib/libunitree_sdk2.a"
test -f "$INSTALL_PREFIX/lib/libddscxx.so"
test -f "$INSTALL_PREFIX/lib/libddsc.so"
test -f "$INSTALL_PREFIX/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake"
install -d -m 0755 "$RECEIPT_DIR"
printf '%s\n' "$COMMIT" >"$RECEIPT.tmp"
mv -f "$RECEIPT.tmp" "$RECEIPT"
echo "unitree_sdk2 $COMMIT installed at $INSTALL_PREFIX"
