#!/usr/bin/env bash
set -euo pipefail

readonly REPOSITORY="https://github.com/koide3/small_gicp.git"
readonly COMMIT="8a2d3734f699c042db74ae61d295b0b928163526"

if [[ $# -ne 1 || "$1" != /* ]]; then
  echo "usage: $0 /absolute/install/prefix" >&2
  exit 2
fi

readonly INSTALL_PREFIX="$1"
readonly RECEIPT_DIR="$INSTALL_PREFIX/share/go2-dependency-locks"
readonly RECEIPT="$RECEIPT_DIR/small_gicp.commit"
if [[ -f "$INSTALL_PREFIX/lib/libsmall_gicp.so" \
   && -f "$INSTALL_PREFIX/lib/cmake/small_gicp/small_gicp-config.cmake" \
   && -f "$RECEIPT" \
   && "$(<"$RECEIPT")" = "$COMMIT" ]]; then
  echo "reusing locked small_gicp $COMMIT at $INSTALL_PREFIX"
  exit 0
fi
readonly WORK_DIR="$(mktemp -d /tmp/go2-small-gicp-install.XXXXXX)"
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
  -DBUILD_HELPER=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WITH_OPENMP=ON \
  -DBUILD_WITH_TBB=OFF \
  -DBUILD_WITH_PCL=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$WORK_DIR/build" --parallel
cmake --install "$WORK_DIR/build"

test -f "$INSTALL_PREFIX/lib/libsmall_gicp.so"
test -f "$INSTALL_PREFIX/lib/cmake/small_gicp/small_gicp-config.cmake"
install -d -m 0755 "$RECEIPT_DIR"
printf '%s\n' "$COMMIT" >"$RECEIPT.tmp"
mv -f "$RECEIPT.tmp" "$RECEIPT"
echo "small_gicp $COMMIT installed at $INSTALL_PREFIX"
echo "Add $INSTALL_PREFIX to CMAKE_PREFIX_PATH before colcon build."
