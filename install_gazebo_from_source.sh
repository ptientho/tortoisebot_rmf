#!/usr/bin/env bash
#
# This script follows the Gazebo Classic "Install from source (Ubuntu)" tutorial:
# https://classic.gazebosim.org/tutorials?tut=install_from_source
#
set -eo pipefail
IFS=$'\n\t'

# Configurable variables (kept minimal and tutorial-compatible)
GAZEBO_MAJOR_VERSION=${GAZEBO_MAJOR_VERSION:-11}     # tutorial examples use generic 'version'
INSTALL_PREFIX=${INSTALL_PREFIX:-/usr/local}        # tutorial default for source install
BUILD_ROOT=${BUILD_ROOT:-$HOME/tmp/gazebo}     # where we'll clone & build
DEPS_SCRIPT_URL="https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh"
DEPS_SCRIPT_PATH="/tmp/dependencies.sh"

echo "GAZEBO_MAJOR_VERSION=${GAZEBO_MAJOR_VERSION}"
echo "INSTALL_PREFIX=${INSTALL_PREFIX}"
echo "BUILD_ROOT=${BUILD_ROOT}"
echo

# --- Step 0: Ensure apt-get present (tutorial targets Ubuntu) ---
if ! command -v apt-get >/dev/null 2>&1; then
  echo "Error: apt-get not found. This script targets Ubuntu/Debian as in the tutorial." >&2
  exit 1
fi

# "Make sure you have removed the Ubuntu pre-compiled binaries before installing from source:"
echo "Removing any system-installed Gazebo / sdformat / ignition libraries (if present)..."
sudo apt-get remove -y '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*' || true

# "Setup your computer to accept software from packages.osrfoundation.org."
echo "Adding OSRF Gazebo apt repository and key..."
sudo sh -c "echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main' > /etc/apt/sources.list.d/gazebo-stable.list"
if command -v wget >/dev/null 2>&1; then
  wget -qO - https://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
else
  curl -fsSL https://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
fi
sudo apt-get update

# GAZEBO_MAJOR_VERSION and ROS_DISTRO set, then install the listed packages.
echo "Downloading dependencies helper script (dependencies_archive.sh) as in tutorial..."
if command -v wget >/dev/null 2>&1; then
  wget "${DEPS_SCRIPT_URL}" -O "${DEPS_SCRIPT_PATH}"
  GAZEBO_MAJOR_VERSION="${GAZEBO_MAJOR_VERSION}" ROS_DISTRO=humble . "${DEPS_SCRIPT_PATH}" 
else
  curl "${DEPS_SCRIPT_URL}" -o "${DEPS_SCRIPT_PATH}"
  GAZEBO_MAJOR_VERSION="${GAZEBO_MAJOR_VERSION}" ROS_DISTRO=humble . "${DEPS_SCRIPT_PATH}" 
fi

if [[ -n "${BASE_DEPENDENCIES-}" || -n "${GAZEBO_BASE_DEPENDENCIES-}" ]]; then
  echo "Installing packages reported by dependencies helper..."
  echo "${BASE_DEPENDENCIES-} ${GAZEBO_BASE_DEPENDENCIES-}" | tr -d '\\' | xargs sudo apt-get -y install
else
  echo "Warning: dependency variables not exported by helper script; tutorial suggests the above command—"
  echo "you may need to inspect ${DEPS_SCRIPT_PATH} output and install required packages manually."
fi

# --- Optional: DART support via PPA (tutorial block) ---
cat <<'DART_NOTE'
# If you want DART support (tutorial):
# sudo apt-add-repository ppa:dartsim
# sudo apt-get update
# sudo apt-get install libdart6-dev
#
# (Only enable if you know you need DART — tutorial warns about potential ROS conflicts)
DART_NOTE


# Install the listed dependencies (from source, tutorial style)
echo "Installing Gazebo 11 dependencies from source..."
cd "$BUILD_ROOT"
mkdir -p deps_src
cd deps_src

# SDFormat 11
git clone -b sdf11 https://github.com/osrf/sdformat.git
cd sdformat
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..
make -j"$(nproc)"
sudo make install
cd "$BUILD_ROOT/deps_src"

# Ignition Math 6
git clone -b ignition-math6 https://github.com/gazebosim/ignition-math.git
cd ignition-math
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..
make -j"$(nproc)"
sudo make install
cd "$BUILD_ROOT/deps_src"

# Ignition Transport 8
git clone -b ignition-transport8 https://github.com/gazebosim/ignition-transport.git
cd ignition-transport
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..
make -j"$(nproc)"
sudo make install
cd "$BUILD_ROOT/deps_src"

# Ignition Messages 7
git clone -b ignition-msgs7 https://github.com/gazebosim/ignition-msgs.git
cd ignition-msgs
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..
make -j"$(nproc)"
sudo make install
cd "$BUILD_ROOT/deps_src"

# Steps: git clone https://github.com/osrf/gazebo /tmp/gazebo
echo "Preparing source tree under ${BUILD_ROOT}..."
rm -rf "${BUILD_ROOT}/gazebo"
mkdir -p "${BUILD_ROOT}/gazebo"

echo "Cloning gazebo repository (as tutorial: git clone https://github.com/osrf/gazebo /tmp/gazebo)..."
git clone https://github.com/osrf/gazebo "${BUILD_ROOT}/gazebo"
cd "${BUILD_ROOT}/gazebo"

# Optional: checkout a stable branch matching major version (tutorial suggests gazebo6 branch as example)
BRANCH_NAME="gazebo${GAZEBO_MAJOR_VERSION}"
if git ls-remote --heads origin "${BRANCH_NAME}" | grep -q "${BRANCH_NAME}"; then
  echo "Checking out branch ${BRANCH_NAME} (tutorial suggests using matching branch for stability)..."
  git fetch origin "${BRANCH_NAME}:${BRANCH_NAME}" || true
  git checkout "${BRANCH_NAME}" || true
else
  echo "Branch ${BRANCH_NAME} not found; staying on default branch (tutorial allows using master for latest)."
fi

mkdir -p build
cd build

echo "Configuring CMake (tutorial suggests cmake ../ or cmake -DCMAKE_BUILD_TYPE=Debug ../)..."
# Use INSTALL_PREFIX if non-default as tutorial suggests custom install path
cmake -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" ../

echo "Running make (tutorial: make -j4 example; we use available cores)..."
make -j"$(nproc)"

echo "Running sudo make install (tutorial step)..."
sudo make install

# --- Tutorial: post-install environment setup for local installs (lines ~150-155, 166-169) ---
if [[ "${INSTALL_PREFIX}" == "/usr/local" ]]; then
  echo "If installed to /usr/local, ensure /usr/local/lib is in ld path (tutorial recommends):"
  echo "/usr/local/lib -> /etc/ld.so.conf.d/gazebo.conf and ldconfig"
  echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf >/dev/null
  sudo ldconfig
else
  echo "If installed to a local directory, tutorial suggests adding PATH/LD_LIBRARY_PATH/PKG_CONFIG_PATH to ~/.bashrc:"
  echo "  export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:\$LD_LIBRARY_PATH"
  echo "  export PATH=${INSTALL_PREFIX}/bin:\$PATH"
  echo "  export PKG_CONFIG_PATH=${INSTALL_PREFIX}/lib/pkgconfig:\$PKG_CONFIG_PATH"
  {
    printf "\n# Gazebo (source install) environment (added by tutorial-faithful script)\n"
    printf "export LD_LIBRARY_PATH=%s/lib:\$LD_LIBRARY_PATH\n" "${INSTALL_PREFIX}"
    printf "export PATH=%s/bin:\$PATH\n" "${INSTALL_PREFIX}"
    printf "export PKG_CONFIG_PATH=%s/lib/pkgconfig:\$PKG_CONFIG_PATH\n" "${INSTALL_PREFIX}"
  } >> "${HOME}/.bashrc"
  # shellcheck disable=SC1090
  source "${HOME}/.bashrc"
fi

echo "=== Done. Try running 'gazebo' as the tutorial suggests. ==="
