#!/usr/bin/env bash
set -euo pipefail

# One-shot setup for Jetson NX / Ubuntu 18.04 aarch64.
# Installs the toolchain and third-party dependencies needed by dart2026,
# clones the repository to ~/Desktop/dart, and builds it.

UBUNTU_PORTS_MIRROR="${UBUNTU_PORTS_MIRROR:-http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports}"
REPO_URL="${REPO_URL:-https://github.com/Misaka21/dart.git}"
REPO_DIR="${REPO_DIR:-$HOME/Desktop/dart}"
JOBS="${JOBS:-4}"

CMAKE_VERSION="${CMAKE_VERSION:-3.27.9}"
FMT_VERSION="${FMT_VERSION:-8.1.1}"
PYBIND_VERSION="${PYBIND_VERSION:-2.12.0}"
OPENCV_VERSION="${OPENCV_VERSION:-4.5.5}"
MVS_TAG="${MVS_TAG:-v5.0.0-20260421}"

CMAKE_PREFIX="/opt/cmake-${CMAKE_VERSION}"
FMT_PREFIX="/opt/fmt-${FMT_VERSION}"
PYBIND_PREFIX="/opt/pybind11-${PYBIND_VERSION}"
OPENCV_PREFIX="/opt/opencv-${OPENCV_VERSION}"

sudo_keepalive_pid=""

log() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

cleanup() {
  if [ -n "${sudo_keepalive_pid}" ]; then
    kill "${sudo_keepalive_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

require_ubuntu18_aarch64() {
  if [ "$(uname -m)" != "aarch64" ]; then
    echo "This script is intended for aarch64 Jetson NX." >&2
    exit 1
  fi

  if ! grep -q '^VERSION_ID="18.04"' /etc/os-release; then
    echo "This script is intended for Ubuntu 18.04." >&2
    exit 1
  fi
}

init_sudo() {
  log "Requesting sudo"
  sudo -v
  while true; do
    sudo -n true
    sleep 60
  done &
  sudo_keepalive_pid="$!"
}

configure_apt_sources() {
  log "Configuring apt sources"

  if [ -f /etc/apt/sources.list ] && [ ! -f /etc/apt/sources.list.backup-by-dart-setup ]; then
    sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup-by-dart-setup
  fi

  cat >/tmp/dart_sources.list <<EOF
deb ${UBUNTU_PORTS_MIRROR}/ bionic main restricted universe multiverse
deb ${UBUNTU_PORTS_MIRROR}/ bionic-updates main restricted universe multiverse
deb ${UBUNTU_PORTS_MIRROR}/ bionic-backports main restricted universe multiverse
deb ${UBUNTU_PORTS_MIRROR}/ bionic-security main restricted universe multiverse
EOF
  sudo mv /tmp/dart_sources.list /etc/apt/sources.list

  if [ -f /etc/apt/sources.list.d/nvidia-l4t-apt-source.list ]; then
    if [ ! -f /etc/apt/sources.list.d/nvidia-l4t-apt-source.list.backup-by-dart-setup ]; then
      sudo cp /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
        /etc/apt/sources.list.d/nvidia-l4t-apt-source.list.backup-by-dart-setup
    fi
    sudo sed -i 's/^deb /# deb /' /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
  fi

  sudo apt-get update
}

install_apt_packages() {
  log "Installing apt packages"

  sudo apt-get install -y --no-install-recommends \
    ca-certificates gnupg dirmngr software-properties-common apt-transport-https \
    build-essential git wget unzip tar xz-utils pkg-config ninja-build ccache \
    libeigen3-dev libusb-1.0-0-dev \
    python3.8 python3.8-dev python3.8-venv \
    libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libgtk-3-dev libcanberra-gtk3-dev libdc1394-22-dev \
    libopenblas-dev liblapack-dev libtbb-dev
}

install_gcc10() {
  log "Installing GCC/G++ 10"

  if ! apt-cache search --names-only '^g\+\+-10$' | grep -q '^g++-10 '; then
    sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
    sudo apt-get update
  fi

  sudo apt-get install -y gcc-10 g++-10
  sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-10
  sudo update-alternatives --set gcc /usr/bin/gcc-10

  gcc --version | head -1
  g++ --version | head -1
}

install_cmake() {
  log "Installing CMake ${CMAKE_VERSION}"

  if [ ! -x "${CMAKE_PREFIX}/bin/cmake" ]; then
    cd /tmp
    rm -rf "cmake-${CMAKE_VERSION}-linux-aarch64" "cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz"
    wget -q --show-progress \
      -O "cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz" \
      "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz"
    tar -xzf "cmake-${CMAKE_VERSION}-linux-aarch64.tar.gz"
    sudo rm -rf "${CMAKE_PREFIX}"
    sudo mv "cmake-${CMAKE_VERSION}-linux-aarch64" "${CMAKE_PREFIX}"
  fi

  sudo ln -sf "${CMAKE_PREFIX}/bin/cmake" /usr/local/bin/cmake
  sudo ln -sf "${CMAKE_PREFIX}/bin/ctest" /usr/local/bin/ctest
  sudo ln -sf "${CMAKE_PREFIX}/bin/cpack" /usr/local/bin/cpack
  cmake --version | head -1
}

install_python_packages() {
  log "Installing Python 3.8 packages"

  if ! python3.8 -m pip --version >/dev/null 2>&1; then
    cd /tmp
    wget -q -O get-pip.py https://bootstrap.pypa.io/pip/3.8/get-pip.py
    python3.8 get-pip.py --user
  fi

  python3.8 -m pip install --user --upgrade "pip<25" setuptools wheel
  python3.8 -m pip install --user "numpy==1.24.4" "Flask<3.1"

  python3.8 - <<'PY'
import flask
import numpy
print("python deps ok", flask.__version__, numpy.__version__)
PY
}

install_fmt() {
  log "Installing fmt ${FMT_VERSION}"

  if [ -f "${FMT_PREFIX}/lib/cmake/fmt/fmt-config.cmake" ]; then
    return
  fi

  cd /tmp
  rm -rf "fmt-${FMT_VERSION}" "fmt-${FMT_VERSION}.tar.gz" fmt-build
  wget -q --show-progress \
    -O "fmt-${FMT_VERSION}.tar.gz" \
    "https://github.com/fmtlib/fmt/archive/refs/tags/${FMT_VERSION}.tar.gz"
  tar -xzf "fmt-${FMT_VERSION}.tar.gz"

  cmake -S "fmt-${FMT_VERSION}" -B fmt-build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
    -DCMAKE_INSTALL_PREFIX="${FMT_PREFIX}" \
    -DFMT_TEST=OFF -DFMT_DOC=OFF
  cmake --build fmt-build --parallel "${JOBS}"
  sudo cmake --install fmt-build
}

install_pybind11() {
  log "Installing pybind11 ${PYBIND_VERSION}"

  if [ -f "${PYBIND_PREFIX}/share/cmake/pybind11/pybind11Config.cmake" ]; then
    return
  fi

  cd /tmp
  rm -rf "pybind11-${PYBIND_VERSION}" "pybind11-${PYBIND_VERSION}.tar.gz" pybind11-build
  wget -q --show-progress \
    -O "pybind11-${PYBIND_VERSION}.tar.gz" \
    "https://github.com/pybind/pybind11/archive/refs/tags/v${PYBIND_VERSION}.tar.gz"
  tar -xzf "pybind11-${PYBIND_VERSION}.tar.gz"

  cmake -S "pybind11-${PYBIND_VERSION}" -B pybind11-build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
    -DCMAKE_INSTALL_PREFIX="${PYBIND_PREFIX}" \
    -DPYBIND11_TEST=OFF
  cmake --build pybind11-build --parallel "${JOBS}"
  sudo cmake --install pybind11-build
}

install_mvs() {
  log "Installing HikRobot MVS SDK ${MVS_TAG}"

  if [ -f /opt/MVS/include/MvCameraControl.h ] && [ -f /opt/MVS/lib/aarch64/libMvCameraControl.so ]; then
    return
  fi

  cd /tmp
  rm -rf mvs-sdk-aarch64.tar.gz mvs-sdk
  wget -q --show-progress \
    -O mvs-sdk-aarch64.tar.gz \
    "https://github.com/Alliance-Algorithm/hik-mvs/releases/download/${MVS_TAG}/mvs-sdk-aarch64.tar.gz"
  mkdir -p mvs-sdk
  tar -xzf mvs-sdk-aarch64.tar.gz -C mvs-sdk

  sudo rm -rf /opt/MVS
  sudo mkdir -p /opt/MVS
  sudo cp -a mvs-sdk/. /opt/MVS/
  echo /opt/MVS/lib/aarch64 | sudo tee /etc/ld.so.conf.d/hik-mvs.conf >/dev/null
  sudo ldconfig
}

install_opencv() {
  log "Installing OpenCV ${OPENCV_VERSION}"

  if [ -f "${OPENCV_PREFIX}/lib/cmake/opencv4/OpenCVConfig.cmake" ]; then
    return
  fi

  mkdir -p "$HOME/.ccache"
  : > "$HOME/.ccache/ccache.conf"

  cd /tmp
  rm -rf "opencv-${OPENCV_VERSION}" "opencv-${OPENCV_VERSION}.tar.gz" opencv-build
  wget -q --show-progress \
    -O "opencv-${OPENCV_VERSION}.tar.gz" \
    "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz"
  tar -xzf "opencv-${OPENCV_VERSION}.tar.gz"

  local numpy_include
  numpy_include="$(python3.8 -c 'import numpy; print(numpy.get_include())')"

  cmake -S "opencv-${OPENCV_VERSION}" -B opencv-build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=/usr/bin/gcc-10 \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
    -DCMAKE_INSTALL_PREFIX="${OPENCV_PREFIX}" \
    -DCMAKE_INSTALL_RPATH="${OPENCV_PREFIX}/lib" \
    -DOPENCV_GENERATE_PKGCONFIG=ON \
    -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,calib3d,features2d,flann,python3 \
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF \
    -DWITH_FFMPEG=ON -DWITH_GTK=ON -DWITH_V4L=ON -DWITH_OPENGL=OFF \
    -DPYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3.8 \
    -DPYTHON3_EXECUTABLE=/usr/bin/python3.8 \
    -DPYTHON3_INCLUDE_DIR=/usr/include/python3.8 \
    -DPYTHON3_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so \
    -DPYTHON3_NUMPY_INCLUDE_DIRS="${numpy_include}" \
    -DPYTHON3_PACKAGES_PATH=/usr/local/lib/python3.8/dist-packages

  cmake --build opencv-build --parallel "${JOBS}"
  sudo cmake --install opencv-build
  echo "${OPENCV_PREFIX}/lib" | sudo tee "/etc/ld.so.conf.d/opencv-${OPENCV_VERSION}.conf" >/dev/null
  sudo ldconfig

  PKG_CONFIG_PATH="${OPENCV_PREFIX}/lib/pkgconfig" pkg-config --modversion opencv4
  python3.8 - <<'PY'
import cv2
print("cv2 ok", cv2.__version__, cv2.__file__)
PY
}

clone_or_update_repo() {
  log "Cloning or updating repository"

  mkdir -p "$(dirname "${REPO_DIR}")"
  if [ ! -d "${REPO_DIR}/.git" ]; then
    rm -rf "${REPO_DIR}"
    git clone "${REPO_URL}" "${REPO_DIR}"
    return
  fi

  if [ -n "$(git -C "${REPO_DIR}" status --porcelain)" ]; then
    echo "Repository exists and has local changes: ${REPO_DIR}" >&2
    echo "Leaving it untouched; build will use the existing checkout." >&2
    return
  fi

  git -C "${REPO_DIR}" fetch origin main
  git -C "${REPO_DIR}" checkout main
  git -C "${REPO_DIR}" reset --hard origin/main
}

build_project() {
  log "Building dart2026"

  cmake -S "${REPO_DIR}" -B "${REPO_DIR}/build" -G Ninja \
    -DCMAKE_C_COMPILER=/usr/bin/gcc-10 \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++-10 \
    -DCMAKE_PREFIX_PATH="${OPENCV_PREFIX};${FMT_PREFIX};${PYBIND_PREFIX}" \
    -DOpenCV_DIR="${OPENCV_PREFIX}/lib/cmake/opencv4" \
    -Dfmt_DIR="${FMT_PREFIX}/lib/cmake/fmt" \
    -Dpybind11_DIR="${PYBIND_PREFIX}/share/cmake/pybind11" \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
    -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
    -DPython_EXECUTABLE=/usr/bin/python3.8 \
    -DPython_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so \
    -DPython_INCLUDE_DIR=/usr/include/python3.8

  cmake --build "${REPO_DIR}/build" --parallel "${JOBS}"

  cd "${REPO_DIR}/build"
  ldd ./dart2026 | grep "not found" && exit 1 || true
  ./dart2026 --help >/dev/null
  ls -lh dart2026 test_calibration test_serial
}

main() {
  require_ubuntu18_aarch64
  init_sudo
  configure_apt_sources
  install_apt_packages
  install_gcc10
  install_cmake
  install_python_packages
  install_fmt
  install_pybind11
  install_mvs
  install_opencv
  clone_or_update_repo
  build_project
  log "Done"
}

main "$@"
