#!/usr/bin/env bash
set -euo pipefail

install_dev_tools=false
apt_install_recommends=()

usage() {
    cat <<'USAGE'
Usage: bash scripts/setup_rpi_dependencies.sh [options]

Options:
  --no-recommends   Pass --no-install-recommends to apt-get install
  --with-dev-tools  Also install clang-format and shellcheck when available
  -h, --help        Show this help

Checks and installs packages needed to build and run uav-onboard on a clean
Raspberry Pi OS Lite 64-bit system.
USAGE
}

for arg in "$@"; do
    case "${arg}" in
        --no-recommends)
            apt_install_recommends+=(--no-install-recommends)
            ;;
        --with-dev-tools)
            install_dev_tools=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option '${arg}'" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if ! command -v apt-get >/dev/null 2>&1; then
    echo "error: apt-get was not found. This script expects Raspberry Pi OS or Debian." >&2
    exit 1
fi

SUDO=()
if [[ "${EUID}" -ne 0 ]]; then
    if ! command -v sudo >/dev/null 2>&1; then
        echo "error: sudo was not found. Run this script as root or install sudo first." >&2
        exit 1
    fi
    SUDO=(sudo)
fi

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    echo "Detected OS: ${PRETTY_NAME:-unknown}"
fi

arch="$(dpkg --print-architecture)"
if [[ "${arch}" != "arm64" ]]; then
    echo "warning: detected architecture '${arch}', expected 'arm64' for Raspberry Pi OS Lite 64-bit." >&2
fi

required_packages=(
    build-essential
    cmake
    pkg-config
    git
    libopencv-dev
    libserial-dev
    libv4l-dev
    v4l-utils
    ffmpeg
    nlohmann-json3-dev
    libtomlplusplus-dev
    libgtest-dev
)

echo "Updating apt package index..."
"${SUDO[@]}" apt-get update

optional_camera_packages=()
for camera_pkg in rpicam-apps libcamera-apps libcamera-tools; do
    if apt-cache show "${camera_pkg}" >/dev/null 2>&1; then
        optional_camera_packages+=("${camera_pkg}")
        break
    fi
done
if apt-cache show libcamera-dev >/dev/null 2>&1; then
    optional_camera_packages+=(libcamera-dev)
fi

optional_dev_packages=()
if [[ "${install_dev_tools}" == true ]]; then
    for pkg in clang-format shellcheck gdb rsync; do
        if apt-cache show "${pkg}" >/dev/null 2>&1; then
            optional_dev_packages+=("${pkg}")
        fi
    done
fi

all_packages=("${required_packages[@]}" "${optional_camera_packages[@]}" "${optional_dev_packages[@]}")

missing_packages=()
for pkg in "${all_packages[@]}"; do
    if ! dpkg-query -W -f='${Status}' "${pkg}" 2>/dev/null | grep -q "install ok installed"; then
        missing_packages+=("${pkg}")
    fi
done

if [[ "${#missing_packages[@]}" -eq 0 ]]; then
    echo "All required packages are already installed."
else
    echo "Installing missing packages:"
    printf '  %s\n' "${missing_packages[@]}"
    "${SUDO[@]}" apt-get install -y "${apt_install_recommends[@]}" "${missing_packages[@]}"
fi

cat <<'NOTES'

Dependency setup complete.

Notes:
- MAVLink C headers are intentionally not installed here. Vendor or generate
  them later under the project when the exact dialect/version is selected.
- If camera access fails, enable the camera interface and reboot:
    sudo raspi-config
- Build check:
    cmake -S . -B build -DBUILD_TESTS=OFF
    cmake --build build
NOTES
