#!/usr/bin/env bash
# Set up the Linux/WSL side of Astroquad Gazebo + ArduCopter SITL.
#
# Defaults assume:
#   ~/astroquad/uav-onboard      this repository
#   ~/ardupilot                  ArduPilot checkout
#   ~/ardupilot_gazebo           ArduPilot Gazebo plugin checkout
#
# Useful overrides:
#   ONBOARD_DIR=/path/to/uav-onboard
#   ARDUPILOT_DIR=/path/to/ardupilot
#   ARDUPILOT_GAZEBO_DIR=/path/to/ardupilot_gazebo
#   UPDATE_EXISTING=1            run git pull --ff-only in existing external repos
#   BUILD_ONBOARD=0              skip uav-onboard CMake build

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ONBOARD_DIR="${ONBOARD_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUCOPTER_DIR="${ARDUCOPTER_DIR:-$ARDUPILOT_DIR/ArduCopter}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
UPDATE_EXISTING="${UPDATE_EXISTING:-0}"
BUILD_ONBOARD="${BUILD_ONBOARD:-1}"
JOBS="${JOBS:-$(nproc)}"

ARDUPILOT_REPO="${ARDUPILOT_REPO:-https://github.com/ArduPilot/ardupilot.git}"
ARDUPILOT_GAZEBO_REPO="${ARDUPILOT_GAZEBO_REPO:-https://github.com/ArduPilot/ardupilot_gazebo.git}"

die() {
    echo "error: $*" >&2
    exit 1
}

info() {
    echo
    echo "==> $*"
}

require_ubuntu() {
    if [[ ! -r /etc/os-release ]]; then
        die "this setup script expects Ubuntu under WSL/Linux"
    fi
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${ID:-}" != "ubuntu" ]]; then
        die "this setup script expects Ubuntu; detected ID=${ID:-unknown}"
    fi

    case "${VERSION_CODENAME:-}" in
        jammy|noble)
            ;;
        *)
            echo "warning: tested on Ubuntu jammy/noble; detected ${VERSION_CODENAME:-unknown}" >&2
            echo "warning: continuing, but Gazebo Harmonic packages may not be available." >&2
            ;;
    esac
}

install_osrf_repo() {
    info "Installing Gazebo OSRF apt repository"
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg lsb-release
    sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
    sudo apt-get update
}

install_packages() {
    info "Installing Gazebo Harmonic, build tools, OpenCV, and plugin dependencies"
    sudo apt-get install -y \
        build-essential \
        ccache \
        cmake \
        g++ \
        gcc \
        gdb \
        git \
        git-gui \
        gitk \
        gstreamer1.0-gl \
        gstreamer1.0-libav \
        gstreamer1.0-plugins-bad \
        gz-harmonic \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev \
        libgz-msgs10-dev \
        libgz-sim8-dev \
        libgz-transport13-dev \
        libopencv-dev \
        make \
        ninja-build \
        pkg-config \
        python3 \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-venv \
        python3-wheel \
        rapidjson-dev
}

clone_or_update() {
    local repo_url="$1"
    local dst="$2"
    local name="$3"

    if [[ -d "$dst/.git" ]]; then
        info "$name already exists at $dst"
        if [[ "$UPDATE_EXISTING" == "1" ]]; then
            git -C "$dst" pull --ff-only
        else
            echo "Leaving existing checkout untouched. Set UPDATE_EXISTING=1 to fast-forward it."
        fi
    elif [[ -e "$dst" ]]; then
        die "$dst exists but is not a git checkout"
    else
        info "Cloning $name into $dst"
        git clone --recurse-submodules "$repo_url" "$dst"
    fi

    git -C "$dst" submodule update --init --recursive
}

install_ardupilot_prereqs() {
    info "Installing ArduPilot build/SITL prerequisites"
    [[ -x "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" ]] \
        || die "missing ArduPilot prereq script under $ARDUPILOT_DIR"
    (
        cd "$ARDUPILOT_DIR"
        Tools/environment_install/install-prereqs-ubuntu.sh -y
    )
}

build_ardupilot_gazebo() {
    info "Building ardupilot_gazebo plugin"
    export GZ_VERSION=harmonic
    cmake -S "$ARDUPILOT_GAZEBO_DIR" \
        -B "$ARDUPILOT_GAZEBO_DIR/build" \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "$ARDUPILOT_GAZEBO_DIR/build" --parallel "$JOBS"
}

write_bashrc_block() {
    local bashrc="$HOME/.bashrc"
    local marker_begin="# >>> astroquad gazebo sitl >>>"
    local marker_end="# <<< astroquad gazebo sitl <<<"
    local tmp
    tmp="$(mktemp)"

    if [[ -f "$bashrc" ]]; then
        awk -v begin="$marker_begin" -v end="$marker_end" '
            $0 == begin {skip=1; next}
            $0 == end {skip=0; next}
            !skip {print}
        ' "$bashrc" >"$tmp"
    fi

    {
        cat "$tmp"
        echo "$marker_begin"
        echo "export ARDUPILOT_DIR=\"$ARDUPILOT_DIR\""
        echo "export ARDUCOPTER_DIR=\"$ARDUCOPTER_DIR\""
        echo "export ARDUPILOT_GAZEBO_DIR=\"$ARDUPILOT_GAZEBO_DIR\""
        echo "export PATH=\"\$ARDUPILOT_DIR/Tools/autotest:\$HOME/.local/bin:\$PATH\""
        echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\"\$ARDUPILOT_GAZEBO_DIR/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}\""
        echo "export GZ_SIM_RESOURCE_PATH=\"\$ARDUPILOT_GAZEBO_DIR/models:\$ARDUPILOT_GAZEBO_DIR/worlds:\${GZ_SIM_RESOURCE_PATH:-}\""
        echo "export SDF_PATH=\"$ONBOARD_DIR/sim/gazebo/models:$ONBOARD_DIR/sim/gazebo/worlds:\$ARDUPILOT_GAZEBO_DIR/models:\$ARDUPILOT_GAZEBO_DIR/worlds:\${SDF_PATH:-}\""
        echo "$marker_end"
    } >"$tmp.new"

    install -m 0644 "$tmp.new" "$bashrc"
    rm -f "$tmp" "$tmp.new"
}

build_onboard() {
    if [[ "$BUILD_ONBOARD" != "1" ]]; then
        info "Skipping uav-onboard build because BUILD_ONBOARD=$BUILD_ONBOARD"
        return
    fi

    info "Building uav-onboard"
    cmake -S "$ONBOARD_DIR" -B "$ONBOARD_DIR/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTS=ON
    cmake --build "$ONBOARD_DIR/build" --parallel "$JOBS"
}

validate_setup() {
    info "Validating setup"
    export PATH="$ARDUPILOT_DIR/Tools/autotest:$HOME/.local/bin:$PATH"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"
    export SDF_PATH="$ONBOARD_DIR/sim/gazebo/models:$ONBOARD_DIR/sim/gazebo/worlds:$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:${SDF_PATH:-}"

    command -v gz >/dev/null || die "gz command was not found"
    command -v sim_vehicle.py >/dev/null || die "sim_vehicle.py was not found in PATH"
    [[ -d "$ARDUCOPTER_DIR" ]] || die "ArduCopter directory was not found: $ARDUCOPTER_DIR"
    [[ -f "$ONBOARD_DIR/sim/gazebo/params/indoor_flow_tfmini_sitl.parm" ]] \
        || die "repo-local SITL param file is missing"

    gz sim --versions
    gz sdf --check "$ONBOARD_DIR/sim/gazebo/worlds/grid_arena_test_world.sdf"
    gz sdf --check "$ONBOARD_DIR/sim/gazebo/worlds/line_tracing_test_world.sdf"
}

main() {
    require_ubuntu
    install_osrf_repo
    install_packages
    clone_or_update "$ARDUPILOT_REPO" "$ARDUPILOT_DIR" "ArduPilot"
    install_ardupilot_prereqs
    clone_or_update "$ARDUPILOT_GAZEBO_REPO" "$ARDUPILOT_GAZEBO_DIR" "ardupilot_gazebo"
    build_ardupilot_gazebo
    write_bashrc_block
    build_onboard
    validate_setup

    info "Done"
    echo "Open a new WSL terminal or run: source ~/.bashrc"
    echo "Then start a world, for example:"
    echo "  bash \"$ONBOARD_DIR/scripts/grid_arena_test.sh\""
}

main "$@"
