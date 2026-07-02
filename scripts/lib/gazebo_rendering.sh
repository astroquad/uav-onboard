#!/usr/bin/env bash

# Select and verify the renderer used by Gazebo / Ogre. Supported overrides:
#   GAZEBO_RENDER_MODE=auto  use NVIDIA when available, otherwise CPU (default)
#   GAZEBO_RENDER_MODE=gpu   require an NVIDIA GPU and hardware OpenGL
#   GAZEBO_RENDER_MODE=cpu   force Mesa software rendering
configure_gazebo_rendering() {
    local mode="${GAZEBO_RENDER_MODE:-auto}"
    local renderer
    local gpu_name

    case "$mode" in
        auto|gpu|cpu) ;;
        *) echo "오류: GAZEBO_RENDER_MODE는 auto, gpu, cpu 중 하나여야 합니다: $mode" >&2; return 1 ;;
    esac

    gpu_name="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 || true)"
    if [[ "$mode" == "cpu" || -z "$gpu_name" ]]; then
        if [[ "$mode" == "gpu" ]]; then
            echo "오류: GAZEBO_RENDER_MODE=gpu이지만 NVIDIA GPU를 찾지 못했습니다." >&2
            return 1
        fi
        export LIBGL_ALWAYS_SOFTWARE=1
        unset MESA_D3D12_DEFAULT_ADAPTER_NAME __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME
        export GAZEBO_RENDER_BACKEND="cpu"
        echo "    Gazebo renderer: CPU software (${mode}; NVIDIA GPU 없음 또는 CPU 강제)"
        return 0
    fi

    unset LIBGL_ALWAYS_SOFTWARE
    if [[ -e /dev/dxg ]]; then
        # WSLg exposes OpenGL acceleration through Mesa's D3D12 driver.
        export GALLIUM_DRIVER="d3d12"
        export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"
    else
        # Prefer the discrete NVIDIA GPU on PRIME-capable native Linux systems.
        unset GALLIUM_DRIVER MESA_D3D12_DEFAULT_ADAPTER_NAME
        export __NV_PRIME_RENDER_OFFLOAD=1
        export __GLX_VENDOR_LIBRARY_NAME=nvidia
    fi

    command -v glxinfo >/dev/null || {
        echo "오류: GPU 렌더러 검사용 glxinfo가 없습니다. setup_gazebo_sitl.sh를 다시 실행하세요." >&2
        return 1
    }
    renderer="$(glxinfo -B 2>/dev/null | sed -n 's/^[[:space:]]*OpenGL renderer string:[[:space:]]*//p' | head -1)"
    if [[ -z "$renderer" ]]; then
        echo "오류: OpenGL renderer를 확인하지 못했습니다. DISPLAY/WSLg 상태를 확인하세요." >&2
        return 1
    fi
    if [[ "$renderer" =~ llvmpipe|softpipe|Software[[:space:]]Rasterizer ]]; then
        echo "오류: NVIDIA GPU는 감지됐지만 OpenGL이 CPU renderer를 사용합니다: $renderer" >&2
        return 1
    fi
    if [[ ! "$renderer" =~ NVIDIA|Nvidia|nvidia|D3D12.*NVIDIA|D3D12.*Nvidia ]]; then
        echo "오류: NVIDIA GPU 대신 다른 OpenGL renderer가 선택됐습니다: $renderer" >&2
        return 1
    fi

    export GAZEBO_RENDER_BACKEND="nvidia"
    echo "    Gazebo renderer: NVIDIA GPU ($gpu_name)"
    echo "    OpenGL renderer: $renderer"
}
