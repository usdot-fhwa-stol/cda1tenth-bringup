#!/usr/bin/env bash
set -e

detect_gpu() {
    if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        if docker info 2>/dev/null | grep -qi "nvidia"; then
            return 0
        fi
    fi
    return 1
}

detect_wsl() {
    grep -qiE "(microsoft|wsl)" /proc/version 2>/dev/null
}

GPU_MODE=""
REBUILD=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --gpu)     GPU_MODE="nvidia"; shift ;;
        --no-gpu)  GPU_MODE="none";   shift ;;
        --rebuild) REBUILD=true;      shift ;;
        --help|-h)
            echo "Usage: ./launch.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --gpu       Force NVIDIA GPU mode"
            echo "  --no-gpu    Force non-GPU mode"
            echo "  --rebuild   Rebuild docker images before launching"
            exit 0
            ;;
        *)
            echo "Unknown option: $1 (try --help)"
            exit 1
            ;;
    esac
done

if [[ -z "$GPU_MODE" ]]; then
    if detect_wsl; then
        echo "[launch] Detected WSL — using non-GPU mode"
        GPU_MODE="none"
    elif detect_gpu; then
        echo "[launch] Detected NVIDIA GPU + runtime — using GPU mode"
        GPU_MODE="nvidia"
    else
        echo "[launch] No NVIDIA GPU or Container Toolkit detected — using non-GPU mode"
        GPU_MODE="none"
    fi
fi

xhost +local:docker > /dev/null

COMPOSE_FILES="-f docker-compose.yml"
if [[ "$GPU_MODE" == "nvidia" ]]; then
    export LIBGL_ALWAYS_SOFTWARE=0
    COMPOSE_FILES="$COMPOSE_FILES -f docker-compose.nvidia.yml"
fi

if [ "$REBUILD" = true ]; then
    echo "[launch] Rebuilding images..."
    docker compose $COMPOSE_FILES build
fi

echo "[launch] Executing: docker compose $COMPOSE_FILES up"

exec docker compose $COMPOSE_FILES up
