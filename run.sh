#!/bin/bash
# Ball detector — GStreamer + Hailo + mediaMTX RTSP (MJPEG)
#
# Usage:
#   ./run.sh                          # MJPEG RTSP стрим
#   ./run.sh --no-stream              # только консоль
#   ./run.sh --quality=85             # качество JPEG (по умолч. 75)
#   ./run.sh path.hef path.so         # кастомные пути
#
# Смотреть:
#   VLC:    rtsp://<RPI_IP>:8554/ball
#   ffplay: ffplay rtsp://<RPI_IP>:8554/ball

# Settings from the environment, else the repository's .env (as src/utils/envfile.py), else the default
ENV_FILE="${FB4_ENV_FILE:-$(dirname "$(readlink -f "$0")")/.env}"
setting() {
    local name="$1" default="$2" value="${!1}"
    if [ -z "$value" ] && [ -f "$ENV_FILE" ]; then
        value=$(sed -n "s/^\(export \)\?$name=//p" "$ENV_FILE" | tail -n 1 | sed 's/ #.*//; s/^["'"'"']//; s/["'"'"']$//')
    fi
    echo "${value:-$default}"
}

# Camera: NV12 from the ISP, converted to RGB on the CPU in 2 threads. RGB straight from the ISP would skip the
# conversion, but hailonet crashes (SIGSEGV) on libcamerasrc's own buffers: videoconvert gives it a buffer of its own
# Exposure [us] and analogue gain, manual: a short exposure blurs a moving ball less, and the frame is fresher.
# auto: the camera's auto exposure (up to a frame, ~30 ms indoors)
CAM_EXPOSURE_US=$(setting FB4_CAM_EXPOSURE_US 5000)
CAM_GAIN=$(setting FB4_CAM_GAIN 8.0)
if [ "$CAM_EXPOSURE_US" = "auto" ]; then
    CAM_CONTROLS=""
else
    CAM_CONTROLS="exposure-time-mode=manual exposure-time=${CAM_EXPOSURE_US} analogue-gain-mode=manual analogue-gain=${CAM_GAIN}"
fi

HEF="/root/fb4-runner/10_hours_brainrot.hef"
SO="/root/fb4-runner/libyolo26_post.so"
MEDIAMTX="/root/mediamtx"
MEDIAMTX_CFG="/root/mediamtx.yml"
STREAM=1
QUALITY=75

for arg in "$@"; do
    case "$arg" in
        --no-stream)   STREAM=0 ;;
        --quality=*)   QUALITY="${arg#*=}" ;;
        *.hef)         HEF="$arg" ;;
        *.so)          SO="$arg" ;;
    esac
done

cleanup() {
    echo "[INFO] Stopping..."
    [ -n "$MEDIAMTX_PID" ] && kill "$MEDIAMTX_PID" 2>/dev/null
    wait "$MEDIAMTX_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

PREFIX=""

if [ "$STREAM" -eq 1 ]; then
    echo "[INFO] Starting mediaMTX..."
    "$MEDIAMTX" "$MEDIAMTX_CFG" &
    MEDIAMTX_PID=$!
    sleep 1

    IP=$(hostname -I | awk '{print $1}')
    echo "[INFO] Stream: rtsp://${IP}:8554/ball"

    PREFIX="rtspclientsink name=sink location=rtsp://127.0.0.1:8554/ball protocols=tcp latency=0"

    SINK="queue leaky=downstream max-size-buffers=2 ! \
hailooverlay ! \
videoconvert ! \
video/x-raw,format=I420 ! \
jpegenc quality=${QUALITY} ! \
queue leaky=downstream max-size-buffers=2 ! \
sink."
else
    SINK="fakesink sync=false"
    echo "[INFO] No-stream mode — console only"
fi

echo "[INFO] HEF:     $HEF"
echo "[INFO] SO:      $SO"
echo "[INFO] Quality: $QUALITY"
echo "[INFO] Camera:  exposure ${CAM_EXPOSURE_US} us, gain ${CAM_GAIN}"
echo ""

gst-launch-1.0 -e $PREFIX \
libcamerasrc $CAM_CONTROLS ! \
"video/x-raw,format=NV12,width=1280,height=960,framerate=30/1" ! \
videoconvert n-threads=2 ! \
"video/x-raw,format=RGB" ! \
queue leaky=upstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! \
hailonet \
hef-path="$HEF" \
batch-size=1 \
scheduling-algorithm=1 ! \
queue leaky=upstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! \
hailofilter \
so-path="$SO" \
qos=false ! \
$SINK

