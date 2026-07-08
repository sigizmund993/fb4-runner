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

HEF="/root/rpi_fb4-runner/10_hours_brainrot.hef"
SO="/root/rpi_fb4-runner/libyolo26_post.so"
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

if [ "$STREAM" -eq 1 ]; then
    echo "[INFO] Starting mediaMTX..."
    "$MEDIAMTX" "$MEDIAMTX_CFG" &
    MEDIAMTX_PID=$!
    sleep 1

    IP=$(hostname -I | awk '{print $1}')
    echo "[INFO] Stream: rtsp://${IP}:8554/ball"

    # GStreamer пушит MJPEG в mediaMTX через RTSP RECORD
    SINK="queue leaky=downstream max-size-buffers=2 ! \
hailooverlay ! \
videoconvert ! \
jpegenc quality=${QUALITY} ! \
rtpjpegpay ! \
rtspclientsink location=rtsp://127.0.0.1:8554/ball protocols=tcp"
else
    SINK="fakesink sync=false"
    echo "[INFO] No-stream mode — console only"
fi

echo "[INFO] HEF:     $HEF"
echo "[INFO] SO:      $SO"
echo "[INFO] Quality: $QUALITY"
echo ""

gst-launch-1.0 -e \
    libcamerasrc ! \
    videoconvert ! \
    "video/x-raw,format=RGB,width=1296,height=972,framerate=30/1" ! \
    videoscale ! \
    "video/x-raw,format=RGB,width=1280,height=960" ! \
    queue leaky=downstream max-size-buffers=2 ! \
    hailonet \
        hef-path="$HEF" \
        batch-size=1 \
        scheduling-algorithm=1 ! \
    queue leaky=downstream max-size-buffers=2 ! \
    hailofilter \
        so-path="$SO" \
        qos=false ! \
    $SINK