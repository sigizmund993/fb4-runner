/**
 * yolo26_post.cpp — YOLO26n postprocessing for hailofilter
 *
 * Does:
 *   1. Decode all grid cells above conf_threshold (sigmoid + dequant)
 *   2. NMS to suppress duplicates
 *   3. Write best detection to HailoROI (for hailooverlay)
 *   4. Print best detection to stderr (console)
 *
 * Compiled as shared library, called by GStreamer hailofilter element.
 * No Python involved.
 */

#include <vector>
#include <cmath>
#include <string>
#include <cstdio>
#include <algorithm>
#include <chrono>

#include "hailo_objects.hpp"
#include "hailo_tensors.hpp"

// ---------------------------------------------------------------------------
// Model constants
// ---------------------------------------------------------------------------
static const int   INPUT_H    = 960;
static const int   INPUT_W    = 1280;
static const int   STRIDES[3] = {8, 16, 32};
static const char* CLS_NAMES[3] = {
    "yolo26n/conv64",
    "yolo26n/conv80",
    "yolo26n/conv94",
};
static const char* REG_NAMES[3] = {
    "yolo26n/conv61",
    "yolo26n/conv77",
    "yolo26n/conv91",
};

static const float CONF_THRESHOLD = 0.25f;
static const float IOU_THRESHOLD  = 0.45f;

// ---------------------------------------------------------------------------
inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// ---------------------------------------------------------------------------
struct Det {
    float x1, y1, x2, y2;  // absolute pixels
    float conf;
};

// ---------------------------------------------------------------------------
static float iou(const Det& a, const Det& b) {
    float ix1 = std::max(a.x1, b.x1);
    float iy1 = std::max(a.y1, b.y1);
    float ix2 = std::min(a.x2, b.x2);
    float iy2 = std::min(a.y2, b.y2);
    float iw  = std::max(0.0f, ix2 - ix1);
    float ih  = std::max(0.0f, iy2 - iy1);
    float inter = iw * ih;
    float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    float uni = area_a + area_b - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

static std::vector<Det> nms(std::vector<Det>& dets, float iou_thr) {
    // sort descending by conf
    std::sort(dets.begin(), dets.end(),
              [](const Det& a, const Det& b){ return a.conf > b.conf; });

    std::vector<bool> suppressed(dets.size(), false);
    std::vector<Det>  keep;

    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        keep.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (!suppressed[j] && iou(dets[i], dets[j]) >= iou_thr)
                suppressed[j] = true;
        }
    }
    return keep;
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Frame counter + FPS tracking (static — persists across calls)
// ---------------------------------------------------------------------------
static uint64_t s_frame_count = 0;
static float    s_fps         = 0.0f;
static auto     s_fps_ts      = std::chrono::steady_clock::now();

extern "C" void filter(HailoROIPtr roi, void* /*extra_param*/) {
    auto t0 = std::chrono::steady_clock::now();

    if (!roi) return;

    auto tensors = roi->get_tensors();
    if (tensors.empty()) return;

    // --- Find tensors by name ---
    HailoTensorPtr cls_t[3] = {};
    HailoTensorPtr reg_t[3] = {};
    for (auto& t : tensors) {
        if (!t) continue;
        const std::string& name = t->name();
        for (int s = 0; s < 3; ++s) {
            if (name == CLS_NAMES[s]) cls_t[s] = t;
            if (name == REG_NAMES[s]) reg_t[s] = t;
        }
    }

    // --- Decode all grid cells ---
    std::vector<Det> raw;
    raw.reserve(512);

    for (int s = 0; s < 3; ++s) {
        if (!cls_t[s] || !reg_t[s]) continue;

        int stride = STRIDES[s];
        int gh     = INPUT_H / stride;
        int gw     = INPUT_W / stride;

        uint8_t* cls_raw = cls_t[s]->data();
        uint8_t* reg_raw = reg_t[s]->data();
        if (!cls_raw || !reg_raw) continue;

        auto cls_q  = cls_t[s]->quant_info();
        auto reg_q  = reg_t[s]->quant_info();
        float cs    = cls_q.qp_scale, cz = cls_q.qp_zp;
        float rs    = reg_q.qp_scale, rz = reg_q.qp_zp;

        for (int row = 0; row < gh; ++row) {
            for (int col = 0; col < gw; ++col) {
                int idx  = row * gw + col;
                float conf = sigmoid((cls_raw[idx] - cz) * cs);
                if (conf <= CONF_THRESHOLD) continue;

                float cx = (col + 0.5f) * stride;
                float cy = (row + 0.5f) * stride;
                float l  = (reg_raw[idx*4+0] - rz) * rs * stride;
                float t  = (reg_raw[idx*4+1] - rz) * rs * stride;
                float r  = (reg_raw[idx*4+2] - rz) * rs * stride;
                float b  = (reg_raw[idx*4+3] - rz) * rs * stride;

                Det d;
                d.x1   = std::max(0.0f, std::min((float)INPUT_W, cx - l));
                d.y1   = std::max(0.0f, std::min((float)INPUT_H, cy - t));
                d.x2   = std::max(0.0f, std::min((float)INPUT_W, cx + r));
                d.y2   = std::max(0.0f, std::min((float)INPUT_H, cy + b));
                d.conf = conf;
                if (d.x2 > d.x1 && d.y2 > d.y1)
                    raw.push_back(d);
            }
        }
    }

    if (raw.empty()) {
        fprintf(stderr, "[yolo26] no detections\n");
        return;
    }

    // --- NMS ---
    std::vector<Det> kept = nms(raw, IOU_THRESHOLD);

    // --- Write to HailoROI (for hailooverlay) ---
    // Only best detection — draw crosshair via overlay
    const Det& best = kept[0];
    float norm_cx = (best.x1 + best.x2) * 0.5f / INPUT_W;
    float norm_cy = (best.y1 + best.y2) * 0.5f / INPUT_H;
    // tiny 1px box at center — hailooverlay will draw it
    float dot = 1.0f / INPUT_W;
    roi->add_object(std::make_shared<HailoDetection>(
        HailoBBox(norm_cx - dot, norm_cy - dot, dot*2, dot*2),
        0, "ball", best.conf
    ));

    // --- Timing end + FPS ---
    auto t1 = std::chrono::steady_clock::now();
    float post_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    ++s_frame_count;
    float elapsed = std::chrono::duration<float>(t1 - s_fps_ts).count();
    if (elapsed >= 1.0f) {
        s_fps    = s_frame_count / elapsed;
        s_frame_count = 0;
        s_fps_ts = t1;
    }

    // --- Console output ---
    float cx_px = (best.x1 + best.x2) * 0.5f;
    float cy_px = (best.y1 + best.y2) * 0.5f;
    fprintf(stderr,
        "[yolo26] frame=%5.1ffps  post=%5.2fms  "
        "dets=%zu  best=%.1f%%  center=(%.0f,%.0f)\n",
        s_fps, post_ms,
        kept.size(), best.conf * 100.0f,
        cx_px, cy_px
    );
}