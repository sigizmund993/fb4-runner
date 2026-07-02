#include <vector>
#include <cmath>
#include <string>
#include <cstdio>
#include <algorithm>
#include "hailo_objects.hpp"
#include "hailo_tensors.hpp"

const int INPUT_H = 960;
const int INPUT_W = 1280;
const int STRIDES[3] = {8, 16, 32};
const char* CLS_NAMES[3] = {"yolo26n/conv64", "yolo26n/conv80", "yolo26n/conv94"};
const char* REG_NAMES[3] = {"yolo26n/conv61", "yolo26n/conv77", "yolo26n/conv91"};

inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

extern "C" void filter(HailoROIPtr roi, void *extra_param) {
    if (!roi) return;
    auto tensors = roi->get_tensors();
    if (tensors.empty()) return;

    HailoTensorPtr cls_tensors[3] = {nullptr, nullptr, nullptr};
    HailoTensorPtr reg_tensors[3] = {nullptr, nullptr, nullptr};

    for (auto &tensor : tensors) {
        if (!tensor) continue;
        std::string name = tensor->name();
        for (int s = 0; s < 3; ++s) {
            if (name == CLS_NAMES[s]) cls_tensors[s] = tensor;
            if (name == REG_NAMES[s]) reg_tensors[s] = tensor;
        }
    }

    float conf_threshold = 0.1f;
    float best_conf = 0;
    float best_cx = -1, best_cy = -1;
    float best_x0 = 0, best_y0 = 0, best_x1 = 0, best_y1 = 0;
    int det_count = 0;

    for (int i = 0; i < 3; ++i) {
        if (!cls_tensors[i] || !reg_tensors[i]) continue;

        int stride = STRIDES[i];
        int gh = INPUT_H / stride;
        int gw = INPUT_W / stride;

        uint8_t* cls_raw = cls_tensors[i]->data();
        uint8_t* reg_raw = reg_tensors[i]->data();
        if (!cls_raw || !reg_raw) continue;

        auto cls_q = cls_tensors[i]->quant_info();
        auto reg_q = reg_tensors[i]->quant_info();
        float cls_scale = cls_q.qp_scale;
        float cls_zp = cls_q.qp_zp;
        float reg_scale = reg_q.qp_scale;
        float reg_zp = reg_q.qp_zp;

        for (int row = 0; row < gh; ++row) {
            for (int col = 0; col < gw; ++col) {
                int idx = row * gw + col;
                float conf = sigmoid((cls_raw[idx] - cls_zp) * cls_scale);
                if (conf <= conf_threshold) continue;

                det_count++;
                float cx = (col + 0.5f) * stride;
                float cy = (row + 0.5f) * stride;

                float reg_x1 = (reg_raw[idx * 4 + 0] - reg_zp) * reg_scale;
                float reg_y1 = (reg_raw[idx * 4 + 1] - reg_zp) * reg_scale;
                float reg_x2 = (reg_raw[idx * 4 + 2] - reg_zp) * reg_scale;
                float reg_y2 = (reg_raw[idx * 4 + 3] - reg_zp) * reg_scale;

                float x1 = std::max(0.0f, std::min((float)INPUT_W, cx - reg_x1 * stride));
                float y1 = std::max(0.0f, std::min((float)INPUT_H, cy - reg_y1 * stride));
                float x2 = std::max(0.0f, std::min((float)INPUT_W, cx + reg_x2 * stride));
                float y2 = std::max(0.0f, std::min((float)INPUT_H, cy + reg_y2 * stride));

                float norm_x = x1 / INPUT_W;
                float norm_y = y1 / INPUT_H;
                float norm_w = (x2 - x1) / INPUT_W;
                float norm_h = (y2 - y1) / INPUT_H;
                if (norm_w <= 0 || norm_h <= 0) continue;

                roi->add_object(std::make_shared<HailoDetection>(
                    HailoBBox(norm_x, norm_y, norm_w, norm_h), 0, "ball", conf
                ));

                if (conf > best_conf) {
                    best_conf = conf;
                    best_cx = (x1 + x2) * 0.5f;
                    best_cy = (y1 + y2) * 0.5f;
                    best_x0 = x1;
                    best_y0 = y1;
                    best_x1 = x2;
                    best_y1 = y2;
                }
            }
        }
    }

    fprintf(stderr, "[yolo26] dets=%d best=%.1f%% center=(%.0f,%.0f) box=(%.0f,%.0f)-(%.0f,%.0f)\n",
        det_count, best_conf * 100, best_cx, best_cy, best_x0, best_y0, best_x1, best_y1);
}
