#include <hailo/hailort.h>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>

static std::atomic<bool> g_running(true);
static void sig_handler(int) { g_running = false; }

struct Detection { float x0, y0, x1, y1, score; };

static float sigmoid_f(float x) { return 1.0f / (1.0f + std::exp(-x)); }
static float clamp_f(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float compute_iou(const Detection &a, const Detection &b) {
    float ix = std::max(a.x0, b.x0), iy = std::max(a.y0, b.y0);
    float ax = std::min(a.x1, b.x1), ay = std::min(a.y1, b.y1);
    float inter = std::max(0.f, ax - ix) * std::max(0.f, ay - iy);
    float u = (a.x1 - a.x0) * (a.y1 - a.y0) + (b.x1 - b.x0) * (b.y1 - b.y0) - inter;
    return u > 0 ? inter / u : 0;
}

static std::vector<Detection> do_nms(std::vector<Detection> &d, float thr) {
    std::sort(d.begin(), d.end(), [](const Detection &a, const Detection &b) { return a.score > b.score; });
    std::vector<bool> sup(d.size(), false);
    std::vector<Detection> r;
    for (size_t i = 0; i < d.size(); i++) {
        if (sup[i]) continue;
        r.push_back(d[i]);
        for (size_t j = i + 1; j < d.size(); j++)
            if (!sup[j] && compute_iou(d[i], d[j]) > thr) sup[j] = true;
    }
    return r;
}

static void decode_scale(const float *box, const float *score,
                          int gh, int gw, float thr,
                          float ih, float iw, std::vector<Detection> &out) {
    for (int gy = 0; gy < gh; gy++)
        for (int gx = 0; gx < gw; gx++) {
            float sc = sigmoid_f(score[gy * gw + gx]);
            if (sc < thr) continue;
            int b = (gy * gw + gx) * 4;
            float cx = (sigmoid_f(box[b]) + gx) / gw * iw;
            float cy = (sigmoid_f(box[b + 1]) + gy) / gh * ih;
            float bw = std::exp(box[b + 2]) * iw / gw;
            float bh = std::exp(box[b + 3]) * ih / gh;
            Detection d;
            d.x0 = clamp_f(cx - bw * 0.5f, 0.f, iw);
            d.y0 = clamp_f(cy - bh * 0.5f, 0.f, ih);
            d.x1 = clamp_f(cx + bw * 0.5f, 0.f, iw);
            d.y1 = clamp_f(cy + bh * 0.5f, 0.f, ih);
            d.score = sc;
            out.push_back(d);
        }
}

class HailoDetector {
public:
    ~HailoDetector() {
        for (auto h : out_streams_) if (h) hailo_release_output_vstreams(&h, 1);
        if (in_stream_) hailo_release_input_vstreams(&in_stream_, 1);
        if (hef_) hailo_release_hef(hef_);
        if (vdev_) hailo_release_vdevice(vdev_);
    }

    bool init(const std::string &hef_path) {
        hailo_status s;
        hailo_vdevice_params_t vp;
        hailo_init_vdevice_params(&vp);
        s = hailo_create_vdevice(&vp, &vdev_);
        if (s != HAILO_SUCCESS) { std::cerr << "vdevice: " << s << "\n"; return false; }

        s = hailo_create_hef_file(&hef_, hef_path.c_str());
        if (s != HAILO_SUCCESS) { std::cerr << "hef: " << s << "\n"; return false; }

        size_t ng_count = 1;
        s = hailo_configure_vdevice(vdev_, hef_, nullptr, &ng_, &ng_count);
        if (s != HAILO_SUCCESS) { std::cerr << "configure: " << s << "\n"; return false; }

        size_t in_count = 16;
        hailo_input_vstream_params_by_name_t in_p[16];
        memset(in_p, 0, sizeof(in_p));
        s = hailo_make_input_vstream_params(ng_, false, HAILO_FORMAT_TYPE_UINT8, in_p, &in_count);
        if (s != HAILO_SUCCESS) { std::cerr << "make_in: " << s << "\n"; return false; }

        size_t out_count = 16;
        hailo_output_vstream_params_by_name_t out_p[16];
        memset(out_p, 0, sizeof(out_p));
        s = hailo_make_output_vstream_params(ng_, false, HAILO_FORMAT_TYPE_FLOAT32, out_p, &out_count);
        if (s != HAILO_SUCCESS) { std::cerr << "make_out: " << s << "\n"; return false; }
        out_count_ = out_count;

        s = hailo_create_input_vstreams(ng_, in_p, in_count, &in_stream_);
        if (s != HAILO_SUCCESS) { std::cerr << "create_in: " << s << "\n"; return false; }

        out_streams_.resize(out_count, nullptr);
        s = hailo_create_output_vstreams(ng_, out_p, out_count, out_streams_.data());
        if (s != HAILO_SUCCESS) { std::cerr << "create_out: " << s << "\n"; return false; }

        size_t in_fs = 0;
        s = hailo_get_input_vstream_frame_size(in_stream_, &in_fs);
        if (s != HAILO_SUCCESS) { std::cerr << "in_size: " << s << "\n"; return false; }
        in_frame_size_ = in_fs;

        out_frame_sizes_.resize(out_count);
        for (size_t i = 0; i < out_count; i++) {
            size_t fs = 0;
            s = hailo_get_output_vstream_frame_size(out_streams_[i], &fs);
            if (s != HAILO_SUCCESS) { std::cerr << "out_size_" << i << ": " << s << "\n"; return false; }
            out_frame_sizes_[i] = fs;
        }

        mw_ = 1280; mh_ = 960;
        std::cout << "Hailo ready. Input: " << mw_ << "x" << mh_ << " (" << in_frame_size_ << " bytes)\n";
        return true;
    }

    std::vector<Detection> detect(const uint8_t *rgb, int w, int h) {
        std::vector<uint8_t> in_buf(in_frame_size_);
        for (int y = 0; y < mh_; y++)
            for (int x = 0; x < mw_; x++) {
                int sx = x * w / mw_, sy = y * h / mh_;
                int si = (sy * w + sx) * 3, di = (y * mw_ + x) * 3;
                in_buf[di] = rgb[si];
                in_buf[di + 1] = rgb[si + 1];
                in_buf[di + 2] = rgb[si + 2];
            }

        auto ws = hailo_vstream_write_raw_buffer(in_stream_, in_buf.data(), in_buf.size());
        if (ws != HAILO_SUCCESS) return {};

        std::vector<std::vector<uint8_t>> out_bufs(out_count_);
        for (size_t i = 0; i < out_count_; i++) {
            out_bufs[i].resize(out_frame_sizes_[i]);
            auto rs = hailo_vstream_read_raw_buffer(out_streams_[i], out_bufs[i].data(), out_bufs[i].size());
            if (rs != HAILO_SUCCESS) return {};
        }

        std::vector<Detection> all;
        float ih = (float)mh_, iw = (float)mw_;

        struct ScaleDef { int gw, gh; };
        ScaleDef scales[] = { {160, 120}, {80, 60}, {40, 30} };

        for (size_t si = 0; si < 3 && si * 2 + 1 < out_count_; si++) {
            size_t box_floats = out_frame_sizes_[si * 2] / sizeof(float);
            if (box_floats != (size_t)scales[si].gw * scales[si].gh * 4) continue;

            const float *box_f = reinterpret_cast<const float *>(out_bufs[si * 2].data());
            const float *score_f = reinterpret_cast<const float *>(out_bufs[si * 2 + 1].data());

            decode_scale(box_f, score_f, scales[si].gh, scales[si].gw, 0.25f, ih, iw, all);
        }

        auto nms_result = do_nms(all, 0.45f);

        float sx = (float)w / (float)mw_;
        float sy = (float)h / (float)mh_;
        for (auto &d : nms_result) {
            d.x0 *= sx; d.y0 *= sy;
            d.x1 *= sx; d.y1 *= sy;
        }
        return nms_result;
    }

private:
    hailo_vdevice vdev_ = nullptr;
    hailo_hef hef_ = nullptr;
    hailo_configured_network_group ng_ = nullptr;
    hailo_input_vstream in_stream_ = nullptr;
    std::vector<hailo_output_vstream> out_streams_;
    size_t out_count_ = 0;
    size_t in_frame_size_ = 0;
    std::vector<size_t> out_frame_sizes_;
    int mw_ = 1280, mh_ = 960;
};

static const char *HTML_PAGE = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Ball Detector</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:monospace;background:#0a0a0f;color:#e0e0e0;display:flex;height:100vh}
.cam{flex:2;display:flex;align-items:center;justify-content:center;position:relative;background:#000}
.cam img{max-width:100%;max-height:100%}
.cam canvas{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.side{flex:1;max-width:320px;padding:12px;display:flex;flex-direction:column;gap:8px}
.card{background:#16213e;border:1px solid #0f3460;border-radius:6px;padding:12px}
.card h3{color:#53a8b6;font-size:12px;text-transform:uppercase;margin-bottom:6px}
.row{display:flex;justify-content:space-between;padding:3px 0;font-size:13px}
.lbl{color:#888}.val{color:#e94560;font-weight:bold}
.val.ok{color:#4ecca3}
#fps{position:fixed;top:8px;right:8px;background:rgba(0,0,0,.7);padding:4px 10px;border-radius:12px;font-size:13px;color:#4ecca3;border:1px solid #333}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:4px}
.dot.on{background:#4ecca3;box-shadow:0 0 6px #4ecca3}.dot.off{background:#e94560}
</style></head><body>
<div class="cam">
  <img id="stream" src="/stream">
  <canvas id="ov"></canvas>
</div>
<div class="side">
  <div class="card"><h3><span class="dot on" id="cs"></span>Camera</h3>
    <div class="row"><span class="lbl">FPS</span><span class="val" id="fpsv">--</span></div>
    <div class="row"><span class="lbl">Inference</span><span class="val" id="infv">--</span></div>
  </div>
  <div class="card"><h3>Detections</h3>
    <div class="row"><span class="lbl">Count</span><span class="val" id="dcnt">0</span></div>
    <div class="row"><span class="lbl">Best</span><span class="val" id="bscr">--</span></div>
    <div class="row"><span class="lbl">Center X</span><span class="val" id="bcx">--</span></div>
    <div class="row"><span class="lbl">Center Y</span><span class="val" id="bcy">--</span></div>
  </div>
  <div class="card"><h3>System</h3>
    <div class="row"><span class="lbl">Temp</span><span class="val" id="tmp">--</span></div>
  </div>
  <div id="detlist" style="font-size:11px;max-height:300px;overflow-y:auto"></div>
</div>
<div id="fps">-- FPS</div>
<script>
var c=document.getElementById('ov'),ctx=c.getContext('2d');
function rc(){c.width=c.clientWidth;c.height=c.clientHeight}
window.addEventListener('resize',rc);rc();
var img=document.getElementById('stream');
img.onerror=function(){document.getElementById('cs').className='dot off'};
img.onload=function(){document.getElementById('cs').className='dot on';rc()};
function poll(){
  fetch('/api/detections').then(r=>r.json()).then(d=>{
    document.getElementById('fps').textContent=d.fps.toFixed(1)+' FPS';
    document.getElementById('fpsv').textContent=d.fps.toFixed(1);
    document.getElementById('infv').textContent=d.infer_ms.toFixed(0)+'ms';
    document.getElementById('dcnt').textContent=d.dets.length;
    ctx.clearRect(0,0,c.width,c.height);
    var fw=d.fw,fh=d.fh;
    function mx(x){return x/fw*c.width}
    function my(y){return y/fh*c.height}
    if(d.best){
      document.getElementById('bscr').textContent=(d.best.score*100).toFixed(0)+'%';
      document.getElementById('bcx').textContent=d.best.cx.toFixed(1);
      document.getElementById('bcy').textContent=d.best.cy.toFixed(1);
    }else{
      document.getElementById('bscr').textContent='none';
      document.getElementById('bcx').textContent='--';
      document.getElementById('bcy').textContent='--';
    }
    var h='';
    d.dets.forEach(function(det,i){
      var color=det.score>0.7?'#4ecca3':det.score>0.4?'#f0a500':'#e94560';
      ctx.strokeStyle=color;ctx.lineWidth=2;
      ctx.strokeRect(mx(det.x0),my(det.y0),mx(det.x1)-mx(det.x0),my(det.y1)-my(det.y0));
      ctx.fillStyle=color;ctx.font='11px monospace';
      ctx.fillText((det.score*100).toFixed(0)+'%',mx(det.x0)+2,my(det.y0)-4);
      var bcx=(det.x0+det.x1)/2,bcy=(det.y0+det.y1)/2;
      ctx.beginPath();ctx.arc(mx(bcx),my(bcy),4,0,Math.PI*2);ctx.fill();
      h+='<div style="color:'+color+';padding:2px 0">'+i+': '+(det.score*100).toFixed(0)+'% ('+bcx.toFixed(0)+','+bcy.toFixed(0)+')</div>';
    });
    document.getElementById('detlist').innerHTML=h;
    fetch('/api/system').then(r=>r.json()).then(s=>{
      document.getElementById('tmp').textContent=s.temp?s.temp+'C':'--';
    }).catch(()=>{});
  }).catch(()=>{});
  setTimeout(poll,200);
}
poll();
</script></body></html>)rawliteral";

struct FrameData {
    std::vector<uint8_t> jpeg;
    std::vector<Detection> dets;
    int cam_w = 640, cam_h = 480;
    float fps = 0, infer_ms = 0;
    std::mutex mtx;
};

static FrameData g_frame;

static void *camera_thread(void *arg) {
    HailoDetector *det = (HailoDetector *)arg;
    FILE *pipe = popen("rpicam-vid --codec mjpeg -t 0 --width 640 --height 480 --framerate 30 --quality 50 -o - 2>/dev/null", "r");
    if (!pipe) { std::cerr << "Camera fail\n"; return nullptr; }

    std::vector<uint8_t> jpeg_buf;
    jpeg_buf.reserve(100000);
    int frames = 0;
    auto t0 = std::chrono::steady_clock::now();

    while (g_running) {
        auto t1 = std::chrono::steady_clock::now();
        jpeg_buf.clear();
        int c;
        bool found = false;
        while ((c = fgetc(pipe)) != EOF) {
            jpeg_buf.push_back(c);
            size_t n = jpeg_buf.size();
            if (n >= 2 && jpeg_buf[n-2] == 0xFF && jpeg_buf[n-1] == 0xD9) { found = true; break; }
            if (jpeg_buf.size() > 500000) { jpeg_buf.clear(); break; }
        }
        if (!found || jpeg_buf.empty()) { std::cerr << "Frame fail\n"; break; }

        cv::Mat jpeg_mat(1, (int)jpeg_buf.size(), CV_8UC1, jpeg_buf.data());
        cv::Mat bgr = cv::imdecode(jpeg_mat, cv::IMREAD_COLOR);
        if (bgr.empty()) continue;
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        auto t_infer = std::chrono::steady_clock::now();
        auto dets = det->detect(rgb.data, rgb.cols, rgb.rows);
        auto t_out = std::chrono::steady_clock::now();
        float infer_ms = std::chrono::duration<float, std::milli>(t_out - t_infer).count();

        for (auto &d : dets) {
            cv::rectangle(bgr, cv::Point((int)d.x0, (int)d.y0), cv::Point((int)d.x1, (int)d.y1), cv::Scalar(0, 255, 0), 2);
            char label[32];
            snprintf(label, sizeof(label), "%.0f%%", d.score * 100);
            cv::putText(bgr, label, cv::Point((int)d.x0, (int)d.y0 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            float cx = (d.x0 + d.x1) * 0.5f, cy = (d.y0 + d.y1) * 0.5f;
            cv::circle(bgr, cv::Point((int)cx, (int)cy), 5, cv::Scalar(0, 0, 255), -1);
        }

        std::vector<uchar> out_jpeg;
        cv::imencode(".jpg", bgr, out_jpeg, {cv::IMWRITE_JPEG_QUALITY, 60});

        frames++;
        float fps = frames / std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();

        {
            std::lock_guard<std::mutex> lk(g_frame.mtx);
            g_frame.jpeg = std::move(out_jpeg);
            g_frame.dets = std::move(dets);
            g_frame.cam_w = rgb.cols;
            g_frame.cam_h = rgb.rows;
            g_frame.fps = fps;
            g_frame.infer_ms = infer_ms;
        }
    }
    pclose(pipe);
    return nullptr;
}

static void handle_client(int client_fd) {
    char buf[4096];
    int n = recv(client_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) { close(client_fd); return; }
    buf[n] = '\0';

    bool is_api_dets = strstr(buf, "GET /api/detections") != nullptr;
    bool is_api_sys = strstr(buf, "GET /api/system") != nullptr;
    bool is_stream = strstr(buf, "GET /stream") != nullptr;
    bool is_index = !is_api_dets && !is_api_sys && !is_stream;

    if (is_index) {
        const char *hdr = "HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nConnection: close\r\n\r\n";
        send(client_fd, hdr, strlen(hdr), 0);
        send(client_fd, HTML_PAGE, strlen(HTML_PAGE), 0);
    } else if (is_api_dets) {
        const char *hdr = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";
        send(client_fd, hdr, strlen(hdr), 0);

        std::lock_guard<std::mutex> lk(g_frame.mtx);
        float best_score = 0; float best_cx = -1, best_cy = -1;
        for (auto &d : g_frame.dets) {
            if (d.score > best_score) {
                best_score = d.score;
                best_cx = (d.x0 + d.x1) * 0.5f;
                best_cy = (d.y0 + d.y1) * 0.5f;
            }
        }
        std::string json = "{\"fps\":" + std::to_string(g_frame.fps) +
            ",\"infer_ms\":" + std::to_string(g_frame.infer_ms) +
            ",\"fw\":" + std::to_string(g_frame.cam_w) +
            ",\"fh\":" + std::to_string(g_frame.cam_h) + ",\"dets\":[";
        for (size_t i = 0; i < g_frame.dets.size(); i++) {
            auto &d = g_frame.dets[i];
            if (i > 0) json += ",";
            json += "{\"x0\":" + std::to_string(d.x0) + ",\"y0\":" + std::to_string(d.y0) +
                    ",\"x1\":" + std::to_string(d.x1) + ",\"y1\":" + std::to_string(d.y1) +
                    ",\"score\":" + std::to_string(d.score) + "}";
        }
        json += "],\"best\":";
        if (best_score > 0) {
            json += "{\"cx\":" + std::to_string(best_cx) + ",\"cy\":" + std::to_string(best_cy) +
                    ",\"score\":" + std::to_string(best_score) + "}";
        } else {
            json += "null";
        }
        json += "}";
        send(client_fd, json.c_str(), json.size(), 0);
    } else if (is_api_sys) {
        const char *hdr = "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n";
        send(client_fd, hdr, strlen(hdr), 0);
        float temp = 0;
        FILE *tf = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
        if (tf) { int t; fscanf(tf, "%d", &t); temp = t / 1000.0f; fclose(tf); }
        std::string json = "{\"temp\":" + std::to_string(temp) + "}";
        send(client_fd, json.c_str(), json.size(), 0);
    } else if (is_stream) {
        const char *hdr = "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nConnection: close\r\n\r\n";
        send(client_fd, hdr, strlen(hdr), 0);

        while (g_running) {
            std::vector<uchar> jpg;
            {
                std::lock_guard<std::mutex> lk(g_frame.mtx);
                if (g_frame.jpeg.empty()) { usleep(10000); continue; }
                jpg = g_frame.jpeg;
            }
            char part[256];
            int plen = snprintf(part, sizeof(part), "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", jpg.size());
            if (send(client_fd, part, plen, 0) <= 0) break;
            if (send(client_fd, jpg.data(), jpg.size(), 0) <= 0) break;
            if (send(client_fd, "\r\n", 2, 0) <= 0) break;
            usleep(33000);
        }
    }
    close(client_fd);
}

static void *web_thread(void *) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(8080);

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "bind fail\n";
        return nullptr;
    }
    listen(server_fd, 10);
    std::cout << "Web dashboard: http://0.0.0.0:8080\n";

    while (g_running) {
        struct sockaddr_in client_addr;
        socklen_t clen = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &clen);
        if (client_fd < 0) continue;
        pthread_t tid;
        int *fd_ptr = new int(client_fd);
        pthread_create(&tid, nullptr, [](void *arg) -> void* {
            int fd = *(int*)arg;
            delete (int*)arg;
            handle_client(fd);
            return nullptr;
        }, fd_ptr);
        pthread_detach(tid);
    }
    close(server_fd);
    return nullptr;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    std::string hef = "/root/fb4-runner/10_hours_brainrot.hef";
    if (argc > 1) hef = argv[1];

    std::cout << "=== Ball Detector (C++) ===\n";
    HailoDetector det;
    if (!det.init(hef)) return 1;

    pthread_t cam_tid, web_tid;
    pthread_create(&web_tid, nullptr, web_thread, nullptr);
    pthread_create(&cam_tid, nullptr, camera_thread, &det);

    pthread_join(cam_tid, nullptr);
    g_running = false;
    pthread_join(web_tid, nullptr);

    return 0;
}
