#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <map>
#include <atomic>
#include <vector>
#include <chrono>
#include <csignal>
#include <memory>
#include <json/json.h>
#include <microhttpd.h>
#include "VimbaCPP/Include/VimbaCPP.h"

using namespace AVT::VmbAPI;

#define DEFAULT_SERVER_HOST "0.0.0.0"
#define DEFAULT_SERVER_PORT 8080

static const char* ENV_CAMERA_IP = "CAMERA_IP";
static const char* ENV_SERVER_HOST = "HTTP_SERVER_HOST";
static const char* ENV_SERVER_PORT = "HTTP_SERVER_PORT";

static std::string get_env(const char* key, const char* fallback = nullptr) {
    const char* val = std::getenv(key);
    if (val) return std::string(val);
    if (fallback) return std::string(fallback);
    return "";
}

struct CameraSettings {
    double exposure = 0;
    double gain = 0;
    double white_balance = 0;
    int roi_x = 0, roi_y = 0, roi_w = 0, roi_h = 0;
};

enum class AcquisitionState { Stopped, Started };

struct CameraContext {
    std::string camera_ip;
    std::string camera_id;
    std::shared_ptr<Camera> camera;
    VimbaSystem& system;
    std::mutex mutex;
    CameraSettings settings;
    std::atomic<AcquisitionState> acq_state;
    std::atomic<bool> run_stream;
    std::queue<std::vector<uint8_t>> frame_queue;
    std::condition_variable frame_cv;
    size_t max_frame_queue = 8;
    CameraContext(VimbaSystem& sys) : system(sys), acq_state(AcquisitionState::Stopped), run_stream(false) {}
};

static int send_json_response(struct MHD_Connection* con, int code, const std::string& result) {
    struct MHD_Response* resp = MHD_create_response_from_buffer(result.size(), (void*)result.data(), MHD_RESPMEM_MUST_COPY);
    MHD_add_response_header(resp, "Content-Type", "application/json");
    int ret = MHD_queue_response(con, code, resp);
    MHD_destroy_response(resp);
    return ret;
}

static int send_text_response(struct MHD_Connection* con, int code, const std::string& result, const char* mimetype) {
    struct MHD_Response* resp = MHD_create_response_from_buffer(result.size(), (void*)result.data(), MHD_RESPMEM_MUST_COPY);
    MHD_add_response_header(resp, "Content-Type", mimetype);
    int ret = MHD_queue_response(con, code, resp);
    MHD_destroy_response(resp);
    return ret;
}

static int send_stream_header(struct MHD_Connection* con) {
    const char* ct = "multipart/x-mixed-replace; boundary=frame";
    struct MHD_Response* resp = MHD_create_response_from_buffer(0, (void*)"", MHD_RESPMEM_PERSISTENT);
    MHD_add_response_header(resp, "Content-Type", ct);
    int ret = MHD_queue_response(con, MHD_HTTP_OK, resp);
    MHD_destroy_response(resp);
    return ret;
}

static std::string make_status_json(CameraContext& ctx) {
    Json::Value root;
    root["device"] = "Mako ultra-compact AIA GigE Vision camera";
    root["ip"] = ctx.camera_ip;
    try {
        FeaturePtr f;
        ctx.camera->GetFeatureByName("DeviceTemperature", f);
        double temp = 0;
        if (f) f->GetValue(temp);
        root["temperature"] = temp;
    } catch (...) {}
    root["acquisition_state"] = (ctx.acq_state == AcquisitionState::Started) ? "started" : "stopped";
    std::ostringstream ss;
    ss << root;
    return ss.str();
}

class FrameObserver : public IFrameObserver {
    CameraContext& ctx;
public:
    FrameObserver(CameraPtr cam, CameraContext& c) : IFrameObserver(cam), ctx(c) {}
    void FrameReceived(const FramePtr pFrame) override {
        if (pFrame->GetReceiveStatus() == VmbFrameStatusComplete) {
            VmbUint32_t size = 0;
            pFrame->GetImageSize(size);
            const VmbUchar_t* data = nullptr;
            pFrame->GetImage(data);
            if (data && size) {
                std::vector<uint8_t> img(data, data + size);
                std::lock_guard<std::mutex> lk(ctx.mutex);
                if (ctx.frame_queue.size() < ctx.max_frame_queue) {
                    ctx.frame_queue.push(std::move(img));
                    ctx.frame_cv.notify_all();
                }
            }
        }
    }
};

static void acquisition_thread(CameraContext* ctx) {
    while (ctx->run_stream) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

static int handle_config(struct MHD_Connection* con, const char* upload_data, size_t* upload_data_size, CameraContext* ctx) {
    static std::string body;
    if (*upload_data_size) {
        body.append(upload_data, *upload_data_size);
        *upload_data_size = 0;
        return MHD_YES;
    }
    // Parse JSON and update settings
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream iss(body);
    if (!Json::parseFromStream(builder, iss, &root, &errs)) {
        body.clear();
        return send_json_response(con, 400, "{\"error\":\"Invalid JSON\"}");
    }
    std::lock_guard<std::mutex> lk(ctx->mutex);
    if (root.isMember("exposure")) {
        double val = root["exposure"].asDouble();
        FeaturePtr f;
        ctx->camera->GetFeatureByName("ExposureTime", f);
        if (f) f->SetValue(val);
        ctx->settings.exposure = val;
    }
    if (root.isMember("gain")) {
        double val = root["gain"].asDouble();
        FeaturePtr f;
        ctx->camera->GetFeatureByName("Gain", f);
        if (f) f->SetValue(val);
        ctx->settings.gain = val;
    }
    if (root.isMember("white_balance")) {
        double val = root["white_balance"].asDouble();
        FeaturePtr f;
        ctx->camera->GetFeatureByName("BalanceRatio", f); // Example
        if (f) f->SetValue(val);
        ctx->settings.white_balance = val;
    }
    if (root.isMember("roi")) {
        auto roi = root["roi"];
        if (roi.isMember("x") && roi.isMember("y") && roi.isMember("width") && roi.isMember("height")) {
            int x=roi["x"].asInt(), y=roi["y"].asInt(), w=roi["width"].asInt(), h=roi["height"].asInt();
            FeaturePtr f;
            ctx->camera->GetFeatureByName("OffsetX", f);
            if (f) f->SetValue(x);
            ctx->camera->GetFeatureByName("OffsetY", f);
            if (f) f->SetValue(y);
            ctx->camera->GetFeatureByName("Width", f);
            if (f) f->SetValue(w);
            ctx->camera->GetFeatureByName("Height", f);
            if (f) f->SetValue(h);
            ctx->settings.roi_x = x; ctx->settings.roi_y = y; ctx->settings.roi_w = w; ctx->settings.roi_h = h;
        }
    }
    body.clear();
    return send_json_response(con, 200, "{\"result\":\"ok\"}");
}

static int handle_acquire(struct MHD_Connection* con, const char* upload_data, size_t* upload_data_size, CameraContext* ctx) {
    static std::string body;
    if (*upload_data_size) {
        body.append(upload_data, *upload_data_size);
        *upload_data_size = 0;
        return MHD_YES;
    }
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    std::istringstream iss(body);
    if (!Json::parseFromStream(builder, iss, &root, &errs)) {
        body.clear();
        return send_json_response(con, 400, "{\"error\":\"Invalid JSON\"}");
    }
    std::string cmd = root.get("command", "").asString();
    if (cmd == "start") {
        if (ctx->acq_state == AcquisitionState::Started) {
            body.clear();
            return send_json_response(con, 409, "{\"error\":\"Already started\"}");
        }
        ctx->acq_state = AcquisitionState::Started;
        ctx->run_stream = true;
        ctx->frame_queue = std::queue<std::vector<uint8_t>>();
        ctx->camera->StartContinuousImageAcquisition(ctx->max_frame_queue, IFrameObserverPtr(new FrameObserver(ctx->camera, *ctx)));
    } else if (cmd == "stop") {
        if (ctx->acq_state == AcquisitionState::Stopped) {
            body.clear();
            return send_json_response(con, 409, "{\"error\":\"Already stopped\"}");
        }
        ctx->acq_state = AcquisitionState::Stopped;
        ctx->run_stream = false;
        ctx->camera->StopContinuousImageAcquisition();
        ctx->frame_cv.notify_all();
    } else {
        body.clear();
        return send_json_response(con, 400, "{\"error\":\"Unknown command\"}");
    }
    body.clear();
    return send_json_response(con, 200, "{\"result\":\"ok\"}");
}

static int handle_capture(struct MHD_Connection* con, CameraContext* ctx) {
    std::vector<uint8_t> img;
    {
        std::unique_lock<std::mutex> lk(ctx->mutex);
        if (ctx->acq_state != AcquisitionState::Started) {
            return send_json_response(con, 409, "{\"error\":\"Acquisition not started\"}");
        }
        if (ctx->frame_queue.empty()) {
            ctx->frame_cv.wait_for(lk, std::chrono::milliseconds(1000));
        }
        if (!ctx->frame_queue.empty()) {
            img = std::move(ctx->frame_queue.front());
            ctx->frame_queue.pop();
        }
    }
    if (img.empty()) {
        return send_json_response(con, 500, "{\"error\":\"No frame available\"}");
    }
    // Assume image is in Mono8; wrap as PGM for browser/CLI compatibility
    std::ostringstream oss;
    int w = ctx->settings.roi_w ? ctx->settings.roi_w : 640;
    int h = ctx->settings.roi_h ? ctx->settings.roi_h : 480;
    oss << "P5\n" << w << " " << h << "\n255\n";
    oss.write((const char*)img.data(), img.size());
    return send_text_response(con, 200, oss.str(), "image/x-portable-graymap");
}

static int handle_stream(struct MHD_Connection* con, CameraContext* ctx) {
    // HTTP multipart/x-mixed-replace streaming
    struct MHD_Response* resp = MHD_create_response_from_callback(
        MHD_SIZE_UNKNOWN,
        4096,
        [](void* cls, uint64_t pos, char* buf, size_t max)->ssize_t {
            CameraContext* ctx = (CameraContext*)cls;
            std::vector<uint8_t> img;
            {
                std::unique_lock<std::mutex> lk(ctx->mutex);
                if (ctx->acq_state != AcquisitionState::Started) return MHD_CONTENT_READER_END_OF_STREAM;
                if (ctx->frame_queue.empty())
                    ctx->frame_cv.wait_for(lk, std::chrono::milliseconds(500));
                if (!ctx->frame_queue.empty()) {
                    img = std::move(ctx->frame_queue.front());
                    ctx->frame_queue.pop();
                }
            }
            if (img.empty()) return MHD_CONTENT_READER_END_OF_STREAM;
            int w = ctx->settings.roi_w ? ctx->settings.roi_w : 640, h = ctx->settings.roi_h ? ctx->settings.roi_h : 480;
            std::ostringstream oss;
            oss << "--frame\r\nContent-Type: image/x-portable-graymap\r\n\r\nP5\n" << w << " " << h << "\n255\n";
            oss.write((const char*)img.data(), img.size());
            oss << "\r\n";
            std::string s = oss.str();
            if (s.size() > max) s.resize(max);
            memcpy(buf, s.data(), s.size());
            return s.size();
        },
        ctx,
        nullptr
    );
    MHD_add_response_header(resp, "Content-Type", "multipart/x-mixed-replace; boundary=frame");
    int ret = MHD_queue_response(con, MHD_HTTP_OK, resp);
    MHD_destroy_response(resp);
    return ret;
}

static int answer_to_connection(void* cls, struct MHD_Connection* con, const char* url, const char* method,
                               const char* version, const char* upload_data, size_t* upload_data_size, void** con_cls) {
    CameraContext* ctx = (CameraContext*)cls;
    if (strcmp(url, "/config") == 0 && strcmp(method, "PUT") == 0) {
        return handle_config(con, upload_data, upload_data_size, ctx);
    }
    if (strcmp(url, "/acquire") == 0 && strcmp(method, "POST") == 0) {
        return handle_acquire(con, upload_data, upload_data_size, ctx);
    }
    if (strcmp(url, "/capture") == 0 && strcmp(method, "POST") == 0) {
        return handle_capture(con, ctx);
    }
    if (strcmp(url, "/status") == 0 && strcmp(method, "GET") == 0) {
        return send_json_response(con, 200, make_status_json(*ctx));
    }
    if (strcmp(url, "/stream") == 0 && strcmp(method, "GET") == 0) {
        return handle_stream(con, ctx);
    }
    return send_json_response(con, 404, "{\"error\":\"Not found\"}");
}

static std::string find_camera_by_ip(VimbaSystem& sys, const std::string& ip) {
    CameraPtrVector cameras;
    sys.GetCameras(cameras);
    for (auto& c : cameras) {
        FeaturePtr f;
        c->GetFeatureByName("GevCurrentIPAddress", f);
        VmbInt64_t addr = 0;
        if (f) f->GetValue(addr);
        unsigned char b[4];
        b[0] = (addr >> 24) & 0xFF;
        b[1] = (addr >> 16) & 0xFF;
        b[2] = (addr >> 8) & 0xFF;
        b[3] = (addr) & 0xFF;
        std::ostringstream oss;
        oss << (int)b[0] << "." << (int)b[1] << "." << (int)b[2] << "." << (int)b[3];
        if (oss.str() == ip) return c->GetID().c_str();
    }
    return "";
}

int main() {
    std::string camera_ip = get_env(ENV_CAMERA_IP, "");
    std::string server_host = get_env(ENV_SERVER_HOST, DEFAULT_SERVER_HOST);
    int server_port = std::stoi(get_env(ENV_SERVER_PORT, std::to_string(DEFAULT_SERVER_PORT).c_str()));

    VimbaSystem& sys = VimbaSystem::GetInstance();
    sys.Startup();

    CameraContext ctx(sys);
    ctx.camera_ip = camera_ip;
    ctx.max_frame_queue = 8;
    std::string camid = find_camera_by_ip(sys, camera_ip);
    if (camid.empty()) {
        std::cerr << "Camera with IP " << camera_ip << " not found." << std::endl;
        return 1;
    }
    ctx.camera_id = camid;
    CameraPtr camera;
    sys.OpenCameraByID(camid.c_str(), VmbAccessModeFull, camera);
    ctx.camera = camera;

    struct MHD_Daemon* daemon = MHD_start_daemon(
        MHD_USE_SELECT_INTERNALLY,
        server_port,
        NULL, NULL,
        &answer_to_connection, &ctx,
        MHD_OPTION_END
    );
    if (!daemon) {
        std::cerr << "Failed to start HTTP server." << std::endl;
        return 1;
    }

    std::cout << "HTTP server started at " << server_host << ":" << server_port << std::endl;
    std::cout << "Available endpoints: /config [PUT], /acquire [POST], /capture [POST], /status [GET], /stream [GET]" << std::endl;
    while (true) std::this_thread::sleep_for(std::chrono::seconds(1));

    MHD_stop_daemon(daemon);
    sys.Shutdown();
    return 0;
}