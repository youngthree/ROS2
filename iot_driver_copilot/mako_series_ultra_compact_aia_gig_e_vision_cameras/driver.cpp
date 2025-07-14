```cpp
#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <sstream>
#include <fstream>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <condition_variable>
#include <json/json.h>
#include "civetweb.h"

// -------------- Device SDK simulation (replace with real Vimba API or GenICam) --------------
class FakeCamera {
public:
    struct Frame {
        std::vector<unsigned char> data;
        std::string format;
        std::chrono::system_clock::time_point timestamp;
    };
    FakeCamera(const std::string& ip, int port) :
        device_ip(ip), device_port(port), exposure(10000), gain(1.0), framerate(10.0), pixel_format("Mono8"),
        roi_x(0), roi_y(0), roi_w(640), roi_h(480), temperature(34.5), buffer_status("OK"), triggered(false)
    {
        device_name = "Mako series ultra-compact AIA GigE Vision cameras";
        device_model = "G-032";
        manufacturer = "Allied Vision";
        device_type = "Machine vision camera";
        last_frame_time = std::chrono::system_clock::now();
    }

    std::string get_info_json() {
        Json::Value info;
        info["device_name"] = device_name;
        info["device_model"] = device_model;
        info["manufacturer"] = manufacturer;
        info["device_type"] = device_type;
        Json::StreamWriterBuilder wbuilder;
        return Json::writeString(wbuilder, info);
    }

    std::string get_status_json(const std::vector<std::string>& fields) {
        Json::Value status;
        if (fields.empty() || std::find(fields.begin(), fields.end(), "exposure") != fields.end())
            status["exposure"] = exposure;
        if (fields.empty() || std::find(fields.begin(), fields.end(), "gain") != fields.end())
            status["gain"] = gain;
        if (fields.empty() || std::find(fields.begin(), fields.end(), "roi") != fields.end()) {
            status["roi"]["x"] = roi_x;
            status["roi"]["y"] = roi_y;
            status["roi"]["width"] = roi_w;
            status["roi"]["height"] = roi_h;
        }
        if (fields.empty() || std::find(fields.begin(), fields.end(), "framerate") != fields.end())
            status["framerate"] = framerate;
        if (fields.empty() || std::find(fields.begin(), fields.end(), "pixel_format") != fields.end())
            status["pixel_format"] = pixel_format;
        if (fields.empty() || std::find(fields.begin(), fields.end(), "temperature") != fields.end())
            status["temperature"] = temperature;
        if (fields.empty() || std::find(fields.begin(), fields.end(), "buffer_status") != fields.end())
            status["buffer_status"] = buffer_status;
        Json::StreamWriterBuilder wbuilder;
        return Json::writeString(wbuilder, status);
    }

    bool set_params(const Json::Value& params, std::string& err) {
        if (params.isMember("exposure")) {
            int v = params["exposure"].asInt();
            if (v < 1 || v > 1000000) { err = "Exposure out of range"; return false; }
            exposure = v;
        }
        if (params.isMember("gain")) {
            double v = params["gain"].asDouble();
            if (v < 0.0 || v > 20.0) { err = "Gain out of range"; return false; }
            gain = v;
        }
        if (params.isMember("roi")) {
            const auto& r = params["roi"];
            if (!(r.isMember("x") && r.isMember("y") && r.isMember("width") && r.isMember("height"))) {
                err = "ROI incomplete"; return false;
            }
            int x = r["x"].asInt(), y = r["y"].asInt(), w = r["width"].asInt(), h = r["height"].asInt();
            if (x < 0 || y < 0 || w < 1 || h < 1) { err = "ROI invalid"; return false; }
            roi_x = x; roi_y = y; roi_w = w; roi_h = h;
        }
        if (params.isMember("framerate")) {
            double v = params["framerate"].asDouble();
            if (v < 1.0 || v > 120.0) { err = "Framerate out of range"; return false; }
            framerate = v;
        }
        if (params.isMember("pixel_format")) {
            pixel_format = params["pixel_format"].asString();
        }
        // (Other params omitted for brevity)
        return true;
    }

    void reset() {
        exposure = 10000;
        gain = 1.0;
        framerate = 10.0;
        pixel_format = "Mono8";
        roi_x = 0; roi_y = 0; roi_w = 640; roi_h = 480;
        buffer_status = "OK";
        temperature = 34.5;
    }

    bool trigger(const Json::Value& trig, std::string& err) {
        // For simulation, just mark triggered and generate a frame
        triggered = true;
        return true;
    }

    // Simulate image acquisition
    Frame get_frame(const std::string& format = "", int frame_idx = 0) {
        std::lock_guard<std::mutex> lk(frame_mtx);
        std::string fmt = format.empty() ? pixel_format : format;
        // Simulate a grayscale image
        int w = roi_w, h = roi_h;
        std::vector<unsigned char> data(w * h, 128); // Gray
        last_frame.format = fmt;
        last_frame.data = std::move(data);
        last_frame.timestamp = std::chrono::system_clock::now();
        return last_frame;
    }

private:
    std::string device_ip;
    int device_port;

    std::string device_name, device_model, manufacturer, device_type;

    int exposure;
    double gain;
    double framerate;
    std::string pixel_format;
    int roi_x, roi_y, roi_w, roi_h;
    double temperature;
    std::string buffer_status;
    bool triggered;

    Frame last_frame;
    std::chrono::system_clock::time_point last_frame_time;
    std::mutex frame_mtx;
};
// -------------- End Device SDK simulation --------------

// -------------- HTTP Handler Implementation --------------
struct AppContext {
    FakeCamera camera;
    std::mutex cam_mtx;
    AppContext(const std::string& ip, int port) : camera(ip, port) {}
};

static int handle_get_info(struct mg_connection *conn, void *cbdata)
{
    AppContext* ctx = static_cast<AppContext*>(cbdata);
    std::lock_guard<std::mutex> lk(ctx->cam_mtx);
    std::string info = ctx->camera.get_info_json();
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\nContent-Length: %zu\r\n\r\n%s",
                    info.size(), info.c_str());
    return 200;
}

static int handle_get_status(struct mg_connection *conn, void *cbdata)
{
    AppContext* ctx = static_cast<AppContext*>(cbdata);
    char query[1024];
    mg_get_query_string(conn, query, sizeof(query));
    std::vector<std::string> fields;
    if (strstr(query, "fields=") != nullptr) {
        std::string q(query);
        auto fpos = q.find("fields=");
        if (fpos != std::string::npos) {
            std::string fs = q.substr(fpos + 7);
            std::istringstream ss(fs);
            std::string tok;
            while (std::getline(ss, tok, ','))
                fields.push_back(tok);
        }
    }
    std::lock_guard<std::mutex> lk(ctx->cam_mtx);
    std::string status = ctx->camera.get_status_json(fields);
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\nContent-Length: %zu\r\n\r\n%s",
                    status.size(), status.c_str());
    return 200;
}

static int handle_post_params(struct mg_connection *conn, void *cbdata)
{
    AppContext* ctx = static_cast<AppContext*>(cbdata);
    char buffer[8192];
    int n = mg_read(conn, buffer, sizeof(buffer)-1);
    buffer[n] = '\0';
    Json::CharReaderBuilder rbuilder;
    Json::Value root;
    std::string errs;
    std::unique_ptr<Json::CharReader> const reader(rbuilder.newCharReader());
    if (!reader->parse(buffer, buffer+n, &root, &errs)) {
        mg_printf(conn, "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\n"
                        "Access-Control-Allow-Origin: *\r\n\r\nInvalid JSON");
        return 400;
    }
    std::string err;
    {
        std::lock_guard<std::mutex> lk(ctx->cam_mtx);
        if (!ctx->camera.set_params(root, err)) {
            mg_printf(conn, "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\n"
                            "Access-Control-Allow-Origin: *\r\n\r\n%s", err.c_str());
            return 400;
        }
    }
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\n\r\n{\"result\": \"ok\"}");
    return 200;
}

static int handle_post_trig(struct mg_connection *conn, void *cbdata)
{
    AppContext* ctx = static_cast<AppContext*>(cbdata);
    char buffer[4096];
    int n = mg_read(conn, buffer, sizeof(buffer)-1);
    buffer[n] = '\0';
    Json::CharReaderBuilder rbuilder;
    Json::Value root;
    std::string errs;
    std::unique_ptr<Json::CharReader> const reader(rbuilder.newCharReader());
    if (!reader->parse(buffer, buffer+n, &root, &errs)) {
        mg_printf(conn, "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\n"
                        "Access-Control-Allow-Origin: *\r\n\r\nInvalid JSON");
        return 400;
    }
    std::string err;
    {
        std::lock_guard<std::mutex> lk(ctx->cam_mtx);
        if (!ctx->camera.trigger(root, err)) {
            mg_printf(conn, "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\n"
                            "Access-Control-Allow-Origin: *\r\n\r\n%s", err.c_str());
            return 500;
        }
    }
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\n\r\n{\"result\": \"ok\"}");
    return 200;
}

static int handle_post_reset(struct mg_connection *conn, void *cbdata)
{
    AppContext* ctx = static_cast<AppContext*>(cbdata);
    {
        std::lock_guard<std::mutex> lk(ctx->cam_mtx);
        ctx->camera.reset();
    }
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                    "Access-Control-Allow-Origin: *\r\n\r\n{\"result\": \"ok\"}");
    return 200;
}

static int handle_get_frame(struct mg_connection *conn, void *cbdata)
{
    AppContext* ctx = static_cast<AppContext*>(cbdata);
    char query[1024];
    mg_get_query_string(conn, query, sizeof(query));
    std::string format = "Mono8";
    int idx = 0;
    if (strstr(query, "format=") != nullptr) {
        std::string q(query);
        auto fpos = q.find("format=");
        if (fpos != std::string::npos) {
            std::string fs = q.substr(fpos + 7);
            auto endpos = fs.find('&');
            if (endpos != std::string::npos)
                fs = fs.substr(0, endpos);
            format = fs;
        }
    }
    if (strstr(query, "idx=") != nullptr) {
        std::string q(query);
        auto ipos = q.find("idx=");
        if (ipos != std::string::npos) {
            std::string is = q.substr(ipos + 4);
            auto endpos = is.find('&');
            if (endpos != std::string::npos)
                is = is.substr(0, endpos);
            idx = std::stoi(is);
        }
    }
    FakeCamera::Frame frame;
    {
        std::lock_guard<std::mutex> lk(ctx->cam_mtx);
        frame = ctx->camera.get_frame(format, idx);
    }
    // Send as image/octet-stream, in real code convert to PNG/JPEG if browser needs it
    mg_printf(conn, "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Content-Disposition: attachment; filename=\"frame_%lld.raw\"\r\n"
                    "Content-Length: %zu\r\n\r\n",
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        frame.timestamp.time_since_epoch()).count(),
                    frame.data.size());
    mg_write(conn, frame.data.data(), frame.data.size());
    return 200;
}

// Routing table
struct Route {
    std::string method;
    std::string path;
    int (*handler)(struct mg_connection *, void *);
};

static const Route routes[] = {
    {"GET",  "/cam/info",   handle_get_info},
    {"POST", "/cam/trig",   handle_post_trig},
    {"GET",  "/cam/status", handle_get_status},
    {"POST", "/cam/reset",  handle_post_reset},
    {"GET",  "/cam/frame",  handle_get_frame},
    {"POST", "/cam/params", handle_post_params}
};

static int dispatch_handler(struct mg_connection *conn, void *cbdata)
{
    const struct mg_request_info *req_info = mg_get_request_info(conn);
    AppContext* app = static_cast<AppContext*>(cbdata);

    for (const auto& r : routes) {
        if (req_info->request_method && req_info->local_uri &&
            r.method == req_info->request_method &&
            r.path == req_info->local_uri) {
            return r.handler(conn, cbdata);
        }
    }
    mg_printf(conn, "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n"
                    "Access-Control-Allow-Origin: *\r\n\r\nNot Found");
    return 404;
}

// -------------- Main & Environment --------------
int main(int argc, char* argv[])
{
    // Environment
    const char* device_ip = std::getenv("DEVICE_IP");
    const char* device_port_env = std::getenv("DEVICE_PORT");
    const char* server_host = std::getenv("SERVER_HOST");
    const char* server_port = std::getenv("SERVER_PORT");

    if (!device_ip || !device_port_env || !server_host || !server_port) {
        std::cerr << "Missing environment variables: DEVICE_IP, DEVICE_PORT, SERVER_HOST, SERVER_PORT" << std::endl;
        return 1;
    }
    int device_port = atoi(device_port_env);

    // Civetweb options
    std::string port_opt = std::string(server_host) + ":" + std::string(server_port);
    const char *options[] = {
        "listening_ports", port_opt.c_str(),
        0
    };

    AppContext app_ctx(device_ip, device_port);

    struct mg_callbacks callbacks;
    memset(&callbacks, 0, sizeof(callbacks));
    struct mg_context *ctx = mg_start(&callbacks, &app_ctx, options);

    for (const auto& r : routes) {
        mg_set_request_handler(ctx, r.path.c_str(), dispatch_handler, &app_ctx);
    }

    std::cout << "HTTP server running on http://" << server_host << ":" << server_port << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;
    // Run indefinitely
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(60));
    }

    mg_stop(ctx);
    return 0;
}
```
