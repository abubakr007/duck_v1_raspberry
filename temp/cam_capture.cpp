// Minimal libcamera still capture for IMX708 on Pi 5.
// Captures one frame to PPM after a few warm-up frames so AE/AWB stabilises.
//
// Build: g++ -std=c++17 -O2 cam_capture.cpp $(pkg-config --cflags --libs libcamera) -o cam_capture
// Run:   ./cam_capture <output.ppm>

#include <libcamera/libcamera.h>
#include <libcamera/formats.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <sys/mman.h>
#include <unistd.h>

using namespace libcamera;

namespace {
constexpr int kWarmupFrames = 6;  // discard first 5, save the 6th

std::shared_ptr<Camera> g_camera;
std::map<FrameBuffer*, void*> g_mapped;
std::map<FrameBuffer*, std::size_t> g_mapped_len;

std::mutex g_mutex;
std::condition_variable g_cv;
int g_completed = 0;
Request* g_final_request = nullptr;
}  // namespace

static void onRequestComplete(Request* request) {
    if (request->status() == Request::RequestCancelled) return;

    {
        std::lock_guard<std::mutex> lock(g_mutex);
        ++g_completed;
        std::cout << "Frame " << g_completed << " ready" << std::endl;
        if (g_completed >= kWarmupFrames) {
            g_final_request = request;
            g_cv.notify_all();
            return;
        }
    }

    request->reuse(Request::ReuseBuffers);
    g_camera->queueRequest(request);
}

int main(int argc, char* argv[]) {
    const std::string out_path = (argc > 1) ? argv[1] : "/tmp/capture.ppm";

    auto cm = std::make_unique<CameraManager>();
    if (cm->start()) {
        std::cerr << "CameraManager::start() failed\n";
        return 1;
    }

    auto cams = cm->cameras();
    if (cams.empty()) {
        std::cerr << "No cameras detected by libcamera\n";
        return 1;
    }

    g_camera = cams[0];
    std::cout << "Camera: " << g_camera->id() << std::endl;

    if (g_camera->acquire()) {
        std::cerr << "Camera::acquire() failed\n";
        return 1;
    }

    auto config = g_camera->generateConfiguration({ StreamRole::StillCapture });
    if (!config || config->empty()) {
        std::cerr << "generateConfiguration returned no streams\n";
        return 1;
    }

    StreamConfiguration& cfg = config->at(0);
    std::cout << "Default still: " << cfg.toString() << std::endl;

    // Ask for a packed BGR888 buffer at a sane mid resolution.
    cfg.pixelFormat = formats::BGR888;
    cfg.size.width = 2304;
    cfg.size.height = 1296;
    cfg.bufferCount = 4;

    auto status = config->validate();
    if (status == CameraConfiguration::Invalid) {
        std::cerr << "Configuration invalid after validate()\n";
        return 1;
    }
    std::cout << "Validated:    " << cfg.toString() << std::endl;

    if (g_camera->configure(config.get()) < 0) {
        std::cerr << "Camera::configure() failed\n";
        return 1;
    }

    FrameBufferAllocator allocator(g_camera);
    Stream* stream = cfg.stream();
    if (allocator.allocate(stream) < 0) {
        std::cerr << "FrameBufferAllocator::allocate() failed\n";
        return 1;
    }

    const auto& buffers = allocator.buffers(stream);
    std::cout << "Allocated " << buffers.size() << " buffers" << std::endl;

    for (const auto& buf : buffers) {
        const FrameBuffer::Plane& plane = buf->planes()[0];
        void* p = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                       MAP_SHARED, plane.fd.get(), 0);
        if (p == MAP_FAILED) {
            std::cerr << "mmap failed\n";
            return 1;
        }
        g_mapped[buf.get()] = p;
        g_mapped_len[buf.get()] = plane.length;
    }

    std::vector<std::unique_ptr<Request>> requests;
    for (const auto& buf : buffers) {
        auto req = g_camera->createRequest();
        if (!req) {
            std::cerr << "createRequest failed\n";
            return 1;
        }
        if (req->addBuffer(stream, buf.get()) < 0) {
            std::cerr << "Request::addBuffer failed\n";
            return 1;
        }
        requests.push_back(std::move(req));
    }

    g_camera->requestCompleted.connect(onRequestComplete);

    if (g_camera->start()) {
        std::cerr << "Camera::start() failed\n";
        return 1;
    }

    for (auto& req : requests) {
        if (g_camera->queueRequest(req.get()) < 0) {
            std::cerr << "queueRequest failed\n";
            return 1;
        }
    }

    {
        std::unique_lock<std::mutex> lock(g_mutex);
        if (!g_cv.wait_for(lock, std::chrono::seconds(10),
                           [] { return g_final_request != nullptr; })) {
            std::cerr << "Timeout waiting for capture\n";
            g_camera->stop();
            return 1;
        }
    }

    FrameBuffer* fb = g_final_request->buffers().begin()->second;
    void* data = g_mapped[fb];

    const unsigned w = cfg.size.width;
    const unsigned h = cfg.size.height;
    const unsigned stride = cfg.stride;
    std::cout << "Saving " << w << "x" << h << " stride=" << stride
              << " -> " << out_path << std::endl;

    std::ofstream out(out_path, std::ios::binary);
    if (!out) {
        std::cerr << "Failed to open " << out_path << " for writing\n";
        return 1;
    }
    out << "P6\n" << w << " " << h << "\n255\n";

    std::vector<std::uint8_t> row(static_cast<std::size_t>(w) * 3);
    const auto* src = static_cast<const std::uint8_t*>(data);
    for (unsigned y = 0; y < h; ++y) {
        const std::uint8_t* p = src + static_cast<std::size_t>(y) * stride;
        for (unsigned x = 0; x < w; ++x) {
            // libcamera "BGR888" packs pixels in B,G,R order.
            // PPM (P6) wants R,G,B.
            row[x * 3 + 0] = p[x * 3 + 2];
            row[x * 3 + 1] = p[x * 3 + 1];
            row[x * 3 + 2] = p[x * 3 + 0];
        }
        out.write(reinterpret_cast<const char*>(row.data()),
                  static_cast<std::streamsize>(row.size()));
    }
    out.close();
    std::cout << "Wrote " << out_path << std::endl;

    g_camera->stop();

    for (auto& kv : g_mapped) {
        munmap(kv.second, g_mapped_len[kv.first]);
    }

    allocator.free(stream);
    g_camera->release();
    g_camera.reset();
    cm->stop();

    return 0;
}
