// Camera node for the Duck robot.
//
// Drives the Raspberry Pi Camera Module 3 Wide (IMX708) on the Pi 5 via libcamera 0.5
// (the pisp pipeline). Publishes:
//
//   ~/image_raw    sensor_msgs/Image       (encoding: bgr8)
//   ~/camera_info  sensor_msgs/CameraInfo
//
// Both topics are namespaced under the node name (default `camera_node`), so a typical
// launch will see them as `/camera/image_raw` and `/camera/camera_info`.
//
// See ../../camera.md for the libcamera install steps and the gotchas of this hybrid
// libcamera 0.2/0.5 stack on Ubuntu Noble.

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/libcamera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : rclcpp::Node("camera_node") {
        // ---- Parameters --------------------------------------------------
        width_           = this->declare_parameter<int>("width", 1280);
        height_          = this->declare_parameter<int>("height", 720);
        framerate_       = this->declare_parameter<double>("framerate", 15.0);
        frame_id_        = this->declare_parameter<std::string>("frame_id", "camera_optical_link");
        camera_name_     = this->declare_parameter<std::string>("camera_name", "imx708");
        camera_info_url_ = this->declare_parameter<std::string>("camera_info_url", "");
        rotation_deg_    = this->declare_parameter<int>("rotation_deg", 0);

        // QoS reliability for image + camera_info topics. Default to "reliable" so RViz
        // and other default-QoS tools work out of the box. Switch to "best_effort" if
        // you need maximum throughput on slow subscribers — note that BEST_EFFORT lets
        // DDS drop large image frames silently under load.
        const std::string qos_reliability =
            this->declare_parameter<std::string>("qos_reliability", "reliable");

        if (rotation_deg_ != 0 && rotation_deg_ != 180) {
            RCLCPP_WARN(get_logger(),
                        "rotation_deg=%d not supported (only 0 or 180); falling back to 0",
                        rotation_deg_);
            rotation_deg_ = 0;
        }

        // ---- Publishers --------------------------------------------------
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        if (qos_reliability == "best_effort") {
            qos.best_effort();
            RCLCPP_INFO(get_logger(), "QoS reliability: BEST_EFFORT");
        } else {
            if (qos_reliability != "reliable") {
                RCLCPP_WARN(get_logger(),
                            "Unknown qos_reliability='%s', falling back to 'reliable'",
                            qos_reliability.c_str());
            }
            qos.reliable();
            RCLCPP_INFO(get_logger(), "QoS reliability: RELIABLE");
        }
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image_raw", qos);
        info_pub_  = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", qos);

        // ---- CameraInfo --------------------------------------------------
        cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this, camera_name_, camera_info_url_);

        // ---- libcamera setup --------------------------------------------
        // Spawn the publisher thread BEFORE the camera starts so it is ready to drain
        // the slot as soon as libcamera fires the first completion callback.
        publisher_thread_ = std::thread(&CameraNode::publisherLoop, this);

        if (!initCamera()) {
            stop_publisher_ = true;
            slot_cv_.notify_all();
            if (publisher_thread_.joinable()) publisher_thread_.join();
            throw std::runtime_error("Camera initialisation failed");
        }

        RCLCPP_INFO(get_logger(),
                    "duck_vision camera_node up: %dx%d @ %.1f fps, frame_id='%s'",
                    width_, height_, framerate_, frame_id_.c_str());
    }

    ~CameraNode() override {
        // Order matters:
        //   1. cleanupCamera() — synchronously stops libcamera. After it returns no
        //      more onRequestComplete callbacks will fire, so the slot stops getting
        //      new entries.
        //   2. Signal the publisher thread to stop and join it. Doing this AFTER
        //      cleanupCamera() means we don't race with libcamera trying to deliver
        //      one final frame while the publisher thread is exiting.
        cleanupCamera();

        {
            std::lock_guard<std::mutex> lock(slot_mutex_);
            stop_publisher_ = true;
        }
        slot_cv_.notify_all();
        if (publisher_thread_.joinable()) publisher_thread_.join();
    }

private:
    // -----------------------------------------------------------------
    // libcamera setup / teardown
    // -----------------------------------------------------------------
    bool initCamera() {
        cm_ = std::make_unique<libcamera::CameraManager>();
        if (cm_->start()) {
            RCLCPP_ERROR(get_logger(), "CameraManager::start() failed");
            return false;
        }

        auto cams = cm_->cameras();
        if (cams.empty()) {
            RCLCPP_ERROR(get_logger(),
                         "No cameras detected by libcamera. "
                         "Hint: Ubuntu's `cam` CLI is the stale 0.2 build and is not "
                         "diagnostic — but if this node fails to find a camera, check "
                         "`pkg-config --modversion libcamera` is 0.5+ and that "
                         "/usr/share/libcamera/pipeline/rpi/pisp exists.");
            return false;
        }

        camera_ = cams[0];
        RCLCPP_INFO(get_logger(), "libcamera id: %s", camera_->id().c_str());

        if (camera_->acquire()) {
            RCLCPP_ERROR(get_logger(), "Camera::acquire() failed (in use by another process?)");
            return false;
        }

        config_ = camera_->generateConfiguration({ libcamera::StreamRole::Viewfinder });
        if (!config_ || config_->empty()) {
            RCLCPP_ERROR(get_logger(), "generateConfiguration returned no streams");
            return false;
        }

        libcamera::StreamConfiguration& cfg = config_->at(0);
        cfg.pixelFormat = libcamera::formats::BGR888;
        cfg.size.width  = width_;
        cfg.size.height = height_;
        cfg.bufferCount = 4;

        const auto status = config_->validate();
        if (status == libcamera::CameraConfiguration::Invalid) {
            RCLCPP_ERROR(get_logger(), "CameraConfiguration::validate() returned Invalid");
            return false;
        }
        if (status == libcamera::CameraConfiguration::Adjusted) {
            RCLCPP_WARN(get_logger(),
                        "Configuration adjusted by libcamera to: %s",
                        cfg.toString().c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Configuration: %s", cfg.toString().c_str());
        }
        // Adopt the validated dimensions/stride.
        width_  = static_cast<int>(cfg.size.width);
        height_ = static_cast<int>(cfg.size.height);
        stride_ = cfg.stride;

        if (camera_->configure(config_.get()) < 0) {
            RCLCPP_ERROR(get_logger(), "Camera::configure() failed");
            return false;
        }

        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        stream_ = cfg.stream();
        if (allocator_->allocate(stream_) < 0) {
            RCLCPP_ERROR(get_logger(), "FrameBufferAllocator::allocate() failed");
            return false;
        }

        // mmap each buffer once and remember the pointer.
        for (const auto& buf : allocator_->buffers(stream_)) {
            const libcamera::FrameBuffer::Plane& plane = buf->planes()[0];
            void* p = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, plane.fd.get(), 0);
            if (p == MAP_FAILED) {
                RCLCPP_ERROR(get_logger(), "mmap of frame buffer failed");
                return false;
            }
            mapped_[buf.get()] = p;
            mapped_len_[buf.get()] = plane.length;
        }

        for (const auto& buf : allocator_->buffers(stream_)) {
            std::unique_ptr<libcamera::Request> req = camera_->createRequest();
            if (!req) {
                RCLCPP_ERROR(get_logger(), "createRequest() failed");
                return false;
            }
            if (req->addBuffer(stream_, buf.get()) < 0) {
                RCLCPP_ERROR(get_logger(), "Request::addBuffer failed");
                return false;
            }
            requests_.push_back(std::move(req));
        }

        // Wire the completion signal to our member function. libcamera::Signal supports
        // non-Object receivers via a direct (same-thread) connection, so onRequestComplete
        // will be invoked on libcamera's internal thread — keep that thread-safety in mind
        // when extending it.
        camera_->requestCompleted.connect(this, &CameraNode::onRequestComplete);

        // Cap frame duration to enforce the requested framerate.
        const std::int64_t frame_us =
            static_cast<std::int64_t>(1.0e6 / std::max(framerate_, 1.0));
        std::array<std::int64_t, 2> durations = { frame_us, frame_us };
        libcamera::ControlList controls(camera_->controls());
        controls.set(libcamera::controls::FrameDurationLimits,
                     libcamera::Span<const std::int64_t, 2>(durations));

        if (camera_->start(&controls)) {
            RCLCPP_ERROR(get_logger(), "Camera::start() failed");
            return false;
        }

        for (auto& req : requests_) {
            if (camera_->queueRequest(req.get()) < 0) {
                RCLCPP_ERROR(get_logger(), "queueRequest() failed");
                return false;
            }
        }

        return true;
    }

    void cleanupCamera() {
        if (camera_) {
            camera_->stop();
            for (auto& kv : mapped_) {
                munmap(kv.second, mapped_len_[kv.first]);
            }
            mapped_.clear();
            mapped_len_.clear();
            requests_.clear();
            if (allocator_ && stream_) allocator_->free(stream_);
            allocator_.reset();
            camera_->release();
            camera_.reset();
        }
        if (cm_) {
            cm_->stop();
            cm_.reset();
        }
    }

    // -----------------------------------------------------------------
    // Capture callback (runs on libcamera's thread)
    // -----------------------------------------------------------------
    // Runs on libcamera's internal thread. MUST NOT block on the publisher.
    void onRequestComplete(libcamera::Request* request) {
        if (request->status() == libcamera::Request::RequestCancelled) return;

        libcamera::FrameBuffer* fb = request->buffers().begin()->second;
        auto img = buildImage(fb);

        // Hand the frame off to the publisher thread via a single-slot "latest" buffer.
        // If the previous frame has not been published yet, drop it — keeping libcamera's
        // capture thread completely free of publish-side backpressure.
        {
            std::lock_guard<std::mutex> lock(slot_mutex_);
            if (pending_image_) ++dropped_frames_;
            pending_image_ = std::move(img);
        }
        slot_cv_.notify_one();

        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
    }

    // Build a ROS Image from a libcamera buffer. Pure data work; no I/O, no locks.
    sensor_msgs::msg::Image::UniquePtr buildImage(libcamera::FrameBuffer* fb) {
        const auto* src = static_cast<const std::uint8_t*>(mapped_[fb]);

        auto img = std::make_unique<sensor_msgs::msg::Image>();
        img->header.stamp    = this->now();
        img->header.frame_id = frame_id_;
        img->height          = static_cast<std::uint32_t>(height_);
        img->width           = static_cast<std::uint32_t>(width_);
        // libcamera's `formats::BGR888` actually maps to V4L2_PIX_FMT_RGB24, which is
        // R, G, B byte order — the `BGR` in the libcamera name is a historical misnomer.
        // So the correct ROS encoding label is rgb8, NOT bgr8.
        img->encoding        = "rgb8";
        img->is_bigendian    = 0;
        img->step            = static_cast<std::uint32_t>(width_) * 3;
        img->data.resize(static_cast<std::size_t>(height_) * img->step);

        if (rotation_deg_ == 0) {
            // Plain row-by-row copy (skips libcamera's row stride padding).
            for (int y = 0; y < height_; ++y) {
                std::memcpy(img->data.data() + static_cast<std::size_t>(y) * img->step,
                            src + static_cast<std::size_t>(y) * stride_,
                            img->step);
            }
        } else {
            // 180° rotation: reverse rows AND reverse pixels in each row.
            for (int y = 0; y < height_; ++y) {
                const std::uint8_t* sr =
                    src + static_cast<std::size_t>(height_ - 1 - y) * stride_;
                std::uint8_t* dr =
                    img->data.data() + static_cast<std::size_t>(y) * img->step;
                for (int x = 0; x < width_; ++x) {
                    const int sx = (width_ - 1 - x) * 3;
                    const int dx = x * 3;
                    dr[dx + 0] = sr[sx + 0];
                    dr[dx + 1] = sr[sx + 1];
                    dr[dx + 2] = sr[sx + 2];
                }
            }
        }

        return img;
    }

    // Runs on a dedicated thread. Blocks on publisher backpressure (RELIABLE QoS) so
    // libcamera's capture thread does not.
    void publisherLoop() {
        auto last_drop_log = std::chrono::steady_clock::now();
        while (true) {
            sensor_msgs::msg::Image::UniquePtr img;
            {
                std::unique_lock<std::mutex> lock(slot_mutex_);
                slot_cv_.wait(lock, [this] { return pending_image_ || stop_publisher_; });
                if (stop_publisher_ && !pending_image_) return;
                img = std::move(pending_image_);
            }

            if (!img) continue;

            sensor_msgs::msg::CameraInfo info = cinfo_manager_->getCameraInfo();
            info.header = img->header;
            if (info.width == 0 || info.height == 0) {
                info.width  = img->width;
                info.height = img->height;
            }

            image_pub_->publish(std::move(img));
            info_pub_->publish(info);

            // Periodically log how many frames we have dropped because the subscriber
            // could not keep up. Once every 5 seconds keeps the log readable.
            const auto now = std::chrono::steady_clock::now();
            if (now - last_drop_log > std::chrono::seconds(5)) {
                const std::size_t dropped = dropped_frames_.exchange(0);
                if (dropped > 0) {
                    RCLCPP_WARN(get_logger(),
                                "Dropped %zu frames in the last 5s (publisher slower "
                                "than capture). Consider lowering framerate or resolution, "
                                "or running RViz on a faster machine.",
                                dropped);
                }
                last_drop_log = now;
            }
        }
    }

    // -----------------------------------------------------------------
    // State
    // -----------------------------------------------------------------
    int width_;
    int height_;
    double framerate_;
    int rotation_deg_;
    unsigned int stride_ = 0;
    std::string frame_id_;
    std::string camera_name_;
    std::string camera_info_url_;

    std::unique_ptr<libcamera::CameraManager> cm_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_ = nullptr;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::map<libcamera::FrameBuffer*, void*> mapped_;
    std::map<libcamera::FrameBuffer*, std::size_t> mapped_len_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

    // Publisher thread + single-slot frame handoff. The libcamera callback writes to
    // pending_image_ (overwriting any unconsumed frame); publisherLoop() drains it.
    std::thread publisher_thread_;
    std::mutex slot_mutex_;
    std::condition_variable slot_cv_;
    sensor_msgs::msg::Image::UniquePtr pending_image_;
    bool stop_publisher_ = false;
    std::atomic<std::size_t> dropped_frames_{ 0 };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "Fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
