#ifdef SITL_BUILD

#include "teensy_hal.h"
#include <chrono>
#include <thread>
#include <vector>
#include <functional>
#include <mutex>

// TeensyTimerTool simulation for SITL
namespace TeensyTimerTool {

// Timer channel simulation
class SITLTimerChannel {
public:
    SITLTimerChannel() : active_(false), periodic_(false), period_us_(0) {}
    
    bool begin(std::function<void()> callback, uint32_t period_us, bool periodic) {
        callback_ = callback;
        period_us_ = period_us;
        periodic_ = periodic;
        last_trigger_ = std::chrono::steady_clock::now();
        return true;
    }
    
    void start() {
        active_ = true;
        last_trigger_ = std::chrono::steady_clock::now();
    }
    
    void stop() {
        active_ = false;
    }
    
    void update() {
        if (!active_ || !callback_) return;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_trigger_).count();
            
        if (elapsed >= period_us_) {
            callback_();
            last_trigger_ = now;
            
            if (!periodic_) {
                active_ = false;
            }
        }
    }
    
    float getMaxPeriod() const { return 1000000.0f; } // 1 second max
    float getRemainingTime() const {
        if (!active_) return 0.0f;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_trigger_).count();
        return std::max(0.0f, static_cast<float>(period_us_ - elapsed));
    }

private:
    bool active_;
    bool periodic_;
    uint32_t period_us_;
    std::function<void()> callback_;
    std::chrono::steady_clock::time_point last_trigger_;
};

// Timer pool management
static std::vector<SITLTimerChannel*> timer_pool;
static std::mutex timer_mutex;
static std::thread* timer_thread = nullptr;
static bool timer_system_running = false;

// Timer system update thread
void timer_update_thread() {
    while (timer_system_running) {
        {
            std::lock_guard<std::mutex> lock(timer_mutex);
            for (auto* timer : timer_pool) {
                if (timer) {
                    timer->update();
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100)); // 10kHz update rate
    }
}

// Initialize timer system
void init_timer_system() {
    if (!timer_system_running) {
        timer_system_running = true;
        timer_thread = new std::thread(timer_update_thread);
    }
}

// Shutdown timer system
void shutdown_timer_system() {
    if (timer_system_running) {
        timer_system_running = false;
        if (timer_thread && timer_thread->joinable()) {
            timer_thread->join();
            delete timer_thread;
            timer_thread = nullptr;
        }
    }
}

// Allocate a new timer channel
SITLTimerChannel* allocate_timer() {
    std::lock_guard<std::mutex> lock(timer_mutex);
    
    if (!timer_system_running) {
        init_timer_system();
    }
    
    SITLTimerChannel* timer = new SITLTimerChannel();
    timer_pool.push_back(timer);
    return timer;
}

// Free a timer channel
void free_timer(SITLTimerChannel* timer) {
    std::lock_guard<std::mutex> lock(timer_mutex);
    
    auto it = std::find(timer_pool.begin(), timer_pool.end(), timer);
    if (it != timer_pool.end()) {
        timer_pool.erase(it);
        delete timer;
    }
}

} // namespace TeensyTimerTool

// SITL-specific timer implementations
namespace {
    std::vector<TeensyTimerTool::SITLTimerChannel*> allocated_timers;
}

// Mock IntervalTimer (simple wrapper around SITLTimerChannel)
class SITLIntervalTimer {
public:
    SITLIntervalTimer() : timer_channel_(nullptr) {}
    
    ~SITLIntervalTimer() {
        end();
    }
    
    bool begin(void (*function)(void), uint32_t microseconds) {
        timer_channel_ = TeensyTimerTool::allocate_timer();
        if (timer_channel_) {
            return timer_channel_->begin([function](){ function(); }, microseconds, true);
        }
        return false;
    }
    
    void end() {
        if (timer_channel_) {
            timer_channel_->stop();
            TeensyTimerTool::free_timer(timer_channel_);
            timer_channel_ = nullptr;
        }
    }
    
private:
    TeensyTimerTool::SITLTimerChannel* timer_channel_;
};

// Global IntervalTimer instances (if your code uses them)
static std::vector<SITLIntervalTimer*> interval_timers;

// Public functions for creating interval timers
SITLIntervalTimer* createIntervalTimer() {
    SITLIntervalTimer* timer = new SITLIntervalTimer();
    interval_timers.push_back(timer);
    return timer;
}

// Cleanup function (call on shutdown)
void cleanupTimers() {
    for (auto* timer : interval_timers) {
        delete timer;
    }
    interval_timers.clear();
    
    TeensyTimerTool::shutdown_timer_system();
}

#endif // SITL_BUILD