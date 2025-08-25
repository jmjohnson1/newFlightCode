#ifdef SITL_BUILD

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <vector>
#include <map>

#include "Eigen/Dense"

/**
 * @brief SITL Data Logger - Enhanced logging for simulation analysis
 */
class SITLDataLogger {
public:
    struct LogEntry {
        std::string name;
        std::string value;
        uint64_t timestamp_us;
    };
    
    SITLDataLogger(const std::string& filename = "sitl_log.csv") 
        : filename_(filename), logging_enabled_(false), log_counter_(0) {}
    
    ~SITLDataLogger() {
        stop();
    }
    
    // Start logging
    bool start() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (logging_enabled_) {
            return true; // Already started
        }
        
        log_file_.open(filename_);
        if (!log_file_.is_open()) {
            std::cerr << "Failed to open log file: " << filename_ << std::endl;
            return false;
        }
        
        // Write CSV header
        writeHeader();
        
        logging_enabled_ = true;
        start_time_ = std::chrono::steady_clock::now();
        
        std::cout << "SITL Data Logger started: " << filename_ << std::endl;
        return true;
    }
    
    // Stop logging
    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!logging_enabled_) {
            return;
        }
        
        if (log_file_.is_open()) {
            log_file_.close();
        }
        
        logging_enabled_ = false;
        std::cout << "SITL Data Logger stopped. Logged " << log_counter_ << " entries." << std::endl;
    }
    
    // Log a single value
    template<typename T>
    void log(const std::string& name, const T& value) {
        if (!logging_enabled_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto now = std::chrono::steady_clock::now();
        uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now - start_time_).count();
        
        pending_entries_[name] = {name, toString(value), timestamp_us};
    }
    
    // Log multiple values at once (more efficient)
    void logBatch(const std::map<std::string, std::string>& values) {
        if (!logging_enabled_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto now = std::chrono::steady_clock::now();
        uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now - start_time_).count();
        
        for (const auto& pair : values) {
            pending_entries_[pair.first] = {pair.first, pair.second, timestamp_us};
        }
    }
    
    // Log Eigen vectors
    void log(const std::string& base_name, const Eigen::Vector3f& vec) {
        log(base_name + "_x", vec[0]);
        log(base_name + "_y", vec[1]);  
        log(base_name + "_z", vec[2]);
    }
    
    void log(const std::string& base_name, const Eigen::Vector3d& vec) {
        log(base_name + "_x", vec[0]);
        log(base_name + "_y", vec[1]);
        log(base_name + "_z", vec[2]);
    }
    
    void log(const std::string& base_name, const Eigen::Vector4f& vec) {
        log(base_name + "_0", vec[0]);
        log(base_name + "_1", vec[1]);
        log(base_name + "_2", vec[2]);
        log(base_name + "_3", vec[3]);
    }
    
    // Log arrays
    template<typename T>
    void log(const std::string& base_name, const T* array, size_t length) {
        for (size_t i = 0; i < length; ++i) {
            log(base_name + "_" + std::to_string(i), array[i]);
        }
    }
    
    // Flush pending entries to file (call this periodically)
    void flush() {
        if (!logging_enabled_) return;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (pending_entries_.empty()) return;
        
        // Write timestamp
        auto now = std::chrono::steady_clock::now();
        uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            now - start_time_).count();
        
        log_file_ << timestamp_us;
        
        // Write all registered columns
        for (const auto& col : column_order_) {
            log_file_ << ",";
            auto it = pending_entries_.find(col);
            if (it != pending_entries_.end()) {
                log_file_ << it->second.value;
            } else {
                log_file_ << ""; // Empty value for missing data
            }
        }
        
        log_file_ << "\n";
        log_file_.flush();
        
        pending_entries_.clear();
        log_counter_++;
    }
    
    // Force immediate write (for critical data)
    void flushImmediate() {
        flush();
        if (log_file_.is_open()) {
            log_file_.flush();
        }
    }
    
    // Register columns in advance (for consistent CSV structure)
    void registerColumns(const std::vector<std::string>& columns) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        for (const auto& col : columns) {
            if (std::find(column_order_.begin(), column_order_.end(), col) == column_order_.end()) {
                column_order_.push_back(col);
            }
        }
        
        if (logging_enabled_ && log_file_.is_open()) {
            // Rewrite header if logging already started
            log_file_.seekp(0);
            writeHeader();
        }
    }
    
    // Get logging statistics
    struct Stats {
        bool is_logging;
        uint64_t entries_logged;
        std::string filename;
        double elapsed_time_s;
        size_t file_size_bytes;
    };
    
    Stats getStats() const {
        Stats stats;
        stats.is_logging = logging_enabled_;
        stats.entries_logged = log_counter_;
        stats.filename = filename_;
        
        if (logging_enabled_) {
            auto now = std::chrono::steady_clock::now();
            stats.elapsed_time_s = std::chrono::duration<double>(now - start_time_).count();
        } else {
            stats.elapsed_time_s = 0.0;
        }
        
        // Get file size
        if (log_file_.is_open()) {
            auto pos = log_file_.tellp();
            stats.file_size_bytes = static_cast<size_t>(pos);
        } else {
            stats.file_size_bytes = 0;
        }
        
        return stats;
    }

private:
    std::string filename_;
    std::ofstream log_file_;
    bool logging_enabled_;
    uint64_t log_counter_;
    std::chrono::steady_clock::time_point start_time_;
    
    std::mutex mutex_;
    std::map<std::string, LogEntry> pending_entries_;
    std::vector<std::string> column_order_;
    
    void writeHeader() {
        if (!log_file_.is_open()) return;
        
        log_file_ << "timestamp_us";
        for (const auto& col : column_order_) {
            log_file_ << "," << col;
        }
        log_file_ << "\n";
    }
    
    // Convert various types to string
    template<typename T>
    std::string toString(const T& value) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << value;
        return oss.str();
    }
    
    std::string toString(bool value) {
        return value ? "1" : "0";
    }
    
    std::string toString(const std::string& value) {
        return value;
    }
    
    std::string toString(const char* value) {
        return std::string(value);
    }
};

// Global logger instance
static SITLDataLogger* g_logger = nullptr;
static std::mutex g_logger_mutex;

// C-style interface for integration with existing flight code
extern "C" {
    
    // Initialize logger
    bool sitl_logger_init(const char* filename) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        
        if (g_logger) {
            delete g_logger;
        }
        
        g_logger = new SITLDataLogger(filename ? filename : "sitl_log.csv");
        return g_logger->start();
    }
    
    // Shutdown logger
    void sitl_logger_shutdown() {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        
        if (g_logger) {
            g_logger->stop();
            delete g_logger;
            g_logger = nullptr;
        }
    }
    
    // Log functions for C code
    void sitl_log_float(const char* name, float value) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(std::string(name), value);
        }
    }
    
    void sitl_log_double(const char* name, double value) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(std::string(name), value);
        }
    }
    
    void sitl_log_int(const char* name, int value) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(std::string(name), value);
        }
    }
    
    void sitl_log_bool(const char* name, bool value) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(std::string(name), value);
        }
    }
    
    void sitl_log_string(const char* name, const char* value) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(std::string(name), std::string(value));
        }
    }
    
    void sitl_log_vector3f(const char* name, float x, float y, float z) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            Eigen::Vector3f vec(x, y, z);
            g_logger->log(std::string(name), vec);
        }
    }
    
    void sitl_log_vector4f(const char* name, float w, float x, float y, float z) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            Eigen::Vector4f vec(w, x, y, z);
            g_logger->log(std::string(name), vec);
        }
    }
    
    // Flush data to file
    void sitl_log_flush() {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->flush();
        }
    }
    
    // Register expected columns
    void sitl_log_register_columns(const char** columns, int count) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger && columns) {
            std::vector<std::string> col_vec;
            for (int i = 0; i < count; ++i) {
                col_vec.push_back(std::string(columns[i]));
            }
            g_logger->registerColumns(col_vec);
        }
    }
    
    // Get logging stats
    typedef struct {
        bool is_logging;
        uint64_t entries_logged;
        double elapsed_time_s;
        size_t file_size_bytes;
    } sitl_logger_stats_t;
    
    sitl_logger_stats_t sitl_log_get_stats() {
        sitl_logger_stats_t stats = {false, 0, 0.0, 0};
        
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            auto cpp_stats = g_logger->getStats();
            stats.is_logging = cpp_stats.is_logging;
            stats.entries_logged = cpp_stats.entries_logged;
            stats.elapsed_time_s = cpp_stats.elapsed_time_s;
            stats.file_size_bytes = cpp_stats.file_size_bytes;
        }
        
        return stats;
    }
}

// C++ interface for your flight code
namespace SITL {
    
    bool initLogger(const std::string& filename = "sitl_log.csv") {
        return sitl_logger_init(filename.c_str());
    }
    
    void shutdownLogger() {
        sitl_logger_shutdown();
    }
    
    template<typename T>
    void log(const std::string& name, const T& value) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(name, value);
        }
    }
    
    void log(const std::string& name, const Eigen::Vector3f& vec) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(name, vec);
        }
    }
    
    void log(const std::string& name, const Eigen::Vector4f& vec) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->log(name, vec);
        }
    }
    
    void flush() {
        sitl_log_flush();
    }
    
    void registerColumns(const std::vector<std::string>& columns) {
        std::lock_guard<std::mutex> lock(g_logger_mutex);
        if (g_logger) {
            g_logger->registerColumns(columns);
        }
    }
}

#endif // SITL_BUILD
