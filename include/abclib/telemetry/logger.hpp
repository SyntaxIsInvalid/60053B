#pragma once

#include <string>
#include <cstdio>
#include <cstdint>

namespace abclib
{
    /**
     * @brief Configuration for which telemetry fields to log
     * 
     * All fields default to true (log everything).
     * Set to false to exclude specific categories from CSV output.
     */
    struct LogFields
    {
        bool pose = true;                // x, y, theta
        bool velocity = true;            // v, omega (filtered and raw)
        bool lateral_pid = true;         // lateral control data
        bool angular_pid = true;         // angular control data  
        bool settlement = true;          // settlement tracking
        bool motors = true;              // motor voltages
        bool path_tracking = true;       // cross-track and along-track errors
        bool path_status = true;         // trajectory status and progress
        bool wheels = true;              // individual wheel velocities
        bool turn_in_place = true;       // turn-in-place specific data
        bool timing = true;              // loop timing metrics
        bool battery = true;             // battery monitoring
    };

    /**
     * @brief CSV logger for robot telemetry data
     * 
     * Reads from the global telemetry instance and writes timestamped snapshots to CSV files.
     * Supports two modes:
     * - Continuous: Single file with optional event markers
     * - Segmented: Creates new files when mark() is called
     * 
     * Thread-safe: Safely reads from telemetry using existing mutex
     * 
     * @example
     * // Log everything (default)
     * Logger logger("auto_routine");
     * 
     * @example
     * // Log only specific fields
     * Logger logger("minimal", false, {.pose = true, .battery = true});
     * 
     * @example
     * // Segmented logging with custom fields
     * LogFields fields;
     * fields.timing = false;
     * fields.wheels = false;
     * Logger logger("tuning", true, fields);
     */
    class Logger
    {
    public:
        /**
         * @brief Construct a logger
         * @param name Base name for log file(s)
         * @param auto_segment If true, mark() creates new files. If false, mark() adds event column
         * @param fields Configuration for which telemetry fields to log (defaults to all)
         */
        explicit Logger(const std::string& name, bool auto_segment = false, 
                       const LogFields& fields = LogFields{});
        
        /**
         * @brief Destructor - closes any open file
         */
        ~Logger();

        /**
         * @brief Log current telemetry snapshot to CSV
         * 
         * Reads from global telemetry (thread-safe), extracts raw values from typed units,
         * and writes a timestamped row to the current file.
         * 
         * @note If no file is open yet, creates initial file
         */
        void log();

        /**
         * @brief Mark an event or create new segment
         * @param segment_name Descriptive name for event/segment (empty string = auto-numbered)
         * 
         * Behavior depends on auto_segment mode:
         * - If auto_segment=false: Adds event marker to current file
         * - If auto_segment=true: Closes current file, opens new one named <base>_<segment_name>.csv
         * 
         * If segment_name is empty in segmented mode, auto-numbers as segment_0, segment_1, etc.
         */
        void mark(const std::string& segment_name = "");

    private:
        std::string base_name_;           // Base name passed to constructor
        bool auto_segment_;               // Whether to create new files on mark()
        LogFields fields_;                // Which fields to log
        FILE* file_;                      // Current open file handle
        uint32_t start_time_;             // millis() when logger was created
        int segment_counter_;             // Auto-numbering for unnamed segments
        std::string current_event_marker_; // Current event marker for non-segmented mode

        /**
         * @brief Opens a new CSV file and writes header row
         * @param segment_name Segment name to append to filename (empty = initial file or auto-numbered)
         */
        void open_new_file(const std::string& segment_name = "");

        /**
         * @brief Closes currently open file (if any)
         */
        void close_current_file();

        /**
         * @brief Writes CSV header row with column names
         */
        void write_header();

        /**
         * @brief Writes telemetry data as CSV row
         * @param absolute_time Current system time in milliseconds
         * @param elapsed_time Time since logger creation in seconds
         */
        void write_row(uint32_t absolute_time, double elapsed_time);

        /**
         * @brief Generates filename for current segment
         * @param segment_name Segment identifier (may be empty)
         * @return Full path to CSV file
         */
        std::string generate_filename(const std::string& segment_name) const;
    };

} // namespace abclib