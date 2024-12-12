#ifndef ld2410_h
#define ld2410_h

#include <Arduino.h>

// Check if we're on ESP32 or another platform with FreeRTOS
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
#define USE_FREERTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Configuration
#define LD2410_MAX_FRAME_LENGTH 40

#ifdef USE_FREERTOS
    #ifndef LD2410_BUFFER_SIZE
    #define LD2410_BUFFER_SIZE 256
    #endif
#endif

class ld2410 {
    public:
        ld2410();
        ~ld2410();
        bool begin(Stream &, bool waitForRadar = true);
        void debug(Stream &);
        bool isConnected();
        
        // Core reading functions - platform dependent implementation
        bool read();
        
        // Common methods for all platforms
        bool presenceDetected();
        bool stationaryTargetDetected();
        uint16_t stationaryTargetDistance();
        uint8_t stationaryTargetEnergy();
        bool movingTargetDetected();
        uint16_t movingTargetDistance();
        uint8_t movingTargetEnergy();
        
        // Configuration methods
        bool requestFirmwareVersion();
        bool requestCurrentConfiguration();
        bool requestRestart();
        bool requestFactoryReset();
        bool requestStartEngineeringMode();
        bool requestEndEngineeringMode();
        bool setMaxValues(uint16_t moving, uint16_t stationary, uint16_t inactivityTimer);
        bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary);
        
        // Version info
        uint8_t firmware_major_version = 0;
        uint8_t firmware_minor_version = 0;
        uint32_t firmware_bugfix_version = 0;
        
        // Configuration values
        uint8_t max_gate = 0;
        uint8_t max_moving_gate = 0;
        uint8_t max_stationary_gate = 0;
        uint16_t sensor_idle_time = 0;
        uint8_t motion_sensitivity[9] = {0,0,0,0,0,0,0,0,0};
        uint8_t stationary_sensitivity[9] = {0,0,0,0,0,0,0,0,0};

        #ifdef USE_FREERTOS
        void autoReadTask(uint32_t stack, uint32_t priority, uint32_t core);
        #endif

    protected:
        Stream *radar_uart_ = nullptr;
        Stream *debug_uart_ = nullptr;
        uint32_t radar_uart_timeout = 100;
        uint32_t radar_uart_last_packet_ = 0;
        uint32_t radar_uart_last_command_ = 0;
        uint32_t radar_uart_command_timeout_ = 100;
        
        // Frame processing
        uint8_t radar_data_frame_[LD2410_MAX_FRAME_LENGTH];
        uint8_t radar_data_frame_position_ = 0;
        bool frame_started_ = false;
        bool ack_frame_ = false;
        bool waiting_for_ack_ = false;
        uint8_t latest_ack_ = 0;
        bool latest_command_success_ = false;

        // Target data
        uint8_t target_type_ = 0;
        uint16_t moving_target_distance_ = 0;
        uint8_t moving_target_energy_ = 0;
        uint16_t stationary_target_distance_ = 0;
        uint8_t stationary_target_energy_ = 0;

        #ifdef USE_FREERTOS
        // Circular buffer for RTOS implementations
        uint8_t circular_buffer[LD2410_BUFFER_SIZE];
        uint16_t buffer_head = 0;
        uint16_t buffer_tail = 0;
        void add_to_buffer(uint8_t byte);
        bool read_from_buffer(uint8_t &byte);
        static void taskFunction(void* param);
        bool read_frame_with_buffer_();
        #endif

        // Common methods
        bool read_frame_no_buffer_();
        bool parse_data_frame_();
        bool parse_command_frame_();
        void print_frame_();
        void send_command_preamble_();
        void send_command_postamble_();
        bool enter_configuration_mode_();
        bool leave_configuration_mode_();
};

#endif