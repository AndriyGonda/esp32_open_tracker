#pragma once

// OsmAnd device id (must be unique for navigation system like Traccar or M2M)
#define TRACKER_DEVICE_ID "1122334455"


// Max valid hdop for tracker
#define MAX_HDOP 2.1
#define MIN_SATELLITES 6

// Sending intervals
#define TRACKER_INTERVAL_STATIC   120000   // 2 minutes idle
#define TRACKER_INTERVAL_MOVING    10000   // moving
#define TRACKER_INTERVAL_TURNING    2000   // rotation

// Speed threshold in km/h
#define TRACKER_SPEED_THRESHOLD     4.0f
#define TRACKER_MAX_VALID_SPEED    100.0f  

// Bearing threshold in dec
#define TRACKER_BEARING_THRESHOLD   7.0f   


// Blackbox setup 
#define TRACKER_BLACKBOX_PATH       "/blackbox.csv"
#define TRACKER_BLACKBOX_MIN_FREE   51200 



/*HOME POINT FILTERING (Preventing gps spoofing) */
#define ENABLE_HOME_POINT_FILTERING true
#define TRACKER_HOME_LAT        49.439315 
#define TRACKER_HOME_LNG        32.006180 
#define TRACKER_HOME_RADIUS_KM   500.0f 
#define SENDING_LAT 0.0
#define SENDING_LNG 0.0

/*BATTERY MANAGEMENT */
#define BATTERY_LOW_VOLTAGE       3.5f
#define BATTERY_CRITICAL_VOLTAGE  3.0f

// Use home coordinates when battery is low and location is invalid
#define LOW_BATT_HOME_FALLBACK true

// WiFi power management
#define WIFI_TX_POWER             52       // TX_POWER * 0.25  dBm  (formula)
#define WIFI_RETRY_INTERVAL_MS    60000   // 2 min between retries after fail

// WiFi positioning via beacondb.net (requires WiFi connection)
#define ENABLE_WIFI_POSITIONING false

#define ENABLE_PARKING_FILTER         true

// --- Position pinning ---
// Seconds the device must remain stationary before the anchor is set.
#define PARKING_PIN_DELAY_SEC         120

// Maximum drift (meters) allowed while pinned.
#define PARKING_PIN_RADIUS_M          5.0f


// --- Motion consensus (sliding window) ---
// How many consecutive GPS readings to keep in the window.
#define PARKING_WINDOW_SIZE           5


// before the tracker accepts "moving" state.
#define PARKING_MOTION_MIN_MOVING     3
#define PARKING_MAX_ACCELERATION  3.5f 

// ---- IMU (BMI160) motion assist ----
#define ENABLE_IMU                    true

// I2C pins: SDA -> GPIO6, SCL -> GPIO7
#define IMU_SDA_PIN                   6
#define IMU_SCL_PIN                   7

// Dynamic acceleration threshold (m/s²)
// 0.20 – very sensitive (detects engine vibration at idle)
// 0.30 – recommended default
// 0.60 – only strong jolts / clear driving motion
#define IMU_ACCEL_THRESHOLD           0.30f

// Motion fusion strategy:
//   FUSION_AND – moving if GPS AND IMU agree  (most reliable, default)
//   FUSION_OR  – moving if GPS OR  IMU detect motion  (most sensitive)
//   FUSION_IMU – IMU only, ignore GPS speed  (for testing)
#define IMU_FUSION_STRATEGY           FUSION_AND