#pragma once

// OsmAnd device id (must be unique for navigation system like Traccar or M2M)
#define TRACKER_DEVICE_ID "1122334455"


// Max valid hdop for tracker
#define MAX_HDOP 10.0

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


// WiFi power management
#define WIFI_TX_POWER             52       // TX_POWER * 0.25  dBm 
#define WIFI_MOVING_BATCH_SIZE    10       // max points in moving batch
#define WIFI_RETRY_INTERVAL_MS    60000   // 2 min between retries after fail

// WiFi positioning via beacondb.net (requires WiFi connection)
#define ENABLE_WIFI_POSITIONING false