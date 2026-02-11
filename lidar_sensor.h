#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Arduino.h>

// Car presence states detected by LiDAR
enum LidarState { LIDAR_NO_CAR, LIDAR_CAR_STAGED, LIDAR_CAR_LAUNCHED };

// Initialize Benewake TF-Luna LiDAR on UART. No-op if LiDAR not enabled in config.
void lidarSetup();

// Poll sensor and update state machine. Non-blocking (10Hz via millis timer).
// Broadcasts state changes to WebSocket clients.
void lidarLoop();

// Returns true if a car is currently detected at the sensor.
bool isCarPresent();

// Get raw distance reading in millimeters (0 = no reading).
uint16_t getDistanceMM();

// Get current LiDAR state.
LidarState getLidarState();

// Get LiDAR data as JSON string for WebSocket broadcast.
String getLidarJson();

// Returns true ONCE when car has been staged for >1 second (for auto-arm).
// Safe to call even if LiDAR is disabled — returns false.
bool lidarAutoArmReady();

#endif
