#ifndef CO_DETECTION_H
#define CO_DETECTION_H

#include "main.h"
#include <math.h>

// Co-detection configuration
#define SEISMIC_ACCEL_THRESHOLD    1.5f    // g-force threshold for seismic activity
#define GYRO_MOVEMENT_THRESHOLD    50.0f   // deg/s threshold for rotational movement  
#define GEO_VOLTAGE_THRESHOLD      0.1f    // Minimum geophone voltage to consider
#define CO_DETECTION_TIMEOUT_MS    100     // Time window for multi-sensor correlation

// Co-detection states
typedef enum {
    DETECTION_IDLE = 0,
    DETECTION_GEO_TRIGGERED,
    DETECTION_IMU_CONFIRMED,
    DETECTION_VALID_EVENT,
    DETECTION_FALSE_POSITIVE
} DetectionState_t;

// Event validation result
typedef struct {
    uint8_t isValidEvent;
    float confidence;
    uint32_t timestamp;
    float geoAmplitude;
    float accelMagnitude;
    float gyroMagnitude;
} EventValidation_t;

// Function prototypes
uint8_t CO_IsValidSeismicEvent(float geoValue, float accX, float accY, float accZ, 
                              float gyroX, float gyroY, float gyroZ, uint32_t currentTime);
float CO_CalculateAccelMagnitude(float accX, float accY, float accZ);
float CO_CalculateGyroMagnitude(float gyroX, float gyroY, float gyroZ);
uint8_t CO_CheckSensorCorrelation(float geoValue, float accelMag, float gyroMag);
float CO_CalculateEventConfidence(float geoValue, float accelMag, float gyroMag);
void CO_ResetDetection(void);

#endif /* CO_DETECTION_H */
