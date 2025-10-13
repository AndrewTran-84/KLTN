#include "co_detection.h"

// Co-detection state machine
static DetectionState_t detectionState = DETECTION_IDLE;
static uint32_t geoTriggerTime = 0;
static float lastGeoValue = 0.0f;

uint8_t CO_IsValidSeismicEvent(float geoValue, float accX, float accY, float accZ,
                              float gyroX, float gyroY, float gyroZ, uint32_t currentTime)
{
    float accelMag = CO_CalculateAccelMagnitude(accX, accY, accZ);
    float gyroMag = CO_CalculateGyroMagnitude(gyroX, gyroY, gyroZ);
    
    // State machine for co-detection
    switch(detectionState)
    {
        case DETECTION_IDLE:
            // Wait for geophone trigger
            if(fabs(geoValue) > GEO_VOLTAGE_THRESHOLD)
            {
                detectionState = DETECTION_GEO_TRIGGERED;
                geoTriggerTime = currentTime;
                lastGeoValue = geoValue;
                return 0; // Not confirmed yet
            }
            break;
            
        case DETECTION_GEO_TRIGGERED:
            // Check if IMU confirms within timeout
            if((currentTime - geoTriggerTime) <= CO_DETECTION_TIMEOUT_MS)
            {
                if(accelMag > SEISMIC_ACCEL_THRESHOLD || gyroMag > GYRO_MOVEMENT_THRESHOLD)
                {
                    detectionState = DETECTION_IMU_CONFIRMED;
                    return CO_CheckSensorCorrelation(geoValue, accelMag, gyroMag);
                }
            }
            else
            {
                // Timeout - false positive
                detectionState = DETECTION_FALSE_POSITIVE;
                detectionState = DETECTION_IDLE; // Reset
                return 0;
            }
            break;
            
        case DETECTION_IMU_CONFIRMED:
            // Additional validation if needed
            detectionState = DETECTION_IDLE; // Reset for next event
            return CO_CheckSensorCorrelation(geoValue, accelMag, gyroMag);
            
        default:
            detectionState = DETECTION_IDLE;
            break;
    }
    
    return 0;
}

float CO_CalculateAccelMagnitude(float accX, float accY, float accZ)
{
    // Calculate vector magnitude (remove gravity for seismic detection)
    float magnitude = sqrt(accX * accX + accY * accY + accZ * accZ);
    
    // For seismic events, we're interested in deviations from 1g (gravity)
    float deviation = fabs(magnitude - 1.0f);
    
    return deviation;
}

float CO_CalculateGyroMagnitude(float gyroX, float gyroY, float gyroZ)
{
    // Calculate rotational magnitude
    return sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
}

uint8_t CO_CheckSensorCorrelation(float geoValue, float accelMag, float gyroMag)
{
    // Multi-sensor correlation logic
    uint8_t sensorHits = 0;
    
    // Geophone detection
    if(fabs(geoValue) > GEO_VOLTAGE_THRESHOLD)
        sensorHits++;
    
    // Accelerometer detection  
    if(accelMag > SEISMIC_ACCEL_THRESHOLD)
        sensorHits++;
    
    // Gyroscope detection
    if(gyroMag > GYRO_MOVEMENT_THRESHOLD)
        sensorHits++;
    
    // Require at least 2 sensors to confirm seismic event
    // This reduces false positives from single-sensor noise
    return (sensorHits >= 2);
}

float CO_CalculateEventConfidence(float geoValue, float accelMag, float gyroMag)
{
    float confidence = 0.0f;
    uint8_t sensorCount = 0;
    
    // Normalize sensor readings to confidence scores (0-1)
    if(fabs(geoValue) > GEO_VOLTAGE_THRESHOLD)
    {
        float geoConfidence = fmin(fabs(geoValue) / 2.0f, 1.0f); // Normalize to 2V max
        confidence += geoConfidence;
        sensorCount++;
    }
    
    if(accelMag > SEISMIC_ACCEL_THRESHOLD)
    {
        float accelConfidence = fmin(accelMag / 3.0f, 1.0f); // Normalize to 3g max
        confidence += accelConfidence;
        sensorCount++;
    }
    
    if(gyroMag > GYRO_MOVEMENT_THRESHOLD)
    {
        float gyroConfidence = fmin(gyroMag / 200.0f, 1.0f); // Normalize to 200 deg/s max
        confidence += gyroConfidence;
        sensorCount++;
    }
    
    // Average confidence across active sensors
    if(sensorCount > 0)
        confidence /= sensorCount;
    
    return confidence;
}

// Reset detection state (call after event processing)
void CO_ResetDetection(void)
{
    detectionState = DETECTION_IDLE;
    geoTriggerTime = 0;
    lastGeoValue = 0.0f;
}
