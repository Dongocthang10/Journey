#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <Arduino.h>
// Logging tag
static const char *TAG = "SMART_HOME";

// GPIO pin definitions
#define HUMI_SENSOR_PIN GPIO_NUM_3
#define TEMP_SENSOR_PIN  GPIO_NUM_4
#define MOTION_SENSOR_PIN    GPIO_NUM_5
#define LIGHT_CONTROL_PIN  GPIO_NUM_12
#define HEATER_CONTROL_PIN  GPIO_NUM_14
#define ALARM_PIN GPIO_NUM_15

// Event group bit definitions
#define TEMP_HIGH_BIT        (1 << 0)
#define MOTION_DETECTED_BIT  (1 << 1)
#define LIGHT_ON_BIT         (1 << 2)
#define SYSTEM_ERROR_BIT     (1 << 3)
#define ALL_SYSTEMS_READY    (1 << 4)
#define HUMI_HIGH_BIT        (1 << 5)

const int PWM_FREQ = 50;
const int PWM_RESOLUTION = 16;
const int PWM_CHANNEL = 0;

// Sensor data structure
typedef struct {
    int sensor_id;
    float value;
    uint32_t timestamp;
} sensor_data_t;

// System command structure
typedef enum command_type {
    CMD_LIGHT_ON,
    CMD_LIGHT_OFF,
    CMD_SERVO_ANGLE,
    CMD_HEATER_ON,
    CMD_HEATER_OFF,
    CMD_ALARM_ON,
    CMD_ALARM_OFF,
    CMD_SYSTEM_CHECK
} command_type_t;

typedef struct {
    command_type_t type;
    float value; // Góc quay (0-180 độ)
  } system_command_t;

// Global handles
QueueHandle_t sensorDataQueue;
QueueHandle_t commandQueue;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t i2cBusSemaphore;
TimerHandle_t systemWatchdogTimer;
TimerHandle_t periodicReportTimer;
EventGroupHandle_t systemEventGroup;

// Global variables protected by mutex
struct {
    float current_temperature;
    float temperature_threshold;
    float current_humidity;
    float humidity_threshold;
    float servo_angle;
    bool motion_detected;
    bool light_status;
    bool heater_status;
    bool alarm_status;
    uint32_t error_count;
} system_state;

// Function prototypes
void temperature_sensor_task(void *pvParameters);
void humidity_sensor_task(void *pvParameters);
void motion_sensor_task(void *pvParameters);
void data_processing_task(void *pvParameters);
void control_task(void *pvParameters);
void display_task(void *pvParameters);
void error_monitor_task(void *pvParameters);
void watchdog_timer_callback(TimerHandle_t xTimer);
void periodic_report_callback(TimerHandle_t xTimer);

uint32_t angle_to_duty(float angle) {
    // 0° = 0.5ms pulse (1638), 180° = 2.5ms pulse (7864)
    return (uint32_t)(1638 + (angle/180.0)*(7864-1638)); 
}

// Hàm map giá trị
float map_value(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}                                                                           

// Simulated sensor readings
float read_temperature_sensor() {
    // Simulate temperature reading between 18-30°C
    return 18.0 + (float)(esp_random() % 120) / 10.0;
}

float read_humidity_sensor() {
    return 40.0 + (float)(esp_random() % 300) / 10.0;
}

bool read_motion_sensor() {
    // Simulate motion detection with 20% probability
    return (esp_random() % 5 == 0);
}

// Initialize GPIO pins
void init_gpio() {
    gpio_config_t io_conf = {};
    
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(HEATER_CONTROL_PIN, PWM_CHANNEL);

    // Configure sensor input pins
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << TEMP_SENSOR_PIN) | (1ULL << MOTION_SENSOR_PIN) | (1ULL << HUMI_SENSOR_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    
    // Configure output pins
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LIGHT_CONTROL_PIN) | (1ULL << HEATER_CONTROL_PIN) | (1ULL << ALARM_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
}


void setup() {
    Serial.begin(9600);
    ESP_LOGI(TAG, "System Initializing...");
  
    // Khởi tạo GPIO
    init_gpio();
  
    // Tạo các queue và semaphore
    sensorDataQueue = xQueueCreate(10, sizeof(sensor_data_t));
    commandQueue = xQueueCreate(5, sizeof(system_command_t));
    dataMutex = xSemaphoreCreateMutex();
    i2cBusSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(i2cBusSemaphore);
  
    // Tạo event group
    systemEventGroup = xEventGroupCreate();
  
    // Khởi tạo trạng thái hệ thống
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      system_state.current_temperature = 22.0;
      system_state.temperature_threshold = 25.0;
      system_state.current_humidity = 44.0;
      system_state.humidity_threshold = 50.0;
      system_state.motion_detected = false;
      system_state.light_status = false;
      system_state.heater_status = false;
      system_state.alarm_status = false;
      system_state.error_count = 0;
      xSemaphoreGive(dataMutex);
    }
  
    // Tạo các task
    xTaskCreatePinnedToCore(
      temperature_sensor_task,  // Task function
      "Temp Sensor",            // Task name
      2048,                     // Stack size
      NULL,                     // Parameters
      3,                        // Priority
      NULL,                     // Task handle
      0                         // Core ID (0 hoặc 1)
    );

    xTaskCreatePinnedToCore(
      humidity_sensor_task,
      "Humi Sensor",
      2048,
      NULL,
      3,
      NULL,
      0
    );

    xTaskCreatePinnedToCore(
      motion_sensor_task,
      "Motion Sensor",
      2048,
      NULL,
      3,
      NULL,
      0
    );
  
    xTaskCreatePinnedToCore(
      data_processing_task,
      "Data Processor",
      4096,
      NULL,
      2,
      NULL,
      1
    );
  
    xTaskCreatePinnedToCore(
      control_task,
      "Control System",
      3072,
      NULL,
      4,
      NULL,
      1
    );
  
    // Tạo và khởi động timer
    systemWatchdogTimer = xTimerCreate(
      "Watchdog",
      pdMS_TO_TICKS(5000),
      pdTRUE,
      NULL,
      watchdog_timer_callback
    );
    xTimerStart(systemWatchdogTimer, 0);
  
    periodicReportTimer = xTimerCreate(
      "Reporter",
      pdMS_TO_TICKS(30000),
      pdTRUE,
      NULL,
      periodic_report_callback
    );
    xTimerStart(periodicReportTimer, 0);
  
    ESP_LOGI(TAG, "Initialization Complete");
    Serial.println("System Ready!");
  }

void loop() {
    static unsigned long last_display = 0;
    if (millis() - last_display > 10000) {
      last_display = millis();
      
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
        Serial.printf("[MAIN LOOP] Temp: %.1fC | Humi: %.1fF | Motion: %d | Lights: %d\n",
                     system_state.current_temperature,
                     system_state.current_humidity,
                     system_state.motion_detected,
                     system_state.light_status);
        xSemaphoreGive(dataMutex);
      }
    }
    
    
    // Giữ CPU không bận rộn 100%
    vTaskDelay(pdMS_TO_TICKS(100));
}


// Temperature sensor task - reads temperature and sends it to the queue
void temperature_sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;
    sensor_data.sensor_id = 1; // Temperature sensor ID
    
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Read temperature sensor
        sensor_data.value = read_temperature_sensor();
        Serial.printf("[TEMP] Raw reading: %.1f°C\n", sensor_data.value);
        sensor_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Send data to queue for processing
        if (xQueueSend(sensorDataQueue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW(TAG, "Temperature sensor queue full, data discarded");
            system_state.error_count++;
        }
        
        // Update local system state with mutex protection
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            system_state.current_temperature = sensor_data.value;
            
            // Set event bit if temperature is above threshold
            if (system_state.current_temperature > system_state.temperature_threshold) {
                xEventGroupSetBits(systemEventGroup, TEMP_HIGH_BIT);
            } else {
                xEventGroupClearBits(systemEventGroup, TEMP_HIGH_BIT);
            }
            
            xSemaphoreGive(dataMutex);
        }
        
        // Task runs every 2 seconds
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2000));
    }
}

void humidity_sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;
    sensor_data.sensor_id = 3;
    
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        sensor_data.value = read_humidity_sensor();
        Serial.printf("[HUMI] Raw reading: %.1f°F\n", sensor_data.value);
        sensor_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (xQueueSend(sensorDataQueue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS){
            ESP_LOGW(TAG, "Humidity sensor queue full, data discarded");
            system_state.error_count++;
        }

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE){
            system_state.current_humidity = sensor_data.value;

            if (system_state.current_humidity > system_state.humidity_threshold){
                xEventGroupSetBits(systemEventGroup, HUMI_HIGH_BIT);
            } else {
                xEventGroupClearBits(systemEventGroup, HUMI_HIGH_BIT);
            }

            xSemaphoreGive(dataMutex);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2000));
    }
}

// Motion sensor task - detects motion and signals through event group
void motion_sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;
    sensor_data.sensor_id = 2; // Motion sensor ID
    
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Read motion sensor
        bool motion = read_motion_sensor();
        Serial.printf("[MOTION] Status: %s\n", motion ? "DETECTED" : "NO MOTION");
        sensor_data.value = motion ? 1.0 : 0.0;
        sensor_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Send data to queue for processing
        if (xQueueSend(sensorDataQueue, &sensor_data, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGW(TAG, "Motion sensor queue full, data discarded");
        }
        
        // Update system state with mutex protection
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            system_state.motion_detected = motion;
            xSemaphoreGive(dataMutex);
        }
        
        // Use event group to signal motion detection
        if (motion) {
            xEventGroupSetBits(systemEventGroup, MOTION_DETECTED_BIT);
            ESP_LOGI(TAG, "Motion detected!");
            
            // Simulate accessing a shared resource (I2C bus)
            if (xSemaphoreTake(i2cBusSemaphore, pdMS_TO_TICKS(200)) == pdTRUE) {
                // Simulate I2C communication to log motion event
                vTaskDelay(pdMS_TO_TICKS(50)); // Simulating I2C operation time
                xSemaphoreGive(i2cBusSemaphore);
            } else {
                ESP_LOGW(TAG, "Failed to access I2C bus for motion logging");
            }
            
            // Send command to turn on lights when motion detected
            system_command_t cmd;
            cmd.type = CMD_LIGHT_ON;
            cmd.value = 0;
            xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
        } else {
            xEventGroupClearBits(systemEventGroup, MOTION_DETECTED_BIT);
        }
        
        // Task runs every second
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

// Data processing task - processes sensor data from the queue
void data_processing_task(void *pvParameters) {
    sensor_data_t received_data;
    float temp_readings[10] = {0}; // Ring buffer for temperature readings
    int temp_index = 0;
    float temp_avg = 0;
    float humi_readings[10] = {0};
    int humi_index = 0;
    float humi_avg = 0;
    
    while (1) {
        // Wait for data from sensors
        if (xQueueReceive(sensorDataQueue, &received_data, pdMS_TO_TICKS(5000)) == pdTRUE) {
            
            // Process based on sensor type
            switch (received_data.sensor_id) {
                case 1: // Temperature data
                    // Store in ring buffer for averaging
                    Serial.printf("[PROCESS] Temperature processed: %.1f°C (Avg: %.1f°C)\n", received_data.value, temp_avg);
                    temp_readings[temp_index] = received_data.value;
                    temp_index = (temp_index + 1) % 10;
                    
                    // Calculate average temperature from last 10 readings
                    temp_avg = 0;
                    for (int i = 0; i < 10; i++) {
                        temp_avg += temp_readings[i];
                    }
                    temp_avg /= 10.0;
                    Serial.printf("[PROCESS] Temperature processed: %.1f°C (Avg: %.1f°C)\n", received_data.value, temp_avg);
                    ESP_LOGI(TAG, "Processed temperature: %.1f°C (Avg: %.1f°C)", 
                             received_data.value, temp_avg);
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Ánh xạ nhiệt độ 18-30°C -> góc servo 0-180°
                    float servo_angle = map_value(system_state.current_temperature, 
                                                18.0, 30.0, 
                                                0.0, 180.0);
                    
                    // Tạo và gửi lệnh servo
                    system_command_t servo_cmd;
                    servo_cmd.type = CMD_SERVO_ANGLE;
                    servo_cmd.value = constrain(servo_angle, 0.0, 180.0);
                    xQueueSend(commandQueue, &servo_cmd, 0);
                    
                    xSemaphoreGive(dataMutex);
                }
                    
                    // Check if heater control is needed
                    // if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    //     if (temp_avg < 22.0 && !system_state.heater_status) {
                    //         system_command_t cmd = CMD_HEATER_ON;
                    //         xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
                    //     } else if (temp_avg > 24.0 && system_state.heater_status) {
                    //         system_command_t cmd = CMD_HEATER_OFF;
                    //         xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
                    //     }
                    //     xSemaphoreGive(dataMutex);
                    // }
                    break;
                    
                case 2: // Motion data
                    ESP_LOGD(TAG, "Processed motion data: %s", 
                             received_data.value > 0.5 ? "Movement" : "No movement");
                    break;
                case 3:
                    // Store in ring buffer for averaging
                    humi_readings[humi_index] = received_data.value;
                    humi_index = (humi_index + 1) % 10;
                    
                    // Calculate average temperature from last 10 readings
                    humi_avg = 0;
                    for (int i = 0; i < 10; i++) {
                        humi_avg += humi_readings[i];
                    }
                    humi_avg /= 10.0;
            
                    Serial.printf("[PROCESS] Humidity processed: %.1f%% (Avg: %.1f%%)\n", received_data.value, humi_avg);
            
                    ESP_LOGI(TAG, "Processed humidity: %.1f%% (Avg: %.1f%%)", received_data.value, humi_avg);
                    
                    // Check if heater control is needed
                    // if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    //     if (humi_avg < 42.0 && !system_state.heater_status) {
                    //         system_command_t cmd = CMD_HEATER_ON;
                    //         xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
                    //     } else if (temp_avg > 44.0 && system_state.heater_status) {
                    //         system_command_t cmd = CMD_HEATER_OFF;
                    //         xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
                    //     }
                    //     xSemaphoreGive(dataMutex);
                    // }
                    break;

                default:
                    ESP_LOGW(TAG, "Unknown sensor ID: %d", received_data.sensor_id);
                    break;
            }

            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (temp_avg < 22.0 && humi_avg < 40.0 && !system_state.heater_status) {
                    system_command_t cmd;
                    cmd.type = CMD_HEATER_ON;
                    cmd.value = 0;
                    xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
                } else if ((temp_avg > 24.0 || humi_avg > 50.0) && system_state.heater_status) {
                    system_command_t cmd;
                    cmd.type = CMD_HEATER_OFF;
                    cmd.value = 0;
                    xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
                }
                xSemaphoreGive(dataMutex);
            }
            
            // Simulate high CPU load for processing
            uint32_t start_time = xTaskGetTickCount();
            uint32_t random_cycles = 10000 + (esp_random() % 20000);
            volatile uint32_t counter = 0;
            for (uint32_t i = 0; i < random_cycles; i++) {
                counter++;
            }
            
            // Check if processing took too long
            uint32_t processing_time = xTaskGetTickCount() - start_time;
            if (processing_time > pdMS_TO_TICKS(100)) {
                ESP_LOGW(TAG, "Processing took too long: %d ms", 
                         (int)(processing_time * portTICK_PERIOD_MS));
                
                // Signal system error
                xEventGroupSetBits(systemEventGroup, SYSTEM_ERROR_BIT);
            }
        } else {
            // No data received in timeout period
            ESP_LOGW(TAG, "No sensor data received for 5 seconds");
        }
    }
}

// Control task - handles system commands from the queue
void control_task(void *pvParameters) {
    system_command_t command;
    EventBits_t event_bits;
    bool system_check_required = false;
    
    while (1) {
        // Check for commands with a short timeout
        if (xQueueReceive(commandQueue, &command, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Take mutex before changing system state
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                switch (command.type) {
                    case CMD_SERVO_ANGLE:
                        Serial.println("[SERVO] Rotating");
                        ledcWrite(PWM_CHANNEL, angle_to_duty(command.value));
                        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100))) {
                            system_state.servo_angle = command.value;
                            xSemaphoreGive(dataMutex);
                        }
                        break;
                    case CMD_LIGHT_ON:
                        Serial.println("[CTRL] Turning lights ON");
                        system_state.light_status = true;
                        gpio_set_level(LIGHT_CONTROL_PIN, 1);
                        xEventGroupSetBits(systemEventGroup, LIGHT_ON_BIT);
                        ESP_LOGI(TAG, "Lights turned ON");
                        break;
                        
                    case CMD_LIGHT_OFF:
                        system_state.light_status = false;
                        Serial.println("[CTRL] Turning lights OFF");
                        gpio_set_level(LIGHT_CONTROL_PIN, 0);
                        xEventGroupClearBits(systemEventGroup, LIGHT_ON_BIT);
                        ESP_LOGI(TAG, "Lights turned OFF");
                        break;
                        
                    case CMD_HEATER_ON:
                        Serial.println("[CTRL] Turning heater ON");
                        system_state.heater_status = true;
                        gpio_set_level(HEATER_CONTROL_PIN, 1);
                        ESP_LOGI(TAG, "Heater turned ON");
                        break;
                        
                    case CMD_HEATER_OFF:
                        Serial.println("[CTRL] Turning heater OFF");
                        system_state.heater_status = false;
                        gpio_set_level(HEATER_CONTROL_PIN, 0);
                        ESP_LOGI(TAG, "Heater turned OFF");
                        break;
                        
                    case CMD_ALARM_ON:
                        Serial.println("[CTRL] Activating alarm");
                        system_state.alarm_status = true;
                        gpio_set_level(ALARM_PIN, 1);
                        ESP_LOGW(TAG, "ALARM ACTIVATED!");
                        break;
                        
                    case CMD_ALARM_OFF:
                        Serial.println("[CTRL] Deactivating alarm");
                        system_state.alarm_status = false;
                        gpio_set_level(ALARM_PIN, 0);
                        ESP_LOGI(TAG, "Alarm deactivated");
                        break;
                        
                    case CMD_SYSTEM_CHECK:
                        system_check_required = true;
                        ESP_LOGI(TAG, "System check requested");
                        break;
                }
                xSemaphoreGive(dataMutex);
            } else {
                ESP_LOGE(TAG, "Failed to obtain mutex for control operation!");
                xEventGroupSetBits(systemEventGroup, SYSTEM_ERROR_BIT);
            }
        }
        
        // Check event flags
        event_bits = xEventGroupGetBits(systemEventGroup);
        
        // Temperature high and motion detected simultaneously - possible fire!
        if ((event_bits & (TEMP_HIGH_BIT | MOTION_DETECTED_BIT | HUMI_HIGH_BIT)) == 
            (TEMP_HIGH_BIT | MOTION_DETECTED_BIT | HUMI_HIGH_BIT)) {
            system_command_t alarm_cmd;
            alarm_cmd.type = CMD_ALARM_ON;
            alarm_cmd.value = 0;   
            xQueueSendToFront(commandQueue, &alarm_cmd, 0); // Priority send to front of queue
            ESP_LOGW(TAG, "HIGH TEMP + MOTION DETECTED: Possible fire!");
        }
        
        // Handle system check if requested
        if (system_check_required) {
            // Simulate accessing a shared resource (I2C bus) for system diagnostics
            if (xSemaphoreTake(i2cBusSemaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
                ESP_LOGI(TAG, "Running system diagnostics...");
                
                // Simulate I2C communication with various peripherals
                vTaskDelay(pdMS_TO_TICKS(200));
                
                // Check system parameters with mutex protection
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    ESP_LOGI(TAG, "System Status:");
                    ESP_LOGI(TAG, "  Temperature: %.1f°C (Threshold: %.1f°C)", 
                             system_state.current_temperature, system_state.temperature_threshold);
                    ESP_LOGI(TAG, "  Humidity: %.1f°F (Threshold: %.1f°F)", 
                             system_state.current_humidity, system_state.humidity_threshold);                             
                    ESP_LOGI(TAG, "  Motion: %s", system_state.motion_detected ? "Detected" : "None");
                    ESP_LOGI(TAG, "  Lights: %s", system_state.light_status ? "ON" : "OFF");
                    ESP_LOGI(TAG, "  Heater: %s", system_state.heater_status ? "ON" : "OFF");
                    ESP_LOGI(TAG, "  Alarm: %s", system_state.alarm_status ? "ACTIVE" : "Inactive");
                    ESP_LOGI(TAG, "  Error count: %d", system_state.error_count);
                    
                    // Reset error count after checking
                    system_state.error_count = 0;
                    xSemaphoreGive(dataMutex);
                }
                
                xSemaphoreGive(i2cBusSemaphore);
                system_check_required = false;
            } else {
                ESP_LOGW(TAG, "Could not access I2C bus for system check");
            }
        }
        
        // Task housekeeping delay
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Display task - updates UI and reports system status
void display_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    EventBits_t current_events;
    char status_buffer[128];
    
    // Wait for all systems to be ready
    xEventGroupWaitBits(
        systemEventGroup,
        ALL_SYSTEMS_READY,
        pdFALSE,    // Don't clear the bits
        pdTRUE,     // Wait for all bits
        portMAX_DELAY
    );
    
    while (1) {
        // Get current event bits
        current_events = xEventGroupGetBits(systemEventGroup);
        
        // Format status message based on system state
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            snprintf(status_buffer, sizeof(status_buffer), 
                     "Status: Temp=%.1f°C, Humi=%.1f°F, Motion=%s, Lights=%s, Heater=%s, Alarm=%s",
                     system_state.current_temperature,
                     system_state.current_humidity,
                     system_state.motion_detected ? "YES" : "NO",
                     system_state.light_status ? "ON" : "OFF",
                     system_state.heater_status ? "ON" : "OFF",
                     system_state.alarm_status ? "ACTIVE" : "Inactive");
            xSemaphoreGive(dataMutex);
        } else {
            snprintf(status_buffer, sizeof(status_buffer), "Status: Could not access system state");
        }
        
        // Display status message
        ESP_LOGI(TAG, "Display: %s", status_buffer);
        
        // Check for system errors
        if (current_events & SYSTEM_ERROR_BIT) {
            ESP_LOGW(TAG, "Display: SYSTEM ERROR DETECTED!");
        }
        
        // Task runs every 5 seconds
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000));
    }
}

// Error monitor task - handles system errors
void error_monitor_task(void *pvParameters) {
    EventBits_t event_bits;
    uint8_t consecutive_errors = 0;
    
    while (1) {
        // Wait for system error bit to be set
        event_bits = xEventGroupWaitBits(
            systemEventGroup,
            SYSTEM_ERROR_BIT,
            pdTRUE,         // Clear the bits after reading
            pdFALSE,        // Any bit will do
            pdMS_TO_TICKS(5000)
        );
        
        if (event_bits & SYSTEM_ERROR_BIT) {
            Serial.printf("[ERROR] System error detected! Count: %d\n", consecutive_errors);
            consecutive_errors++;
            ESP_LOGE(TAG, "System error detected (count: %d)", consecutive_errors);
            
            // If we have too many consecutive errors, try to recover the system
            if (consecutive_errors >= 3) {
                ESP_LOGE(TAG, "Multiple errors detected, attempting system recovery");
                
                // Take mutex to ensure exclusive access to system state
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
                    // Reset all outputs to safe state
                    system_state.light_status = false;
                    gpio_set_level(LIGHT_CONTROL_PIN, 0);
                    
                    system_state.heater_status = false;
                    gpio_set_level(HEATER_CONTROL_PIN, 0);
                    
                    if (system_state.alarm_status) {
                        // Keep alarm on if it was already on
                        ESP_LOGW(TAG, "Keeping alarm ON during recovery");
                    } else {
                        system_state.alarm_status = false;
                        gpio_set_level(ALARM_PIN, 0);
                    }
                    
                    // Reset error counters
                    system_state.error_count = 0;
                    consecutive_errors = 0;
                    
                    xSemaphoreGive(dataMutex);
                    
                    // Request a system check
                    system_command_t cmd;
                    cmd.value = 0;
                    cmd.type = CMD_SYSTEM_CHECK;
                    xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100));
                    
                    ESP_LOGI(TAG, "System recovery complete");
                } else {
                    ESP_LOGE(TAG, "Could not obtain mutex for system recovery!");
                    // Emergency recovery without mutex
                    gpio_set_level(LIGHT_CONTROL_PIN, 0);
                    gpio_set_level(HEATER_CONTROL_PIN, 0);
                }
            }
        } else {
            // No errors detected, decrement error count if positive
            if (consecutive_errors > 0) {
                consecutive_errors--;
            }
        }
        
        // Task housekeeping
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Timer callback functions
void watchdog_timer_callback(TimerHandle_t xTimer) {
    Serial.println("[WATCHDOG] Timer tick");
    static uint32_t watchdog_counter = 0;
    watchdog_counter++;
    
    // Check if a system check is needed periodically
    if (watchdog_counter % 6 == 0) { // Every 30 seconds
        ESP_LOGI(TAG, "Watchdog timer: Requesting system check");
        system_command_t cmd;
        cmd.value = 0;
        cmd.type = CMD_SYSTEM_CHECK;
        xQueueSend(commandQueue, &cmd, 0);
    }
    
    // Check if motion detected but lights are off for too long
    EventBits_t bits = xEventGroupGetBits(systemEventGroup);
    if ((bits & MOTION_DETECTED_BIT) && !(bits & LIGHT_ON_BIT)) {
        // Motion detected but lights are off
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (system_state.motion_detected && !system_state.light_status) {
                ESP_LOGW(TAG, "Watchdog: Motion detected but lights are off, correcting");
                system_command_t cmd;
                cmd.value = 0;
                cmd.type = CMD_LIGHT_ON;
                xQueueSend(commandQueue, &cmd, 0);
            }
            xSemaphoreGive(dataMutex);
        }
    }
}

void periodic_report_callback(TimerHandle_t xTimer) {
    Serial.println("[REPORT] Generating periodic report");
    ESP_LOGI(TAG, "Generating periodic report...");
    
    // Get system stats
    UBaseType_t uxHighWaterMark;
    
    // Take mutex to safely access system state
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        ESP_LOGI(TAG, "PERIODIC REPORT:");
        ESP_LOGI(TAG, "  Temperature: %.1f°C", system_state.current_temperature);
        ESP_LOGI(TAG, "  Light status: %s", system_state.light_status ? "ON" : "OFF");
        ESP_LOGI(TAG, "  Heater status: %s", system_state.heater_status ? "ON" : "OFF");
        ESP_LOGI(TAG, "  Errors since last report: %d", system_state.error_count);
        
        // Reset error count
        system_state.error_count = 0;
        xSemaphoreGive(dataMutex);
    }
    
    // Report free heap memory
    ESP_LOGI(TAG, "  System free heap: %d bytes", esp_get_free_heap_size());
    
    // Report queue status
    ESP_LOGI(TAG, "  Sensor queue: %d/%d items",
             uxQueueMessagesWaiting(sensorDataQueue),
             uxQueueSpacesAvailable(sensorDataQueue));
    
    ESP_LOGI(TAG, "  Command queue: %d/%d items",
             uxQueueMessagesWaiting(commandQueue),
             uxQueueSpacesAvailable(commandQueue));
}