#include <plasticOnly_inferencing.h>



// #include <sampleDataSet_inferencing.h>


// #include <.h>
// #include <smartTrashBin_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <Arduino.h>
#include "esp_camera.h"


// ===== CONFIGURATION =====
#define SERVO_PIN 12      
#define SERVO_FREQ 50     
#define SERVO_RESOLUTION 16 

#define TRIG_PIN 13
#define ECHO_PLASTIC 15
#define ECHO_PAPER 14
#define FULL_DISTANCE 15 

// LED Pins
#define LED_RED 2    // Full Indicator
#define LED_GREEN 4  // Open Indicator (Built-in Flash)

// Camera Model
#define CAMERA_MODEL_AI_THINKER 
#include "esp_camera.h" 

// Updated Servo Duty Cycles (16-bit)
uint32_t dutyPlastic  = 1638;  // 0 degrees
uint32_t dutyDefault  = 3440;  // 50 degrees
uint32_t dutyPaper    = 5242;  // 100 degrees

static bool is_initialised = false;
uint8_t *snapshot_buf;

//
unsigned long lastDetectionTime = 0;
const unsigned long detectionCooldown = 3000; // 7 seconds
bool detectionLocked = false;

//

// Prototypes
bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
float getDistance(int echoPin);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
    Serial.begin(115200);
    
    // 1. Setup Servo
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RESOLUTION);
    ledcWrite(SERVO_PIN, dutyDefault);

    // 2. Setup Ultrasonic & LED Pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PLASTIC, INPUT);
    pinMode(ECHO_PAPER, INPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    // Initial LED Test (Blink twice)
    for(int i=0; i<2; i++){
      digitalWrite(LED_RED, HIGH); digitalWrite(LED_GREEN, HIGH);
      delay(200);
      digitalWrite(LED_RED, LOW); digitalWrite(LED_GREEN, LOW);
      delay(200);
    }

    // 3. Setup Camera
    if (ei_camera_init() == false) Serial.println("Camera Init Failed");
}

// void loop() {
    
//     // A. LOCKOUT: If we are in the cooldown period, do absolutely nothing.
//     if (detectionLocked) {
//         if (millis() - lastDetectionTime < detectionCooldown) {
//             // Optional: Blink Green LED slowly while waiting
//             digitalWrite(LED_GREEN, (millis() / 500) % 2);
//             return; 
//         } else {
//             detectionLocked = false;
//             digitalWrite(LED_GREEN, LOW);
//             Serial.println(">> READY: Waiting for trash...");
//         }
//     }

//     // B. FULL BIN CHECK (Blinking Red)
//     float distPlastic = getDistance(ECHO_PLASTIC);
//     float distPaper = getDistance(ECHO_PAPER);
//     if ((distPlastic > 0.1 && distPlastic <= FULL_DISTANCE) || (distPaper > 0.1 && distPaper <= FULL_DISTANCE)) {
//         digitalWrite(LED_RED, (millis() / 800) % 2 == 0 ? HIGH : LOW); // Blink RED
//         ledcWrite(SERVO_PIN, dutyDefault);
//         return; // Don't even turn on the camera if it's full
//     }
//     digitalWrite(LED_RED, LOW);

//     // C. IMAGE CAPTURE
//     snapshot_buf = (uint8_t*)malloc(320 * 240 * 3);
//     if(snapshot_buf == nullptr) return;

//     if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
//         free(snapshot_buf);
//         return;
//     }

//     ei::signal_t signal;
//     signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
//     signal.get_data = &ei_camera_get_data;

//     ei_impulse_result_t result = { 0 };
//     if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
//         free(snapshot_buf);
//         return;
//     }

// // D. DETECTION LOGIC
//     for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
//         ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        
//         if (bb.value < 0.60) continue; 

//         // Convert label check to be more flexible
//         String label = String(bb.label);
//         label.toLowerCase(); // Convert to lowercase to make it easier to match

//         if (label.indexOf("plastic") >= 0) {
//             Serial.println(">> ACTION: Moving to PLASTIC");
//             ledcWrite(SERVO_PIN, dutyPlastic);
//         } 
//         else if (label.indexOf("paper") >= 0) {
//             Serial.println(">> ACTION: Moving to PAPER");
//             ledcWrite(SERVO_PIN, dutyPaper);
//         }
//         else {
//             continue; // Not an object we care about
//         }
        
//         digitalWrite(LED_GREEN, HIGH);
//         delay(5000); 
        
//         ledcWrite(SERVO_PIN, dutyDefault); 
//         digitalWrite(LED_GREEN, LOW);
        
//         lastDetectionTime = millis();
//         detectionLocked = true;
//         break; 
//     } recent Idea
    // // D. DETECTION LOGIC
    // for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    //     ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        
    //     // HIGHER THRESHOLD: 0.85 means it must be very sure!
    //     if (bb.value < 1) continue; 

    //     bool isPlastic = (strstr(bb.label, "plastic") != NULL);
    //     bool isPaper = (strstr(bb.label, "paper") != NULL);

    //     if (isPlastic || isPaper) {
    //         Serial.printf(">> DETECTED: %s (%.2f)\n", bb.label, bb.value);
            
    //         // 1. Move Servo
    //         if (isPlastic) ledcWrite(SERVO_PIN, dutyPlastic);
    //         else ledcWrite(SERVO_PIN, dutyPaper);
            
    //         // 2. Wait for trash to fall (Increase to 5s if it's too fast)
    //         digitalWrite(LED_GREEN, HIGH);
    //         delay(5000); 
            
    //         // 3. Close the lid
    //         ledcWrite(SERVO_PIN, dutyDefault); 
    //         digitalWrite(LED_GREEN, LOW);
            
    //         // 4. LOCK: Set a 5-second cooldown AFTER the lid closes
    //         lastDetectionTime = millis();
    //         detectionLocked = true;
    //         break; 
    //     }

    // new idea, for plastic only //
  
void loop() {
    // 1. TRIGGER: ONLY start if something is actually on the tray
    float triggerDist = getDistance(ECHO_PLASTIC);
    
    // If tray is empty, just keep waiting
    if (triggerDist > 30.0 || triggerDist < 1.0) {
        detectionLocked = false; // Reset lock when tray is empty
        return; 
    }

    // 2. REPEAT PREVENTION: If we just finished a detection, 
    // do not start a new one until the user removes the item or it falls.
    if (detectionLocked) {
        // If it's been less than 3 seconds OR the object is still there, stay locked
        if (millis() - lastDetectionTime < detectionCooldown || triggerDist <= 30.0) {
            return; 
        } else {
            detectionLocked = false;
            Serial.println(">> READY: Tray is clear. Waiting for next item...");
        }
    }

    // 3. FULL BIN CHECK
// 3. FULL BIN CHECK – Blink red LED if any bin is full
float distPlastic = getDistance(ECHO_PLASTIC);
float distPaper = getDistance(ECHO_PAPER);
if ((distPlastic > 0.1 && distPlastic <= FULL_DISTANCE) || (distPaper > 0.1 && distPaper <= FULL_DISTANCE)) {
    // Non‑blocking blink every 500 ms (1 second period)
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    if (millis() - lastBlink >= 500) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_RED, ledState);
    }
    // Ensure lid is closed while bin is full
    ledcWrite(SERVO_PIN, dutyDefault);
    return; // Exit loop – do nothing else
} else {
    digitalWrite(LED_RED, LOW); // Turn LED off when not full
}

    // 4. IMAGE CAPTURE
    snapshot_buf = (uint8_t*)malloc(320 * 240 * 3);
    if(snapshot_buf == nullptr) return;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        free(snapshot_buf);
        return;
    }

    // 5. AI CLASSIFICATION
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = { 0 };
    if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
        free(snapshot_buf);
        return;
    }

    // // 6. DETECTION LOGIC
    // bool foundPlastic = false;
    // for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    //     ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    //     if (bb.value >= 0.50 && strstr(bb.label, "plastic") != NULL) {
    //         foundPlastic = true;
    //         break; 
    //     }
    // }

    // // 7. FINAL SORTING ACTION
    // if (foundPlastic) {
    //     Serial.println(">> RESULT: PLASTIC DETECTED");
    //     ledcWrite(SERVO_PIN, dutyPlastic);
    // } else {
    //     Serial.println(">> RESULT: NOT PLASTIC (Moving to Paper Bin)");
    //     ledcWrite(SERVO_PIN, dutyPaper);
    // }
    // Check if ANY object was actually found

// 6. DETECTION LOGIC: Lowered to 0.40 to be more sensitive
bool foundPlastic = false;
bool foundPaper = false;

for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    
    // Lowered threshold from 0.60 to 0.40 because your model is struggling with background
    if (bb.value >= 0.40) { 
        if (strstr(bb.label, "plastic") != NULL) foundPlastic = true;
        else if (strstr(bb.label, "paper") != NULL) foundPaper = true;
    }
}

// 7. FINAL ACTION
if (foundPlastic || foundPaper) {
    if (foundPlastic) {
        Serial.println(">> DECISION: PLASTIC");
        ledcWrite(SERVO_PIN, dutyPlastic);
        delay(3000);
    } else {
        Serial.println(">> DECISION: PAPER");
        ledcWrite(SERVO_PIN, dutyPaper);
        delay(3000);
    }
    // ... rest of the movement code ...
} else {
    // If the sensor is triggered but AI sees nothing, we need to know why
    Serial.println(">> AI failed to find a label. Check lighting/contrast.");
}

    digitalWrite(LED_GREEN, LOW);
    delay(5000); // Time for trash to fall

    // 8. RESET AND LOCK
    ledcWrite(SERVO_PIN, dutyDefault);
    digitalWrite(LED_GREEN, LOW);
    
    lastDetectionTime = millis();
    detectionLocked = true; // This prevents immediate re-triggering
    
    free(snapshot_buf);
    Serial.println(">> ACTION COMPLETE: Please clear the tray to continue.");
} 

/* --- HELPER FUNCTIONS --- */

float getDistance(int echoPin) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(echoPin, HIGH, 25000); 
    if (duration == 0) return 999;
    return duration * 0.034 / 2;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++; pixel_ix+=3; pixels_left--;
    }
    return 0;
}

bool ei_camera_init(void) {
    if (is_initialised) return true;

    // We define the config here so the function definitely sees it
    static camera_config_t config = {
        .pin_pwdn = 32, .pin_reset = -1, .pin_xclk = 0, .pin_sscb_sda = 26, .pin_sscb_scl = 27,
        .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36, .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
        .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,
        .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565, .frame_size = FRAMESIZE_QVGA, .jpeg_quality = 12, .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

//     static camera_config_t config = {
//         .pin_pwdn = 32, .pin_reset = -1, .pin_xclk = 0, .pin_sscb_sda = 26, .pin_sscb_scl = 27,
//         .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36, .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
//         .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,
//         .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
//         .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA, .jpeg_quality = 12, .fb_count = 1,
//         .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
//     };
// //
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    is_initialised = true;
    return true;

}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, snapshot_buf);
    // bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if ((img_width != 320) || (img_height != 240)) {
        ei::image::processing::crop_and_interpolate_rgb888(out_buf, 320, 240, out_buf, img_width, img_height);
    }
    return converted;
}