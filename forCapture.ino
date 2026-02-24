#include <smartTrashBin_inferencing.h>
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

// Prototypes
bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
float getDistance(int echoPin);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
    delay(3000);
    Serial.begin(115200);
    Serial.println("___IMAGE COLLECTION SERVER___");

    camera.pinout.aithinker();
    camera.brownout.disable();
    camera.resolution.face(); // 240x240 for Edge Impulse
    camera.quality.high();

    // Init camera
    while (!camera.begin().isOk())
        Serial.println(camera.exception.toString());

    // FIX IMAGE ORIENTATION
    camera.sensor.vflip();    // Use this to fix the vertically opposite view
    camera.sensor.hmirror();  // Use this if it also looks like a mirror reflection

    // Connect to WiFi
    while (!wifi.connect().isOk())
      Serial.println(wifi.exception.toString());

    // Init server
    while (!collectionServer.begin().isOk())
        Serial.println(collectionServer.exception.toString());

    Serial.println("Camera OK. Access at:");
    Serial.println(collectionServer.address());
}

void loop() {
    if (ei_sleep(5) != EI_IMPULSE_OK) return;

    snapshot_buf = (uint8_t*)malloc(320 * 240 * 3);
    if(snapshot_buf == nullptr) return;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        free(snapshot_buf);
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = { 0 };
    if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
        free(snapshot_buf);
        return;
    }

// --- LOGIC SECTION ---
    float distPlastic = getDistance(ECHO_PLASTIC);
    float distPaper = getDistance(ECHO_PAPER);

    Serial.printf("SENSORS: Plastic=%0.1fcm | Paper=%0.1fcm\n", distPlastic, distPaper);

    // Check if either bin is full
    bool plasticFull = (distPlastic > 0.1 && distPlastic <= FULL_DISTANCE); 
    bool paperFull = (distPaper > 0.1 && distPaper <= FULL_DISTANCE);
    bool systemLocked = (plasticFull || paperFull);

    // If any bin is full: BLINK RED, LOCK SERVO, and STOP
    if (systemLocked) {
        Serial.println(">> SYSTEM LOCKED: A bin is full. Empty it to continue.");
        ledcWrite(SERVO_PIN, dutyDefault); // Force Center
        
        // Blink RED LED every 1 second (500ms ON, 500ms OFF)
        digitalWrite(LED_RED, HIGH);
        delay(500);
        digitalWrite(LED_RED, LOW);
        delay(500);

        free(snapshot_buf);
        return; // Skip the rest of the loop (including AI logic)
    }

    // If we reach here, bins are NOT full. Ensure Red LED is OFF.
    digitalWrite(LED_RED, LOW);
    bool objectHandled = false;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value < 0.50) continue;

        if (strstr(bb.label, "plastic") != NULL) {
            Serial.println(">> OPENING: Plastic Bin");
            digitalWrite(LED_GREEN, HIGH);
            ledcWrite(SERVO_PIN, dutyPlastic);
            delay(5000);
            digitalWrite(LED_GREEN, LOW);
            objectHandled = true;
            break; 
        } 
        else if (strstr(bb.label, "paper") != NULL) {
            Serial.println(">> OPENING: Paper Bin");
            digitalWrite(LED_GREEN, HIGH);
            ledcWrite(SERVO_PIN, dutyPaper);
            delay(5000);
            digitalWrite(LED_GREEN, LOW);
            objectHandled = true;
            break;
        }
    }

    if (!objectHandled) {
        ledcWrite(SERVO_PIN, dutyDefault);
    }


    Serial.printf("Distances -> Plastic: %.1f cm, Paper: %.1f cm\n", distPlastic, distPaper);
    ledcWrite(SERVO_PIN, dutyDefault);
    free(snapshot_buf);
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
        .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA, .jpeg_quality = 12, .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

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
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if ((img_width != 320) || (img_height != 240)) {
        ei::image::processing::crop_and_interpolate_rgb888(out_buf, 320, 240, out_buf, img_width, img_height);
    }
    return converted;
}