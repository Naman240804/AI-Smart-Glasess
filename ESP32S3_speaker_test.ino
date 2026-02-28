#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include "esp_camera.h"
#include "HTTPClient.h"
#include "base64.h"
#include "ArduinoJson.h"
#include "WebServer.h"
#include "ESPmDNS.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#include "ESP_I2S.h"
#define USE_ESP_I2S_V3
#else
#include "I2S.h"
#endif

// PDM Digital Microphone (XIAO ESP32S3 Sense) - from pinout
#define MIC_PDM_CLK   42   // PDM clock
#define MIC_PDM_DATA  41   // PDM data
#define RECORD_SECONDS 4
#define SAMPLE_RATE   16000
#define RECORD_GAIN   3    // 1=no change, 2=2x, 3=3x louder (clipped to avoid distortion)

// Google Cloud Speech-to-Text - enable "Cloud Speech-to-Text API" in your Google Cloud project
const char* STT_API_URL = "https://speech.googleapis.com/v1/speech:recognize";

// Hardware Pins for XIAO S3 Sense
// Using available pins that don't conflict with camera:
// Camera uses: Y3=GPIO17, Y4=GPIO18, Y5=GPIO16, Y6=GPIO14, Y7=GPIO12, Y8=GPIO11, HREF=GPIO47, VSYNC=GPIO38, PCLK=GPIO13
// Available: GPIO1(D0), GPIO3(D2), GPIO44(D7/RX) - no conflicts with camera
#define I2S_LRC       1   // D0 - GPIO1 (TOUCH1, A0) - Available
#define I2S_BCLK      3   // D2 - GPIO3 (TOUCH3, A2) - Available
#define I2S_DOUT      44  // D7 - GPIO44 (RX) - Available, can be used for I2S

// TTP223 Touch Sensor Pin
#define TOUCH_SENSOR_PIN  2  // D1 - GPIO2 (TOUCH2)

// Camera pins for XIAO ESP32S3 Sense (OV2640) - Official configuration
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48  // Fixed: was 47
#define Y8_GPIO_NUM       11  // Fixed: was 21
#define Y7_GPIO_NUM       12  // Fixed: was 14
#define Y6_GPIO_NUM       14  // Fixed: was 4
#define Y5_GPIO_NUM       16  // Fixed: was 5
#define Y4_GPIO_NUM       18  // Fixed: was 6
#define Y3_GPIO_NUM       17  // Fixed: was 7 (this was conflicting with I2S!)
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38  // Fixed: was 48
#define HREF_GPIO_NUM     47  // Fixed: was 41
#define PCLK_GPIO_NUM     13  // Fixed: was 11

// Configuration
const char* ssid = "S24 FE";
const char* password = "123456789";
const char* GEMINI_API_KEY = "";  // Replace with your Gemini API key
// Try gemini-pro-vision for image analysis, or gemini-1.5-pro-latest
// Using generateContent (not stream) since we need full response for TTS
// Try gemini-1.5-flash or gemini-3-flash-preview (if available)
// Using gemini-3-flash-preview (as shown in the API example)
const char* GEMINI_API_URL = "https://generativelanguage.googleapis.com/v1beta/models/gemini-3-flash-preview:generateContent";

// n8n webhook: send image + audio (base64), get back JSON with "text" (or "response") and we play via TTS
const char* N8N_API_URL = "";  // Replace with your n8n webhook URL
#define N8N_TIMEOUT_MS  60000  // n8n may take time (STT + Gemini + TTS)

// TTS Configuration - Single stream (full response in one request)

// State management
bool processing = false;
bool lastTouchState = false;

// Web server for viewing captured images
WebServer server(80);
uint8_t* lastCapturedImage = NULL;
size_t lastImageSize = 0;
unsigned long lastCaptureTime = 0;

// Recorded audio from PDM mic (raw 16-bit PCM, 16 kHz mono)
uint8_t* recordedAudioBuf = NULL;
size_t recordedAudioLen = 0;

Audio audio;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Initialize touch sensor pin
    pinMode(TOUCH_SENSOR_PIN, INPUT);
    
    // Connect to WiFi
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    delay(1000);
    
    // Initialize camera
    Serial.println("Initializing camera...");
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    // Frame size and quality - maximum quality settings
    if(psramFound()){
        Serial.println("PSRAM found - using SVGA resolution with maximum quality");
        config.frame_size = FRAMESIZE_SVGA; // 800x600 - higher resolution for better quality
        config.jpeg_quality = 10; // 0-63, lower number = higher quality (4 is maximum practical quality)
        config.fb_count = 1; // Use 1 buffer to save memory
    } else {
        Serial.println("No PSRAM - using VGA resolution with maximum quality");
        config.frame_size = FRAMESIZE_VGA; // 640x480 - fits without PSRAM
        config.jpeg_quality = 10; // Maximum quality
        config.fb_count = 1;
    }
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        Serial.println("Check: 1) PSRAM enabled in board settings, 2) Camera connected properly");
        return;
    }
    Serial.println("Camera initialized successfully!");
    
    // Get camera sensor and configure for maximum image quality
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        Serial.println("Camera sensor detected and configured");
        // Optimize settings for good image quality with proper colors and contrast
        s->set_brightness(s, 0);     // -2 to 2 (0 = neutral)
        s->set_contrast(s, 0);       // -2 to 2 (0 = neutral contrast for natural look)
        s->set_saturation(s, 0);     // -2 to 2 (0 = neutral saturation for accurate colors)
        s->set_sharpness(s, 1);      // -2 to 2 (1 = moderate sharpness, not too aggressive)
        s->set_denoise(s, 1);        // 0 = disable, 1 = enable (enable denoise to reduce grain)
        s->set_whitebal(s, 1);       // 0 = disable , 1 = enable (auto white balance for proper colors)
        s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable (auto white balance gain)
        s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable (auto exposure)
        s->set_aec2(s, 0);           // 0 = disable , 1 = enable (AEC2 for low light)
        s->set_ae_level(s, 0);       // -2 to 2 (0 = neutral exposure)
        s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable (auto gain)
        s->set_agc_gain(s, 1);       // 0 to 30 (moderate gain, not too high)
        s->set_gainceiling(s, (gainceiling_t)1);  // 0 to 6 (moderate ceiling to avoid color shifts)
        s->set_lenc(s, 1);           // 0 = disable , 1 = enable (lens correction)
        s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
        s->set_vflip(s, 0);          // 0 = disable , 1 = enable
        s->set_dcw(s, 1);            // 0 = disable , 1 = enable (downsize EN)
        s->set_colorbar(s, 0);       // 0 = disable , 1 = enable (disable colorbar)
        Serial.println("Camera sensor settings optimized for natural colors and good quality");
    } else {
        Serial.println("ERROR: Camera sensor object is NULL after initialization!");
        Serial.println("This indicates the camera sensor was not detected.");
        Serial.println("Check: 1) Camera FPC cable connection, 2) Camera module power");
        return;
    }
    
    // Give camera time to stabilize
    delay(2000);
    
    // Test camera capture capability with retries (BEFORE I2S initialization)
    Serial.println("Testing camera capture (before I2S init)...");
    camera_fb_t *test_fb = NULL;
    for(int retry = 0; retry < 5; retry++) {
        test_fb = esp_camera_fb_get();
        if (test_fb && test_fb->len > 0) {
            Serial.printf("Test capture successful! Size: %d bytes (retry %d)\n", test_fb->len, retry);
            esp_camera_fb_return(test_fb);
            break;
        } else {
            if (test_fb) {
                esp_camera_fb_return(test_fb);
            }
            Serial.printf("Test capture failed, retry %d/5...\n", retry + 1);
            delay(500);
        }
    }
    if (!test_fb || (test_fb && test_fb->len == 0)) {
        Serial.println("WARNING: Test capture failed after 5 retries");
        Serial.println("This might indicate:");
        Serial.println("  1. Camera hardware connection issue (check FPC cable)");
        Serial.println("  2. Incorrect camera pin configuration");
        Serial.println("  3. Camera sensor not responding");
        Serial.println("  4. Try running CameraWebServer example to verify hardware");
    }
    delay(500);
    
    // Setup I2S for audio (after camera test to avoid conflicts)
    // I2S pins changed to avoid conflict with camera Y3 (pin 7)
    Serial.println("Initializing I2S audio...");
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    delay(1000);
    
    // Set Volume
    audio.setVolume(15);
    Serial.println("I2S audio initialized");
    
    // Setup web server for viewing captured images
    setupWebServer();
    
    Serial.println("Setup complete! Touch to record voice + capture image, then get spoken response.");
    Serial.print("View captured images at: http://");
    Serial.println(WiFi.localIP());
    Serial.println("Or visit: http://xiao-esp32s3.local (if mDNS works)");
}

void loop() {

    if (audio.isRunning()) {
        audio.loop();
    }

    server.handleClient();

    if (!processing) {
        bool currentTouchState = digitalRead(TOUCH_SENSOR_PIN);

        if (currentTouchState == HIGH && lastTouchState == LOW) {
            Serial.println("Touch detected! Recording + capturing...");
            handleTouchEvent();
        }

        lastTouchState = currentTouchState;
    }

    delay(10);
}

// Record from PDM mic into recordedAudioBuf (raw PCM 16-bit mono 16 kHz). Caller must free recordedAudioBuf.
bool recordAudio() {
    if (recordedAudioBuf) {
        free(recordedAudioBuf);
        recordedAudioBuf = NULL;
        recordedAudioLen = 0;
    }
    size_t wantBytes = (size_t)SAMPLE_RATE * 2u * RECORD_SECONDS;  // 16-bit = 2 bytes per sample
#if defined(USE_ESP_I2S_V3)
    I2SClass i2s;
    i2s.setPinsPdmRx(MIC_PDM_CLK, MIC_PDM_DATA);
    if (!i2s.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        Serial.println("PDM mic init failed (ESP_I2S)");
        return false;
    }
#else
    I2S.setAllPins(-1, MIC_PDM_CLK, MIC_PDM_DATA, -1, -1);
    if (!I2S.begin(PDM_MONO_MODE, SAMPLE_RATE, 16)) {
        Serial.println("PDM mic init failed (I2S)");
        return false;
    }
#endif
    recordedAudioBuf = (uint8_t*)malloc(wantBytes);
    if (!recordedAudioBuf) {
        Serial.println("Record buffer malloc failed");
        return false;
    }
    Serial.printf("Recording %d seconds...\n", RECORD_SECONDS);
    size_t totalRead = 0;
    unsigned long endMs = millis() + (RECORD_SECONDS * 1000);
    while (millis() < endMs && totalRead < wantBytes) {
#if defined(USE_ESP_I2S_V3)
        int16_t s = i2s.read();
#else
        int16_t s = (int16_t)I2S.read();
#endif
        if (totalRead + 2 <= wantBytes) {
            int32_t scaled = (int32_t)s * RECORD_GAIN;
            if (scaled > 32767) scaled = 32767;
            if (scaled < -32768) scaled = -32768;
            s = (int16_t)scaled;
            recordedAudioBuf[totalRead] = (uint8_t)(s & 0xFF);
            recordedAudioBuf[totalRead + 1] = (uint8_t)((s >> 8) & 0xFF);
            totalRead += 2;
        }
        if (totalRead % 32000 == 0 && totalRead > 0) Serial.print(".");
    }
#if defined(USE_ESP_I2S_V3)
    i2s.end();
#else
    I2S.end();
#endif
    recordedAudioLen = totalRead;
    Serial.printf("\nRecorded %d bytes\n", (int)recordedAudioLen);
    return recordedAudioLen > 0;
}

// Send recorded PCM to Google Speech-to-Text; returns transcript or "".
String sendToGoogleSTT() {
    if (!recordedAudioBuf || recordedAudioLen == 0) return "";
    HTTPClient http;
    String url = String(STT_API_URL);
    http.begin(url);
    http.setTimeout(15000);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-goog-api-key", GEMINI_API_KEY);
    String base64Audio = base64::encode(recordedAudioBuf, recordedAudioLen);
    DynamicJsonDocument doc(1024);
    JsonObject config = doc.createNestedObject("config");
    config["encoding"] = "LINEAR16";
    config["sampleRateHertz"] = (int)SAMPLE_RATE;
    config["languageCode"] = "en-US";
    JsonObject audioObj = doc.createNestedObject("audio");
    audioObj["content"] = base64Audio;
    String payload;
    serializeJson(doc, payload);
    doc.clear();
    base64Audio = "";
    Serial.println("Sending audio to Google STT...");
    int code = http.POST(payload);
    String response = http.getString();
    http.end();
    payload = "";
    if (code != 200) {
        Serial.printf("STT HTTP %d: %s\n", code, response.c_str());
        return "";
    }
    DynamicJsonDocument resDoc(2048);
    if (deserializeJson(resDoc, response)) return "";
    JsonArray results = resDoc["results"];
    if (results.isNull() || results.size() == 0) return "";
    if (!results[0]["alternatives"][0].containsKey("transcript")) return "";
    String transcript = results[0]["alternatives"][0]["transcript"].as<String>();
    resDoc.clear();
    Serial.println("STT: " + transcript);
    return transcript;
}

// WAV header for 16-bit mono at SAMPLE_RATE (44 bytes)
#define WAV_HEADER_SIZE 44
void makeWavHeader(uint8_t* out, uint32_t pcmBytes) {
    uint32_t fileSize = pcmBytes + WAV_HEADER_SIZE - 8;
    uint32_t byteRate = (uint32_t)SAMPLE_RATE * 2u;  // 16-bit = 2 bytes per sample
    memcpy(out, "RIFF", 4);
    out[4] = (uint8_t)(fileSize);
    out[5] = (uint8_t)(fileSize >> 8);
    out[6] = (uint8_t)(fileSize >> 16);
    out[7] = (uint8_t)(fileSize >> 24);
    memcpy(out + 8, "WAVE", 4);
    memcpy(out + 12, "fmt ", 4);
    out[16] = 16; out[17] = 0; out[18] = 0; out[19] = 0;  // Subchunk1Size
    out[20] = 1; out[21] = 0;   // AudioFormat PCM
    out[22] = 1; out[23] = 0;   // NumChannels mono
    out[24] = (uint8_t)(SAMPLE_RATE);
    out[25] = (uint8_t)(SAMPLE_RATE >> 8);
    out[26] = (uint8_t)(SAMPLE_RATE >> 16);
    out[27] = (uint8_t)(SAMPLE_RATE >> 24);
    out[28] = (uint8_t)(byteRate);
    out[29] = (uint8_t)(byteRate >> 8);
    out[30] = (uint8_t)(byteRate >> 16);
    out[31] = (uint8_t)(byteRate >> 24);
    out[32] = 2; out[33] = 0;   // BlockAlign
    out[34] = 16; out[35] = 0;  // BitsPerSample
    memcpy(out + 36, "data", 4);
    out[40] = (uint8_t)(pcmBytes);
    out[41] = (uint8_t)(pcmBytes >> 8);
    out[42] = (uint8_t)(pcmBytes >> 16);
    out[43] = (uint8_t)(pcmBytes >> 24);
}

// Send image + audio (WAV) as raw binary (multipart/form-data).
String sendToN8n(uint8_t* imageData, size_t imageLen, uint8_t* audioData, size_t audioLen) {
    const char* boundary = "----ESP32N8N";
    const char* part1Header = "------ESP32N8N\r\nContent-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    const char* part2Header = "\r\n------ESP32N8N\r\nContent-Disposition: form-data; name=\"audio\"; filename=\"audio.wav\"\r\nContent-Type: audio/wav\r\n\r\n";
    const char* endBoundary = "\r\n------ESP32N8N--\r\n";
    size_t len1 = strlen(part1Header);
    size_t len2 = strlen(part2Header);
    size_t endLen = strlen(endBoundary);
    size_t audioPartLen = (audioData && audioLen > 0) ? (WAV_HEADER_SIZE + audioLen) : 0;
    size_t bodyLen = len1 + imageLen + len2 + audioPartLen + endLen;

    uint8_t* body = (uint8_t*)(psramFound() ? ps_malloc(bodyLen) : malloc(bodyLen));
    if (!body) {
        Serial.println("sendToN8n: malloc failed for multipart body");
        return "";
    }
    size_t off = 0;
    memcpy(body + off, part1Header, len1); off += len1;
    memcpy(body + off, imageData, imageLen); off += imageLen;
    memcpy(body + off, part2Header, len2); off += len2;
    if (audioData && audioLen > 0) {
        makeWavHeader(body + off, (uint32_t)audioLen);
        off += WAV_HEADER_SIZE;
        memcpy(body + off, audioData, audioLen);
        off += audioLen;
        Serial.printf("Audio: %d bytes PCM -> WAV %d bytes\n", (int)audioLen, (int)(WAV_HEADER_SIZE + audioLen));
    } else {
        Serial.println("Audio: none (empty part)");
    }
    memcpy(body + off, endBoundary, endLen); off += endLen;

    HTTPClient http;
    http.begin(N8N_API_URL);
    http.setTimeout(N8N_TIMEOUT_MS);
    http.addHeader("Content-Type", "multipart/form-data; boundary=----ESP32N8N");
    Serial.printf("Sending to n8n: multipart %d bytes (image %d + audio %d)\n", (int)bodyLen, (int)imageLen, (int)audioPartLen);
    int code = http.POST(body, bodyLen);
    free(body);
    String response = http.getString();
    http.end();
    Serial.printf("n8n HTTP %d, response length %d\n", code, response.length());
    if (code != 200) {
        Serial.println("n8n response: " + response.substring(0, 200));
        return "";
    }
    return response;
}

// Play audio from URL (e.g. n8n returns audioUrl).
void playAudioFromUrl(String audioUrl) {
    audioUrl.trim();
    if (audioUrl.length() == 0) {
        processing = false;
        return;
    }
    Serial.println("Playing audio from URL...");
    audio.stopSong();
    delay(200);
    audio.connecttohost(audioUrl.c_str());
    unsigned long start = millis();
    bool started = false;
    while (millis() - start < 120000) {
        audio.loop();
        if (audio.isRunning()) started = true;
        else if (started) break;
        delay(1);
    }
    audio.stopSong();
    delay(200);
    Serial.println("Audio playback complete.");
    processing = false;
}

void handleTouchEvent() {
    processing = true;
    
    // Step 1: Record audio from PDM mic
    Serial.println("Recording from microphone...");
    if (!recordAudio()) {
        Serial.println("Recording failed - sending image only to n8n.");
    }
    
    // Step 2: Capture image from camera
    Serial.println("Attempting to capture image...");
    for (int i = 0; i < 2; i++) {
        camera_fb_t *discard = esp_camera_fb_get();
        if (discard) esp_camera_fb_return(discard);
        delay(100);
    }
    
    camera_fb_t *fb = NULL;
    for (int retry = 0; retry < 3; retry++) {
        fb = esp_camera_fb_get();
        if (fb && fb->len > 0) break;
        if (fb) { esp_camera_fb_return(fb); fb = NULL; }
        Serial.printf("Capture retry %d/3...\n", retry + 1);
        delay(200);
    }
    
    if (!fb || fb->len == 0) {
        Serial.println("Camera capture failed.");
        if (recordedAudioBuf) { free(recordedAudioBuf); recordedAudioBuf = NULL; recordedAudioLen = 0; }
        processing = false;
        return;
    }
    
    Serial.printf("Image captured! Size: %d bytes\n", fb->len);
    
    // Free previous web image to free memory
    if (lastCapturedImage) {
        free(lastCapturedImage);
        lastCapturedImage = NULL;
        lastImageSize = 0;
    }
    
    // Step 3: Send image + audio to n8n
    String n8nResponse = sendToN8n(fb->buf, fb->len, recordedAudioBuf, recordedAudioLen);
    if (recordedAudioBuf) {
        free(recordedAudioBuf);
        recordedAudioBuf = NULL;
        recordedAudioLen = 0;
    }
    
    // Store image for web viewing
    lastCapturedImage = (uint8_t*)malloc(fb->len);
    if (lastCapturedImage) {
        memcpy(lastCapturedImage, fb->buf, fb->len);
        lastImageSize = fb->len;
        lastCaptureTime = millis();
    }
    esp_camera_fb_return(fb);
    
    // Step 4: Parse n8n response for text and play via TTS
    n8nResponse.trim();
    if (n8nResponse.length() == 0) {
        Serial.println("Empty n8n response.");
        processing = false;
        Serial.printf("Free heap after processing: %d bytes\n", ESP.getFreeHeap());
        return;
    }
    String textToSpeak = "";
    // If response is short and doesn't look like JSON, treat whole body as text
    if (n8nResponse.length() <= 200 && n8nResponse.indexOf('{') != 0) {
        textToSpeak = n8nResponse;
    }
    if (textToSpeak.length() == 0 && n8nResponse.indexOf('{') >= 0) {
        DynamicJsonDocument doc(1024);
        if (!deserializeJson(doc, n8nResponse)) {
            const char* t = doc["text"];
            if (!t) t = doc["response"];
            if (!t) t = doc["result"];
            if (!t) t = doc["message"];
            if (!t) t = doc["output"];
            if (!t && doc["data"].is<const char*>()) t = doc["data"];
            if (!t && doc["body"].is<const char*>()) t = doc["body"];
            if (!t && doc["data"].is<JsonObject>()) t = doc["data"]["text"];
            if (!t && doc["body"].is<JsonObject>()) t = doc["body"]["text"];
            if (!t && doc["json"].is<JsonObject>()) t = doc["json"]["text"];
            if (!t && doc["output"].is<JsonObject>()) t = doc["output"]["text"];
            if (t && strlen(t) > 0) textToSpeak = String(t);
        }
        doc.clear();
    }
    if (textToSpeak.length() > 0) {
        Serial.println("n8n text: " + textToSpeak.substring(0, (textToSpeak.length() < 80u) ? textToSpeak.length() : 80) + "...");
        playTTS(textToSpeak);
        Serial.printf("Free heap after processing: %d bytes\n", ESP.getFreeHeap());
        return;
    }
    Serial.println("No text in n8n response. Raw: " + n8nResponse.substring(0, 100));
    processing = false;
    Serial.printf("Free heap after processing: %d bytes\n", ESP.getFreeHeap());
}

String sendToGemini(uint8_t* imageData, size_t imageLen, String userPrompt) {
    HTTPClient http;
    String url = String(GEMINI_API_URL);
    
    // Use API key as header (matching the curl example structure)
    // Configure HTTP client with timeouts
    http.begin(url);
    http.setTimeout(30000); // 30 second timeout
    http.addHeader("Content-Type", "application/json");
    http.addHeader("x-goog-api-key", GEMINI_API_KEY); // Lowercase header name as in example
    
    Serial.printf("Connecting to: %s\n", url.c_str());
    
    // Encode image to base64
    String base64Image = base64::encode(imageData, imageLen);
    Serial.printf("Base64 image length: %d\n", base64Image.length());
    Serial.printf("Free heap before JSON: %d bytes\n", ESP.getFreeHeap());
    
    // Create JSON payload for Gemini API using ArduinoJson - matching REST API structure
    // Calculate needed size: base64 length + JSON structure overhead (quotes, brackets, etc.)
    // Need extra space for JSON formatting (quotes around strings, brackets, etc.)
    size_t docSize = base64Image.length() + 2000; // Base64 length + JSON structure overhead
    // Don't cap - we need the full size for the image data
    
    DynamicJsonDocument doc(docSize);
    JsonArray contents = doc.createNestedArray("contents");
    JsonObject content = contents.createNestedObject();
    content["role"] = "user"; // Add role field as shown in REST API example
    JsonArray parts = content.createNestedArray("parts");
    
    // Add text part: user's spoken prompt (from STT) + instruction for brief response
    JsonObject textPart = parts.createNestedObject();
    String promptText = "The user said: \"" + userPrompt + "\". Look at this image and answer based on what they said. Give a brief 1-2 line response only.";
    if (promptText.length() > 500) promptText = promptText.substring(0, 500);
    textPart["text"] = promptText;
    
    // Add image part
    JsonObject imagePart = parts.createNestedObject();
    JsonObject inlineData = imagePart.createNestedObject("inline_data");
    inlineData["mime_type"] = "image/jpeg";
    inlineData["data"] = base64Image;
    
    String payload;
    size_t serializedSize = serializeJson(doc, payload);
    
    // Verify serialization worked
    if (serializedSize == 0 || payload.length() < 100) {
        Serial.println("ERROR: JSON serialization failed or payload too small!");
        Serial.printf("Serialized size: %d, Payload length: %d\n", serializedSize, payload.length());
        doc.clear();
        base64Image = "";
        return "";
    }
    
    // Free memory immediately after creating payload
    doc.clear();
    base64Image = ""; // Free the base64 string
    Serial.printf("Free heap after JSON: %d bytes\n", ESP.getFreeHeap());
    
    Serial.println("Sending request to Gemini API...");
    Serial.printf("Payload size: %d bytes\n", payload.length());
    
    // Retry logic for rate limiting (429 errors)
    int httpResponseCode = 0;
    String response = "";
    int maxRetries = 3;
    int retryDelay = 2000; // Start with 2 seconds
    
    for (int retry = 0; retry < maxRetries; retry++) {
        httpResponseCode = http.POST(payload);
        
        if (httpResponseCode > 0) {
            Serial.printf("HTTP Response code: %d\n", httpResponseCode);
            response = http.getString();
            
            if (httpResponseCode == 200) {
                // Success! Parse JSON response to extract text
                response = parseGeminiResponse(response);
                break; // Exit retry loop on success
            } else if (httpResponseCode == 429) {
                // Rate limit exceeded - wait and retry
                Serial.println("Rate limit exceeded (429). Waiting before retry...");
                if (retry < maxRetries - 1) {
                    delay(retryDelay);
                    retryDelay *= 2; // Exponential backoff
                    Serial.printf("Retrying... (attempt %d/%d)\n", retry + 2, maxRetries);
                    continue;
                } else {
                    Serial.println("Max retries reached. Quota may be exhausted.");
                    Serial.println("Error response:");
                    Serial.println(response);
                    Serial.println("\nSuggestions:");
                    Serial.println("1. Wait a few minutes and try again");
                    Serial.println("2. Check your API quota at: https://ai.dev/rate-limit");
                    Serial.println("3. Consider upgrading your API plan");
                    response = "";
                }
            } else {
                // Other error - log and exit
                Serial.println("Error response from Gemini API:");
                Serial.println(response);
                response = "";
                break;
            }
        } else {
            // HTTP error codes
            Serial.printf("Error on HTTP request: %d\n", httpResponseCode);
            String errorResponse = http.getString();
            
            // Decode common HTTPClient error codes
            if (httpResponseCode == -1) {
                Serial.println("Error: Connection failed (timeout or no connection)");
            } else if (httpResponseCode == -11) {
                Serial.println("Error: Connection error (SSL/TLS or network issue)");
                Serial.println("Possible causes:");
                Serial.println("  1. SSL certificate issue");
                Serial.println("  2. Network connectivity problem");
                Serial.println("  3. Firewall blocking connection");
                Serial.println("  4. Invalid model name in URL (check gemini-2.5-flash exists)");
            } else if (httpResponseCode < 0) {
                Serial.printf("Error: HTTPClient error code %d\n", httpResponseCode);
            }
            
            if (errorResponse.length() > 0) {
                Serial.println("Error response:");
                Serial.println(errorResponse);
            }
            response = "";
            break;
        }
    }
    
    http.end();
    
    // Clear payload to free memory
    payload = "";
    payload.reserve(0);
    
    if (httpResponseCode != 200) {
        Serial.println("Gemini API request failed. Check the error message above.");
        response = "";
        return ""; // Return empty string on error
    }
    
    Serial.printf("Free heap after API call: %d bytes\n", ESP.getFreeHeap());
    return response;
}

String parseGeminiResponse(String jsonResponse) {
    // Parse JSON to extract the text content
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, jsonResponse);
    
    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return "";
    }
    
    // Navigate through the JSON structure
    if (doc.containsKey("candidates") && doc["candidates"].is<JsonArray>()) {
        JsonArray candidates = doc["candidates"];
        if (candidates.size() > 0) {
            JsonObject candidate = candidates[0];
            if (candidate.containsKey("content") && candidate["content"].is<JsonObject>()) {
                JsonObject content = candidate["content"];
                if (content.containsKey("parts") && content["parts"].is<JsonArray>()) {
                    JsonArray parts = content["parts"];
                    if (parts.size() > 0) {
                        JsonObject part = parts[0];
                        if (part.containsKey("text")) {
                            return part["text"].as<String>();
                        }
                    }
                }
            }
        }
    }
    
    return "";
}

void playTTS(String text) {

    if (text.length() > 200) {
        text = text.substring(0, 200);
        Serial.println("Text truncated to 200 chars");
    }

    text.trim();
    if (text.length() == 0) {
        processing = false;
        return;
    }

    Serial.println("Playing TTS: " + text.substring(0, 50) + "...");
    Serial.printf("Free heap before audio: %d bytes\n", ESP.getFreeHeap());

    String ttsUrl = "https://translate.google.com/translate_tts?ie=UTF-8&q=" + urlEncode(text) + "&tl=en&client=tw-ob";

    audio.stopSong();
    delay(200);
    audio.connecttohost(ttsUrl.c_str());

    unsigned long start = millis();
    bool started = false;

    while (millis() - start < 30000) {

        if (audio.isRunning()) {
            audio.loop();
            started = true;
        } else {
            if (started) break;  // playback finished cleanly
        }

        delay(1);
    }

    audio.stopSong();
    delay(200);

    Serial.printf("Free heap after audio: %d bytes\n", ESP.getFreeHeap());
    Serial.println("TTS playback complete.");

    processing = false;
}

/*void playTTS(String text) {
    if (text.length() > 200) {
        text = text.substring(0, 200);
        Serial.println("Text truncated to 200 chars");
    }
    text.trim();
    if (text.length() == 0) {
        processing = false;
        return;
    }
    Serial.println("Playing TTS (single): " + text.substring(0, 50) + "...");
    Serial.printf("Free heap before audio: %d bytes\n", ESP.getFreeHeap());
    String encodedText = urlEncode(text);
    String ttsUrl = "https://translate.google.com/translate_tts?ie=UTF-8&q=" + encodedText + "&tl=en&client=tw-ob";
    encodedText = "";
    audio.stopSong();
    delay(500);
    audio.connecttohost(ttsUrl.c_str());
    unsigned long connectStart = millis();
    bool connected = false;
    while ((millis() - connectStart < 5000) && !connected) {
        audio.loop();
        if (audio.isRunning()) connected = true;
        delay(10);
    }
    if (!connected) {
        Serial.println("Failed to connect to TTS stream");
        audio.stopSong();
        processing = false;
        ttsUrl = "";
        return;
    }
    delay(3000);
    unsigned long playbackStart = millis();
    unsigned long lastRunningTime = millis();
    bool wasRunning = false;
    unsigned long firstRunningTime = 0;
    while ((millis() - playbackStart < 60000)) {
        audio.loop();
        bool isRunning = audio.isRunning();
        if (isRunning) {
            if (firstRunningTime == 0) firstRunningTime = millis();
            wasRunning = true;
            lastRunningTime = millis();
        } else {
            if (wasRunning) {
                unsigned long timeSinceStop = millis() - lastRunningTime;
                unsigned long totalPlayTime = (firstRunningTime > 0) ? (lastRunningTime - firstRunningTime) : 0;
                if (timeSinceStop > 2000) {
                    if (totalPlayTime > 2000 || timeSinceStop > 3000) break;
                }
            } else if (millis() - playbackStart > 10000) break;
        }
        delay(0);
    }
    audio.stopSong();
    delay(300);
    ttsUrl = "";
    text = "";
    Serial.printf("Free heap after audio: %d bytes\n", ESP.getFreeHeap());
    processing = false;
    Serial.println("TTS playback complete.");
}
*/

String urlEncode(String str) {
    String encoded = "";
    char c;
    char code0;
    char code1;
    for (int i = 0; i < str.length(); i++) {
        c = str.charAt(i);
        if (c == ' ') {
            encoded += '+';
        } else if (isalnum(c)) {
            encoded += c;
        } else {
            code1 = (c & 0xf) + '0';
            if ((c & 0xf) > 9) {
                code1 = (c & 0xf) - 10 + 'A';
            }
            c = (c >> 4) & 0xf;
            code0 = c + '0';
            if (c > 9) {
                code0 = c - 10 + 'A';
            }
            encoded += '%';
            encoded += code0;
            encoded += code1;
        }
    }
    return encoded;
}

// Optional: This will tell us exactly what the error is in the Serial Monitor
void audio_error(const char *info){
    // Suppress buffer underrun warnings - these are expected with WiFi streaming
    // They don't affect playback, just indicate the buffer is temporarily empty
    if (strstr(info, "readSpace") == NULL && strstr(info, "bytesWasRead") == NULL) {
        Serial.print("audio_error: ");
        Serial.println(info);
    }
    
    // Buffer underrun errors are normal with streaming audio over WiFi
    // The audio library handles them automatically
}

// Web server functions for viewing captured images
void setupWebServer() {
    // Root page - shows the last captured image
    server.on("/", handleRoot);
    
    // Image endpoint - serves the JPEG image
    server.on("/image.jpg", handleImage);
    
    // Status endpoint
    server.on("/status", handleStatus);
    
    server.begin();
    Serial.println("Web server started");
    
    // Try to set up mDNS (optional, for easier access)
    if (!MDNS.begin("xiao-esp32s3")) {
        Serial.println("mDNS failed to start");
    } else {
        Serial.println("mDNS responder started - access via http://xiao-esp32s3.local");
    }
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>ESP32S3 Camera Viewer</title>";
    html += "<style>body{font-family:Arial;text-align:center;background:#f0f0f0;}";
    html += "img{border:2px solid #333;max-width:90%;height:auto;margin:20px;}";
    html += ".info{background:white;padding:15px;margin:20px;border-radius:5px;}</style></head><body>";
    html += "<h1>ESP32S3 Camera - Last Captured Image</h1>";
    
    if (lastCapturedImage != NULL && lastImageSize > 0) {
        html += "<div class='info'>";
        html += "<p><strong>Image Size:</strong> " + String(lastImageSize) + " bytes</p>";
        html += "<p><strong>Captured:</strong> " + String((millis() - lastCaptureTime) / 1000) + " seconds ago</p>";
        html += "</div>";
        html += "<img src='/image.jpg' alt='Captured Image' onerror='this.alt=\"Image not available\"'>";
        html += "<p><a href='/image.jpg' download>Download Image</a></p>";
    } else {
        html += "<div class='info'><p>No image captured yet. Touch the sensor to capture an image.</p></div>";
    }
    
    html += "<p><a href='/status'>Status</a> | <a href='/'>Refresh</a></p>";
    html += "<p><small>Auto-refresh every 5 seconds</small></p>";
    html += "<script>setTimeout(function(){location.reload();},5000);</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleImage() {
    if (lastCapturedImage != NULL && lastImageSize > 0) {
        server.send_P(200, "image/jpeg", (const char*)lastCapturedImage, lastImageSize);
    } else {
        server.send(404, "text/plain", "No image available");
    }
}

void handleStatus() {
    String json = "{";
    json += "\"hasImage\":" + String(lastCapturedImage != NULL ? "true" : "false") + ",";
    json += "\"imageSize\":" + String(lastImageSize) + ",";
    json += "\"captureTime\":" + String(lastCaptureTime) + ",";
    json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    if(psramFound()) {
        json += "\"freePSRAM\":" + String(ESP.getFreePsram()) + ",";
    }
    json += "\"processing\":" + String(processing ? "true" : "false");
    json += "}";
    
    server.send(200, "application/json", json);
}
