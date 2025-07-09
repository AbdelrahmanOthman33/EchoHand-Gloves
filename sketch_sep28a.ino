include <WiFi.h>
include <WiFiClientSecure.h>
include <HTTPClient.h>
include <ArduinoJson.h>
include <driver/i2s.h>
include <Wire.h>
include <LiquidCrystal_I2C.h>
include <DFRobotDFPlayerMini.h>
include <HardwareSerial.h>

// -------------------- Configuration --------------------

// Wi-Fi Credentials
const char* ssid = "GOAT";
const char* password = "M@ram7281##";

// Azure Speech API Credentials
const char* azureApiKey = "dafb62bb72af4e359bce043ce99c3fae";
const char* azureRegion = "eastus"; // e.g., "eastus"

// Azure endpoint
String azureEndpoint = String("https://") + azureRegion + ".stt.speech.microsoft.com/speech/recognition/conversation/cognitiveservices/v1?language=en-US";

// I2S configuration for INMP441
define I2S_WS  14  // Word Select (LRCLK)
define I2S_SD  15  // Serial Data Output (DOUT)
define I2S_SCK 25  // Serial Clock (BCLK)

// Audio Settings
define SAMPLE_RATE     16000
define SAMPLE_BITS     16
define CHANNELS        1
define RECORD_TIME     1       // Additional recording time in seconds
define CHUNK_DURATION  1.0     // Duration of each audio chunk in seconds
define CHUNK_SIZE      (SAMPLE_RATE * (SAMPLE_BITS / 8) * CHANNELS * CHUNK_DURATION) // 32000 bytes
define VAD_THRESHOLD   40      // Initial VAD threshold (adjust based on testing)

// -------------------- Flex Sensor Configuration --------------------

// Number of flex sensors
const int numFlexSensors = 4;

// GPIO pins connected to flex sensors
const int flexPins[numFlexSensors] = {34, 35, 39, 32}; // GPIO34: Index, GPIO35: Middle, GPIO39: Little, GPIO32: Thumb

// Finger names corresponding to flex sensors
const char* fingerNames[numFlexSensors] = {"Index", "Middle", "Little", "Thumb"};

// Calibration values for flex sensors (ensure flexMin < flexMax)
int flexMin[numFlexSensors] = {357, 461, 624, 783}; // Example minimum values
int flexMax[numFlexSensors] = {55, 179, 308, 417};  // Example maximum values

// Threshold points for each flex sensor
float flexThresholds[numFlexSensors][3]; // [0]: Half Straight, [1]: Half Bent, [2]: Bent

// Finger states
enum FingerState {
  STRAIGHT,
  HALF_STRAIGHT,
  HALF_BENT,
  BENT
};

FingerState fingerStates[numFlexSensors];

// Previous gesture states to prevent repeated actions
// Updated to accommodate 5 gestures
bool previousGestureStates[5] = {false, false, false, false, false}; // Index 4 for Gesture5

// -------------------- LCD and MP3 Configuration --------------------

// Initialize LCD (I2C address 0x27, 16x2 display)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize DFPlayer Mini using HardwareSerial (UART1: GPIO16 RX, GPIO17 TX)
HardwareSerial mySerial(1);
DFRobotDFPlayerMini mp3;

// -------------------- Timing Variables --------------------
unsigned long previousMillis = 0;
const long interval = 100; // 100 ms for flex sensor readings

// Flag to indicate when Azure response is being displayed
bool displayingAzureResponse = false;
unsigned long azureDisplayStartTime = 0;
const long azureDisplayDuration = 5000; // Display Azure response for 5 seconds

// Variables for scrolling text
bool scrolling = false;
String scrollTextBuffer = "";
unsigned long scrollPreviousMillis = 0;

// Adjusted scroll interval for faster movement
const long scrollInterval = 150; // Reduced from 300 ms to 150 ms

int scrollPosition = 0;

// -------------------- Function Declarations --------------------

void setupWiFi();
void setupMic();
void recordAndSendAudio(int16_t* initialBuffer, size_t initialBytesRead);
void addWavHeader(uint8_t* wavData, uint32_t pcmDataSize);
void parseAzureResponse(const String& response);
bool detectVoice(int16_t* audioBuffer, size_t length);

// Flex Sensor Functions
void calculateFlexThresholds();
void classifyFingerStates(int flexValues[]);
void recognizeGesture();
void performAction1();
void performAction2();
void performAction3();
void performAction4();
void performAction5(); // New function for Gesture5
void resetGestureFlags();

// Scrolling Text Function
void startScroll(String text);
void updateScroll();

// -------------------- Custom Stream Class --------------------

// Custom Stream class to combine WAV header and audio data
class AudioStream : public Stream {
private:
    uint8_t* header;
    size_t headerSize;
    size_t headerPos;
    uint8_t* audioData;
    size_t audioSize;
    size_t audioPos;

public:
    AudioStream(uint8_t* wavHeader, size_t headerSize, uint8_t* audioData, size_t audioSize)
        : header(wavHeader), headerSize(headerSize), headerPos(0),
          audioData(audioData), audioSize(audioSize), audioPos(0) {}

    // Required pure virtual functions
    virtual int read() override {
        if (headerPos < headerSize) {
            return header[headerPos++];
        } else if (audioPos < audioSize) {
            return audioData[audioPos++];
        }
        return -1; // No more data
    }

    virtual int available() override {
        return (headerPos < headerSize) ? (headerSize - headerPos) : (audioSize - audioPos);
    }

    virtual int peek() override {
        if (headerPos < headerSize) {
            return header[headerPos];
        } else if (audioPos < audioSize) {
            return audioData[audioPos];
        }
        return -1;
    }

    virtual void flush() override {}

    // Implement write to satisfy the Stream class's pure virtual function
    virtual size_t write(uint8_t) override { return 0; }
};

// -------------------- Setup --------------------

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial); // Wait for Serial to be ready
    Serial.println("System initializing...");

    // Connect to Wi-Fi
    setupWiFi();

    // Initialize I2S microphone
    setupMic();

    // Initialize flex sensor pins
    for (int i = 0; i < numFlexSensors; i++) {
        pinMode(flexPins[i], INPUT);
    }

    // Initialize I2C for LCD
    Wire.begin();

    // Initialize LCD
    lcd.begin();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    lcd.setCursor(0, 1);
    lcd.print("Please wait");

    // Initialize DFPlayer Mini
    mySerial.begin(9600, SERIAL_8N1, 16, 17); // RX: GPIO16, TX: GPIO17
    if (!mp3.begin(mySerial)) {
        Serial.println("DFPlayer Mini not detected!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("MP3 Error");
        while (1); // Halt if DFPlayer is not detected
    }
    mp3.volume(11); // Set volume (0-30)
    Serial.println("DFPlayer Mini initialized.");

    // Calculate flex sensor thresholds
    calculateFlexThresholds();
    Serial.println("Flex thresholds calculated.");

    // Display ready message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready!");
    lcd.setCursor(0, 1);
    lcd.print("Welcome ECO_Hands");
    Serial.println("System ready.");

    // Play welcome sound
    Serial.println("Playing Welcome Sound");
    mp3.play(1); // Play track 001.mp3
    delay(3000); // Wait for 3 seconds to display the message
}

// -------------------- Loop --------------------

void loop() {
    unsigned long currentMillis = millis();

    // Handle flex sensor readings at specified intervals
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Read flex sensor values
        int flexValues[numFlexSensors];
        for (int i = 0; i < numFlexSensors; i++) {
            flexValues[i] = analogRead(flexPins[i]);
            flexValues[i] = map(flexValues[i], flexMin[i], flexMax[i], 0, 100); // Convert to percentage
            flexValues[i] = constrain(flexValues[i], 0, 100); // Constrain between 0-100%
            Serial.print(fingerNames[i]);
            Serial.print(": ");
            Serial.print(flexValues[i]);
            Serial.print(" | ");
        }
        Serial.println();

        // Classify finger states
        classifyFingerStates(flexValues);

        // Print finger states for debugging
        for (int i = 0; i < numFlexSensors; i++) {
            Serial.print(fingerNames[i]);
            Serial.print(": ");
            switch (fingerStates[i]) {
                case STRAIGHT:
                    Serial.print("Straight");
                    break;
                case HALF_STRAIGHT:
                    Serial.print("Half Straight");
                    break;
                case HALF_BENT:
                    Serial.print("Half Bent");
                    break;
                case BENT:
                    Serial.print("Bent");
                    break;
            }
            Serial.print(" | ");
        }
        Serial.println();

        // Recognize gestures based on finger states
        recognizeGesture();

        // Reset gesture flags if no gesture is matched
        resetGestureFlags();
    }

    // Handle Azure audio processing and communication
    // Allocate memory for a single audio chunk
    int16_t* audioBuffer = (int16_t*)malloc(CHUNK_SIZE);
    if (!audioBuffer) {
        Serial.println("Failed to allocate memory for audio buffer.");
        return;
    }

    size_t bytesRead = 0;
    i2s_read(I2S_NUM_0, audioBuffer, CHUNK_SIZE, &bytesRead, portMAX_DELAY);

    // Calculate the number of samples read
    size_t numSamples = bytesRead / sizeof(int16_t);

    // Detect voice
    bool voiceDetected = detectVoice(audioBuffer, numSamples);

    if (voiceDetected) {
        Serial.println("Voice detected! Recording audio...");
        lcd.setCursor(0, 1);
        lcd.print("Voice Detected  ");
        // Record audio and send to Azure
        recordAndSendAudio(audioBuffer, bytesRead);
    } else {
        Serial.println("No voice detected.");
    }

    // Calculate and print average energy for debugging
    int32_t sum = 0;
    for (size_t i = 0; i < numSamples; i++) {
        sum += abs(audioBuffer[i]);
    }
    int32_t avg = (numSamples > 0) ? (sum / numSamples) : 0;
    Serial.printf("Average Energy: %d\n", avg);

    free(audioBuffer);

    // Manage Azure response display duration
    if (displayingAzureResponse) {
        if (millis() - azureDisplayStartTime >= azureDisplayDuration) {
            displayingAzureResponse = false;
            // Clear Azure response display
            scrollTextBuffer = "";
            scrollPosition = 0;
            scrolling = false;
            lcd.setCursor(0, 1);
            lcd.print("                "); // Clear second line
        } else {
            // Continue scrolling if active
            if (scrolling) {
                updateScroll();
            }
        }
    }
}

// -------------------- Function Definitions --------------------

// Connect to Wi-Fi
void setupWiFi() {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to Wi-Fi.");
}

// Configure I2S for INMP441
void setupMic() {
    i2s_config_t i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Mono channel
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pinConfig = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2sConfig, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing I2S driver: %d\n", err);
        while (true);
    }

    err = i2s_set_pin(I2S_NUM_0, &pinConfig);
    if (err != ESP_OK) {
        Serial.printf("Failed setting I2S pins: %d\n", err);
        while (true);
    }

    i2s_zero_dma_buffer(I2S_NUM_0);
    Serial.println("Microphone initialized.");
}

// Detect voice based on energy level
bool detectVoice(int16_t* audioBuffer, size_t length) {
    int32_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += abs(audioBuffer[i]);
    }
    int32_t avg = (length > 0) ? (sum / length) : 0;

    Serial.printf("VAD - Average Energy: %d\n", avg);

    return avg > VAD_THRESHOLD;
}

// Record audio and send to Azure, including initial audioBuffer
void recordAndSendAudio(int16_t* initialBuffer, size_t initialBytesRead) {
    // Calculate total samples for RECORD_TIME seconds
    size_t recordSamples = SAMPLE_RATE * RECORD_TIME * CHANNELS;
    size_t recordBytes = recordSamples * (SAMPLE_BITS / 8);

    // Allocate memory for additional audio data
    uint8_t* additionalAudioData = (uint8_t*)malloc(recordBytes);
    if (!additionalAudioData) {
        Serial.println("Failed to allocate memory for additional audio data.");
        lcd.setCursor(0, 1);
        lcd.print("Memory Error    ");
        return;
    }

    // Record additional audio
    size_t bytesRead = 0;
    i2s_read(I2S_NUM_0, additionalAudioData, recordBytes, &bytesRead, portMAX_DELAY);
    Serial.printf("Additional recording complete. Bytes read: %d\n", bytesRead);

    // Total audio data size = initialBytesRead + bytesRead
    size_t totalBytes = initialBytesRead + bytesRead;

    // Allocate memory for total audio data
    uint8_t* totalAudioData = (uint8_t*)malloc(totalBytes);
    if (!totalAudioData) {
        Serial.println("Failed to allocate memory for total audio data.");
        free(additionalAudioData);
        lcd.setCursor(0, 1);
        lcd.print("Memory Error    ");
        return;
    }

    // Copy initial audioBuffer to totalAudioData
    memcpy(totalAudioData, initialBuffer, initialBytesRead);

    // Copy additionalAudioData to totalAudioData
    memcpy(totalAudioData + initialBytesRead, additionalAudioData, bytesRead);

    free(additionalAudioData); // No longer needed

    // Prepare WAV header
    uint8_t wavHeader[44];
    addWavHeader(wavHeader, totalBytes);
    Serial.printf("WAV header prepared.\n");

    // Create AudioStream
    AudioStream stream(wavHeader, sizeof(wavHeader), totalAudioData, totalBytes);

    // Calculate total payload size (header + audio data)
    size_t payloadSize = sizeof(wavHeader) + totalBytes;
    Serial.printf("Payload size: %d bytes\n", payloadSize);

    // Check available heap memory
    Serial.printf("Free Heap before sending: %d bytes\n", ESP.getFreeHeap());

    // Initialize HTTPS client
    WiFiClientSecure client;
    HTTPClient https;
    client.setInsecure(); // Use with caution; validate certificates in production

    if (!https.begin(client, azureEndpoint)) {
        Serial.println("Unable to connect to Azure.");
        lcd.setCursor(0, 1);
        lcd.print("Azure Connect Err");
        free(totalAudioData);
        return;
    }

    // Set headers
    https.addHeader("Content-Type", "audio/wav");
    https.addHeader("Ocp-Apim-Subscription-Key", azureApiKey);
    https.addHeader("Content-Length", String(payloadSize));

    // Send the request with AudioStream and payload size
    int httpResponseCode = https.sendRequest("POST", &stream, payloadSize);

    if (httpResponseCode > 0) {
        Serial.printf("HTTP Response Code: %d\n", httpResponseCode);
        String response = https.getString();
        parseAzureResponse(response);
    } else {
        Serial.printf("Error in HTTPS request: %s\n", https.errorToString(httpResponseCode).c_str());
        lcd.setCursor(0, 1);
        lcd.print("Azure Request Err");
    }

    https.end();
    free(totalAudioData);

    // Check available heap memory after sending
    Serial.printf("Free Heap after sending: %d bytes\n", ESP.getFreeHeap());
}

// Add WAV header
void addWavHeader(uint8_t* wavData, uint32_t pcmDataSize) {
    memcpy(wavData, "RIFF", 4);
    uint32_t fileSize = pcmDataSize + 36; // 4 + (8 + fmtChunkSize) + (8 + dataSize)
    memcpy(wavData + 4, &fileSize, 4);
    memcpy(wavData + 8, "WAVE", 4);

    memcpy(wavData + 12, "fmt ", 4);
    uint32_t fmtChunkSize = 16;
    memcpy(wavData + 16, &fmtChunkSize, 4);
    uint16_t audioFormat = 1; // PCM
    memcpy(wavData + 20, &audioFormat, 2);
    uint16_t numChannels = CHANNELS;
    memcpy(wavData + 22, &numChannels, 2);
    uint32_t sampleRate = SAMPLE_RATE;
    memcpy(wavData + 24, &sampleRate, 4);
    uint32_t byteRate = SAMPLE_RATE * CHANNELS * (SAMPLE_BITS / 8);
    memcpy(wavData + 28, &byteRate, 4);
    uint16_t blockAlign = CHANNELS * (SAMPLE_BITS / 8);
    memcpy(wavData + 32, &blockAlign, 2);
    uint16_t bitsPerSample = SAMPLE_BITS;
    memcpy(wavData + 34, &bitsPerSample, 2);

    memcpy(wavData + 36, "data", 4);
    memcpy(wavData + 40, &pcmDataSize, 4);
}

// Parse Azure response
void parseAzureResponse(const String& response) {
    StaticJsonDocument<4096> doc; // Increased size to handle larger responses
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
        Serial.printf("Failed to parse JSON: %s\n", error.c_str());
        lcd.setCursor(0, 1);
        lcd.print("JSON Parse Err  ");
        return;
    }

    const char* recognitionStatus = doc["RecognitionStatus"];
    if (strcmp(recognitionStatus, "Success") == 0) {
        const char* text = doc["DisplayText"];
        Serial.printf("Transcribed Text: %s\n", text);
        lcd.setCursor(0, 1);
        lcd.print("Azure:          "); // Clear the line
        lcd.setCursor(0, 1);
        lcd.print(text);

        // Start scrolling if text exceeds LCD width
        if (strlen(text) > 16) {
            startScroll(String(text));
        }

        // Set flag to manage display duration
        displayingAzureResponse = true;
        azureDisplayStartTime = millis();
    } else {
        Serial.printf("Recognition Status: %s\n", recognitionStatus);
        lcd.setCursor(0, 1);
        lcd.print("Azure Status:   ");
        lcd.setCursor(0, 1);
        lcd.print(recognitionStatus);

        // Set flag to manage display duration
        displayingAzureResponse = true;
        azureDisplayStartTime = millis();
    }
}

// -------------------- Flex Sensor Functions --------------------

// Calculate threshold points for each flex sensor
void calculateFlexThresholds() {
    for (int i = 0; i < numFlexSensors; i++) {
        flexThresholds[i][0] = 25; // Half Straight
        flexThresholds[i][1] = 50; // Half Bent
        flexThresholds[i][2] = 75; // Bent
    }
}

// Classify finger states based on flex sensor values
void classifyFingerStates(int flexValues[]) {
    for (int i = 0; i < numFlexSensors; i++) {
        if (flexValues[i] < (flexThresholds[i][0] - 5)) { // With a small margin
            fingerStates[i] = STRAIGHT;
        }
        else if (flexValues[i] < (flexThresholds[i][1] - 5)) {
            fingerStates[i] = HALF_STRAIGHT;
        }
        else if (flexValues[i] < (flexThresholds[i][2] - 5)) {
            fingerStates[i] = HALF_BENT;
        }
        else {
            fingerStates[i] = BENT;
        }
    }
}

// Recognize gestures based on finger states
void recognizeGesture() {
    // Gesture1: Index BENT, others STRAIGHT
    if (fingerStates[0] == BENT && fingerStates[1] == STRAIGHT && fingerStates[2] == STRAIGHT && fingerStates[3] == STRAIGHT) {
        performAction1();
    }
    // Gesture2: All BENT
    else if (fingerStates[0] == BENT && fingerStates[1] == BENT && fingerStates[2] == BENT && fingerStates[3] == BENT) {
        performAction2();
    }
    // Gesture3: Index HALF_STRAIGHT, others STRAIGHT
    else if (fingerStates[0] == HALF_STRAIGHT && fingerStates[1] == STRAIGHT && fingerStates[2] == STRAIGHT && fingerStates[3] == STRAIGHT) {
        performAction3();
    }
    // Gesture4: Index STRAIGHT, others BENT
    else if (fingerStates[0] == STRAIGHT && fingerStates[1] == BENT && fingerStates[2] == BENT && fingerStates[3] == BENT) {
        performAction4();
    }
    // Gesture5: All fingers HALF_BENT
    else if (fingerStates[0] == HALF_BENT && fingerStates[1] == HALF_BENT && fingerStates[2] == HALF_BENT && fingerStates[3] == HALF_BENT) {
        performAction5();
    }
    // Add more gesture conditions as needed
}

// Perform actions based on recognized gestures
void performAction1() {
    Serial.println("Performing Action 1: Gesture 1 Detected");
    if (!previousGestureStates[0]) { // Ensure the gesture wasn't previously detected
        mp3.play(6); // Play track 006.mp3
        lcd.setCursor(0, 0);
        lcd.print("Gesture 1      ");
        previousGestureStates[0] = true; // Set flag
    }
}

void performAction2() {
    Serial.println("Performing Action 2: Gesture 2 Detected");
    if (!previousGestureStates[1]) {
        mp3.play(2); // Play track 002.mp3
        lcd.setCursor(0, 0);
        lcd.print("Gesture 2      ");
        previousGestureStates[1] = true;
    }
}

void performAction3() {
    Serial.println("Performing Action 3: Gesture 3 Detected");
    if (!previousGestureStates[2]) {
        mp3.play(3); // Play track 003.mp3
        lcd.setCursor(0, 0);
        lcd.print("Gesture 3      ");
        previousGestureStates[2] = true;
    }
}

void performAction4() {
    Serial.println("Performing Action 4: Gesture 4 Detected");
    if (!previousGestureStates[3]) {
        mp3.play(4); // Play track 004.mp3
        lcd.setCursor(0, 0);
        lcd.print("Gesture 4      ");
        previousGestureStates[3] = true;
    }
}

// New function for Gesture5
void performAction5() {                                       ////
    Serial.println("Performing Action 5: Gesture 5 Detected");       
    if (!previousGestureStates[4]) { // Index 4 for Gesture5////
        String phrase = "I want go to the bath";
        lcd.setCursor(0, 0);
        lcd.print("Gesture 5      ");
        lcd.setCursor(0, 1);
        lcd.print("                "); // Clear the second line
        startScroll(phrase); // Start scrolling the phrase
        previousGestureStates[4] = true; // Set flag
    }
}

// Reset gesture flags if no gesture is matched
void resetGestureFlags() {
    bool gestureMatched = false;

    // Check gesture matched
    ///////////y1
    if (fingerStates[0] == BENT && fingerStates[1] == STRAIGHT && fingerStates[2] == STRAIGHT && fingerStates[3] == STRAIGHT) {
        gestureMatched = true;
    }
    //--------/d2
    else if (fingerStates[0] == BENT && fingerStates[1] == BENT && fingerStates[2] == BENT && fingerStates[3] == BENT) {
        gestureMatched = true;
    }
    //-------/u3
    else if (fingerStates[0] == HALF_STRAIGHT && fingerStates[1] == STRAIGHT && fingerStates[2] == STRAIGHT && fingerStates[3] == STRAIGHT) {
        gestureMatched = true;
    }
    // 4h w
    else if (fingerStates[0] == STRAIGHT && fingerStates[1] == BENT && fingerStates[2] == BENT && fingerStates[3] == BENT) {
        gestureMatched = true;
    }
    //5 h
    else if (fingerStates[0] == HALF_BENT && fingerStates[1] == HALF_BENT && fingerStates[2] == HALF_BENT && fingerStates[3] == HALF_BENT) {
        gestureMatched = true;
    }

    // no gesture is matched/ reset previous gesture flags and update LCD
    if (!gestureMatched) {
        for (int i = 0; i < 5; i++) { // Updated to 5 gestures
            previousGestureStates[i] = false;
        }
        // Update only the gesture display line
        lcd.setCursor(0, 0);
        lcd.print("No Gesture     ");
    }
}

// -------------------- Scrolling Text Functions lcd txt--------------------

// Start scrolling txt on the second line
void startScroll(String text) {
    scrollTextBuffer = text;
    scrollPosition = 0;
    scrolling = true;
    scrollPreviousMillis = millis();
}

// Update scrolling txt
void updateScroll() {
    unsigned long currentMillis = millis();
    if (currentMillis - scrollPreviousMillis >= scrollInterval) {
        scrollPreviousMillis = currentMillis;

        // Scroll two characters to the left
        if (scrollPosition < scrollTextBuffer.length()) {
          ////
            int endPos = scrollPosition + 16;
            if (endPos > scrollTextBuffer.length()) {
                endPos = scrollTextBuffer.length();
            }
            String displayText = scrollTextBuffer.substring(scrollPosition, endPos);
            
            // Pad with spaces if necessary
            while (displayText.length() < 16) {
                displayText += " ";
            }
            lcd.setCursor(0, 1);
            lcd.print(displayText);
            
            // 2faster scrolling
            scrollPosition += 2;
        } else {
            // Stop scrolling
            scrolling = false;
        }
    }
}
