#include <Arduino.h>
#include <vector>
#include <string>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "support_vectors.h"
#include "dual_coef.h"
#include "mean.h"
#include "scale.h"
#include "train_value.h"
#include "calculations.h"
#include <U8g2lib.h> 


using namespace std;

const int buzzerPin = 17;
int skipFlag = 0;
const char* ssid = "Thom Ho";
const char* password = "12346789";

bool isCancelled = 0;
const int cancelButton = 15;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const char* stringBuf;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/U8X8_PIN_NONE);
unsigned long lastScroll = 0;


void printAndScroll(const char * text, int maxCharsPerLine, bool isParallel = false);
void displaySetup();
void printWrappedTextUTF8(const char * text, int maxCharsPerLine, int startLine);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Update every minute

Adafruit_MPU6050 mpu;
char record[150];
bool start = true;
std::vector<std::string> data_vector;
unsigned long previousMillis = 0;
const long interval = 1000; // 1 second

void IRAM_ATTR cancelFall() {
  portENTER_CRITICAL_ISR(&mux);
  isCancelled = 1;
  portEXIT_CRITICAL_ISR(&mux);
}


void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  displaySetup();
  // Pin mode for cancel button mode change
  pinMode(cancelButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(cancelButton), cancelFall, CHANGE);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Connected to WiFi");

  // Initialize NTPClient
  timeClient.begin();

  delay(100);
  while (!Serial)
    delay(10);

  initializeMPU();
}

void viewInternetStatus() {
  if (WiFi.status() != WL_CONNECTED) {
    stringBuf = "(!) Không có kết nối với internet";
    printAndScroll(stringBuf, 12);
    stringBuf = "";
  }
  else {
    stringBuf = "Đã kết nối với internet";
    printAndScroll(stringBuf, 12);
    stringBuf = "";
  }
}

void viewFallingStatus() {
  stringBuf = "Báo động, té ngã!";
  printAndScroll(stringBuf, 12);
  stringBuf = "";
  delay(500);
}

void loop() {
  unsigned long currentMillis = millis();
  if (skipFlag == 1) {
    previousMillis = currentMillis;
  }

  viewInternetStatus();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendDataToPredict();
    unsigned long currentMillis = millis();
    previousMillis = currentMillis;
  }

  if (start) {
    logMPUData();
  }
}

void alarm(int duration) {
  int beepDuration = duration / 8; // Calculate the duration of each beep
  for (int i = 0; i < 8; i++) {
    if (isCancelled == true) {
      break;
    }
    digitalWrite(buzzerPin, HIGH); // Turn the buzzer on
    delay(beepDuration / 2); // Beep for half the beep duration
    digitalWrite(buzzerPin, LOW); // Turn the buzzer off
    delay(beepDuration / 2); // Pause for half the beep duration
  }
  skipFlag = 0;
}

void initializeMPU() {
  while (!mpu.begin()) {
    delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  delay(100);
}

void logMPUData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sprintf(record, "%010d;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f;%4.4f", 
          millis(),
          a.acceleration.x, a.acceleration.y, a.acceleration.z,
          g.gyro.x , g.gyro.y , g.gyro.z );

  data_vector.push_back(record);
  delay(5);
}

void sendDataToPredict() {
  if (!data_vector.empty()) {
    std::string data = "time;acc_x;acc_y;acc_z;gyro_x;gyro_y;gyro_z\n";
    for (const auto& row : data_vector) {
      data += row + "\n";
    }

    // print size ò data_vector
    Serial.println(data_vector.size());
    data_vector.clear();


    double value = predict(data, support_vectors, dual_coef, intercept, gammma, mean, scale);
    isCancelled = false;
    if (value > 0) {
      viewFallingStatus();
      skipFlag = 1;
      alarm(10000); // Run alarm for 10s
      if (isCancelled == false) {
        callCloudFunction();
      }
    }
  }

}

String getCurrentTime() {
  timeClient.update();
  time_t rawTime = timeClient.getEpochTime();
  struct tm * timeInfo = localtime(&rawTime);

  char timeString[40];
  sprintf(timeString, "%04d-%02d-%02d %02d:%02d:%02d",
          timeInfo->tm_year + 1900,
          timeInfo->tm_mon + 1,
          timeInfo->tm_mday,
          timeInfo->tm_hour,
          timeInfo->tm_min,
          timeInfo->tm_sec);

  return String(timeString);
}

void sendPostRequest(const String& timeString) {
  Serial.println(timeString);
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("https://asia-southeast1-falling-detection-3e200.cloudfunctions.net/handleFallDetection");
    http.addHeader("Content-Type", "application/json");

    // Create JSON body
    DynamicJsonDocument doc(256);
    doc["data"]["deviceID"] = "DEVICE01";
    doc["data"]["time"] = timeString;  // Real-time value

    String requestBody;
    serializeJson(doc, requestBody);

    // Send POST request
    int httpResponseCode = http.POST(requestBody);

    // Check the result
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(response);
    } else {
      Serial.print("Error sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // Close connection
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void callCloudFunction() {
  String currentTime = getCurrentTime();
  sendPostRequest(currentTime);
}


