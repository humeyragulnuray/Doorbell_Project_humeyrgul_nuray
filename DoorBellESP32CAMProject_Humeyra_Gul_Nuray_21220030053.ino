#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <driver/gpio.h>

// -------------------- WiFi + Telegram --------------------
const char* ssid     = "Gül iPhone";
const char* password = "********";

String chatId   = "6034247974";
String BOTtoken = "8529101918:AAHTWq_GQnJx-PO6u8bezWaremSeBKG9rYk";

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

// -------------------- Durumlar --------------------
bool sendPhoto = false;
String photoReason = "";

// ---------------- ESP32-CAM AI THINKER PIN TANIMLARI ----------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN      4
bool flashState = LOW;

// ---------------- MODÜLLER (Senin proje pinleri) ----------------
#define MOTION_PIN         13      // Hareket sensörü çıkışı -> GPIO13
#define BUTTON_PIN         12      // Buton -> GPIO12 (diğer ucu GND)
#define BUZZER_PIN         15      // Buzzer -> GPIO15 (diğer ucu GND)
#define LED_PIN            2       // LED -> GPIO2 (direnç üzerinden)

// ---------------- Hareket / spam kontrol ----------------
volatile bool motionDetected = false;
unsigned long lastMotionMs = 0;
const unsigned long motionCooldownMs = 8000; // 8 sn spam engel

// ---------------- Buton debounce + edge detect ----------------
static int lastBtnState = HIGH;
unsigned long lastButtonMs = 0;
const unsigned long buttonDebounceMs = 250;

// ---------------- Telegram polling ----------------
int botRequestDelay = 250;
unsigned long lastTimeBotRan = 0;

void handleNewMessages(int numNewMessages);
String sendPhotoTelegram();

// ---------------- Hareket kesmesi (ilk kod mantığı) ----------------
// İlk kodda GPIO13 için NEGEDGE kullanıyordun.
// Sensörün aktif LOW tetikliyorsa doğru seçim: NEGEDGE
static void IRAM_ATTR detectsMovementISR() {
  motionDetected = true;
}

// ---------------- Yardımcı fonksiyonlar ----------------
void beepOnce(int ms = 200) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}

void ringFeedback() {
  digitalWrite(LED_PIN, HIGH);
  beepOnce(200);
  delay(60);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  // Pinler
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // buton: GPIO12 - GND

  // Hareket sensörü pini (aktif LOW sensörler için genelde pullup gerekir)
  pinMode(MOTION_PIN, INPUT_PULLUP);

  // WiFi
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Baglaniliyor: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(150);
  }

  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  // ---------------- Kamera Ayarları ----------------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_XGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera baslatma hatasi 0x%x\n", err);
    delay(1000);
    ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_XGA);

  // ---------------- Hareket sensörü interrupt (ilk kod gibi) ----------------
  // GPIO ISR servis
  err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("ISR service kurulamadi 0x%x\n", err);
  }

  // Kesme ekle
  err = gpio_isr_handler_add((gpio_num_t)MOTION_PIN, (gpio_isr_t)detectsMovementISR, NULL);
  if (err != ESP_OK) {
    Serial.printf("Kesme ekleme basarisiz 0x%x\n", err);
  }

  // İlk koddaki gibi NEGEDGE (aktif LOW)
  err = gpio_set_intr_type((gpio_num_t)MOTION_PIN, GPIO_INTR_NEGEDGE);
  if (err != ESP_OK) {
    Serial.printf("Kesme modu ayarlama basarisiz 0x%x\n", err);
  }

  bot.sendMessage(chatId,
                  "ESP32-CAM hazir ✅\n"
                  "Buton: GPIO12\nLED: GPIO2\nBuzzer: GPIO15\nHareket: GPIO13\n"
                  "Komutlar: /start /foto /flash",
                  "");
}

// ------------------------------------------- LOOP ------------------------------------------- //
void loop() {
  // ---------------- BUTON (edge + debounce) ----------------
  int btn = digitalRead(BUTTON_PIN);
  if (lastBtnState == HIGH && btn == LOW && (millis() - lastButtonMs) > buttonDebounceMs) {
    lastButtonMs = millis();

    Serial.println("Butona basildi!");
    ringFeedback();

    bot.sendMessage(chatId, "🔘 Zile basildi! Kapida biri var! Fotoğraf gönderiyorum 📸", "");
    photoReason = "Buton";
    sendPhoto = true;
  }
  lastBtnState = btn;

  // ---------------- HAREKET ----------------
  if (motionDetected) {
    motionDetected = false;

    if (millis() - lastMotionMs > motionCooldownMs) {
      lastMotionMs = millis();

      Serial.println("Hareket algilandi!");
      ringFeedback();

      bot.sendMessage(chatId, "🚶 Hareket algilandi! Fotoğraf gönderiyorum 📸", "");
      photoReason = "Hareket";
      sendPhoto = true;
    } else {
      Serial.println("Hareket tetiklendi ama cooldown aktif (spam engel).");
    }
  }

  // ---------------- FOTO GÖNDER ----------------
  if (sendPhoto) {
    Serial.println("Foto hazirlaniyor: " + photoReason);
    sendPhotoTelegram();
    sendPhoto = false;
    photoReason = "";
  }

  // ---------------- TELEGRAM MESAJ DİNLE ----------------
  if (millis() > lastTimeBotRan + botRequestDelay) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}

// --------------------------------- TELEGRAM'A FOTO GÖNDER ---------------------------------- //
String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(500);
    return "Camera capture failed";
  }

  if (clientTCP.connect(myDomain, 443)) {
    String head =
      "--davutbay\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + chatId +
      "\r\n--davutbay\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\n"
      "Content-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--davutbay--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=davutbay");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t* fbBuf = fb->buf;
    size_t fbLen = fb->len;

    for (size_t n = 0; n < fbLen; n += 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      } else {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);
    esp_camera_fb_return(fb);

    int waitTime = 10000;
    long startTimer = millis();
    bool state = false;

    while ((startTimer + waitTime) > millis()) {
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state) getBody += c;

        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        } else if (c != '\r') {
          getAll += c;
        }
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }

    clientTCP.stop();
    Serial.println(getBody);
  } else {
    esp_camera_fb_return(fb);
    getBody = "Connected to api.telegram.org failed.";
    Serial.println(getBody);
  }

  return getBody;
}

// --------------------------------- GELEN MESAJLARI İŞLE ---------------------------------- //
void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != chatId) {
      bot.sendMessage(chat_id, "Gecersiz Kullanici", "");
      continue;
    }

    String text = bot.messages[i].text;

    if (text == "/start") {
      String welcome = "ESP32-CAM Telegram Bot ✅\n\n";
      welcome += "/foto  : Foto cek\n";
      welcome += "/flash : Flasi ac/kapat\n";
      welcome += "Buton(GPIO12) basinca ve Hareket(GPIO13) olunca foto gelir.\n";
      bot.sendMessage(chatId, welcome, "");

    } else if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      bot.sendMessage(chatId, flashState ? "Flash: ACIK" : "Flash: KAPALI", "");

    } else if (text == "/foto") {
      bot.sendMessage(chatId, "Komutla foto cekiyorum 📸", "");
      ringFeedback();
      photoReason = "Komut";
      sendPhoto = true;

    } else {
      bot.sendMessage(chatId, "Gecersiz komut. /start yaz.", "");
    }
  }
}
