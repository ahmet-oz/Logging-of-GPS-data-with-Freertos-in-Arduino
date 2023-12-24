#include <Arduino_FreeRTOS.h> 
#include <semphr.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <EEPROM.h>
#include <SD.h>


long debouncing_time = 150; 
volatile unsigned long last_micros;
volatile double Lang;
volatile double Lat;
volatile short date_[7];
long num = 0;

void GPS_Task(void *pvParameters);
void SDCARD_Task(void *pvParameters);
void LCD_Task(void *pvParameters);

void Task_Using();
void debounceInterrupt();
void LCD_OPEN();
void LCD_SHOW();
void LCD_SHOW_ERROR();
void SD_Write();
void EEPROM_LOG();

TaskHandle_t gps_;
TaskHandle_t sd_;
TaskHandle_t lcd_;
SemaphoreHandle_t mutex_1;
SemaphoreHandle_t interruptSemaphore; 

UBaseType_t uxHighWaterMark1;
UBaseType_t uxHighWaterMark2;
UBaseType_t uxHighWaterMark3;

LiquidCrystal_I2C lcd(0x27,16,2);
TinyGPSPlus gps;
File myFile;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(2,INPUT_PULLUP);

//Mutex is created.  
  mutex_1 = xSemaphoreCreateMutex(); 
  if (mutex_1 == NULL) { 
    Serial.println("Mutex can not be created"); 
  } 

//Semaphore is created for button.
  interruptSemaphore = xSemaphoreCreateBinary();
  if (interruptSemaphore != NULL) {
    attachInterrupt(digitalPinToInterrupt(2), debounceInterrupt, LOW);
  } else  Serial.println("Mutex can not be created");
  
  xTaskCreate(GPS_Task, "GPS_Task", 1024, NULL, 2, &gps_); // Remaining Size for Arduino Mega: almost 109*8
  xTaskCreate(SDCARD_Task, "SDCARD_Task", 4096, NULL, 1, &sd_); // Remaining Size for Arduino Mega: almost 228*8
  xTaskCreate(LCD_Task, "LCD_Task", 128, NULL, configMAX_PRIORITIES, &lcd_);  // Remaining Size for Arduino Mega: almost 69*8
}

void loop() {
}

void GPS_Task(void *pvParameters) {
  LCD_OPEN(); // Start State of LCD
  while(1) { 
    (void) pvParameters;
    
// GPS datas are received.
    while (Serial1.available() > 0){
      if(xSemaphoreTake(mutex_1, portMAX_DELAY)==pdTRUE){
        Task_Using();
        
        // Raw gps data is received from Sensor. Coordinate and date information is extracted from raw data.
        if (gps.encode(Serial1.read())){
          Lang = gps.location.lng();
          Lat = gps.location.lat();
          date_[0] = gps.date.day();
          date_[1] = gps.date.month();
          date_[2] = gps.date.year();
          date_[3] = gps.time.hour();
          date_[4] = gps.time.minute();
          date_[5] = gps.time.second();
          date_[6] = gps.time.centisecond();
        }

        // GPS data is checked, whether is correctly received from Sensor.
        if (millis() > 5000 && gps.charsProcessed() < 10){
          Serial.println("No GPS detected: check wiring.");
          LCD_SHOW_ERROR();
          while(true);
        }
        else if(millis() > 5000 || gps.charsProcessed() >= 10){
          LCD_SHOW(); // The Data is written on LCD.
          EEPROM_LOG(); // The lastest Data is logged to EEPROM. It's showed on Serial.
        }
        xSemaphoreGive(mutex_1);
      }
      Serial.println("GPS_Task is completed.");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  } 
}

void SDCARD_Task(void *pvParameters) { 
  if (!SD.begin(7)) {
    while(1);
  }
  SD.mkdir("data/");  // data Folder is created in SD Card.
  while(1) { 
    (void) pvParameters;
    
    if(xSemaphoreTake(mutex_1, portMAX_DELAY)==pdTRUE){ 
      myFile = SD.open("data/GPSdata.txt", FILE_WRITE); // GPSdata file is created in data folder.
      if (myFile) {
        SD_Write(); // The data is logged to SD Card.
        myFile.close();
      } 
      else {
        Serial.println("error opening file");
        while(1);
      }
      xSemaphoreGive(mutex_1);
    }
    Serial.println("SDCARD_Task is completed.");
    vTaskDelay(pdMS_TO_TICKS(1000));
  } 
}

void LCD_Task(void *pvParameters) { 
  pinMode(4,OUTPUT);
  while(1) { 
    (void) pvParameters;
  
    if (xSemaphoreTake(interruptSemaphore, portMAX_DELAY) == pdPASS) {
      digitalWrite(4,!digitalRead(4));  // Button(2.Pin) is set as toggle button. 

      // State of LCD is set according to State of toggle button.
      if(digitalRead(4)){
        lcd.backlight();
        lcd.display();          
      }
      else{
        lcd.noBacklight();
        lcd.noDisplay();
      }
    }
    Serial.println("LCD_Task is completed.");
  }
}


void LCD_OPEN(){
  lcd.begin();
  lcd.clear();  
  lcd.noBacklight();
  lcd.noDisplay();
}

void LCD_SHOW(){
  lcd.setCursor(0,0);
  lcd.print("Lat:");
  lcd.setCursor(0,1);
  lcd.print("Lang :");
  lcd.setCursor(7,0);
  lcd.print(Lat);
  lcd.setCursor(7,1);
  lcd.print(Lang);
}

void LCD_SHOW_ERROR(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("No GPS detected!");
}

void SD_Write(){
  num++;
  myFile.print(num);
  myFile.print(".  Lat: "); myFile.print(Lat,6);  myFile.print(" , Lang: ");  myFile.print(Lang,6);
  myFile.print("   ");
  myFile.print("Date/Time: ");  
  myFile.print(date_[0]);myFile.print("/");myFile.print(date_[1]);myFile.print("/");myFile.print(date_[2]);myFile.print(" ");
  if (date_[3] < 10) myFile.print("0");
  myFile.print(date_[3]);myFile.print(":");
  if (date_[4] < 10) myFile.print("0");
  myFile.print(date_[4]);myFile.print(":");
  if (date_[5] < 10) myFile.print("0");
  myFile.print(date_[5]);myFile.print(".");
  if (date_[6] < 10) myFile.print("0");
  myFile.println(date_[6]);
}

void debounceInterrupt(){
  // Debouncing is applied to button(2-Pin).
  if((long)(micros() - last_micros) >= debouncing_time * 1000){
    xSemaphoreGiveFromISR(interruptSemaphore, NULL);
    last_micros = micros();
  }
}

void EEPROM_LOG(){
  short adress=0;
  Serial.print("EEPROM:  ");
  EEPROM.put(adress,Lat);
  Serial.print(EEPROM.get(adress,Lat));
  adress+=sizeof(double);
  EEPROM.put(adress,Lang);
  Serial.print(" "); Serial.print(EEPROM.get(adress,Lang));
  adress+=sizeof(double);
  for(int i=0;i<7;i++){
    EEPROM.put(adress,date_[i]);
    Serial.print(" "); Serial.print(EEPROM.get(adress,date_[i]));
    adress+=sizeof(short);
  }
  Serial.println();
}

void Task_Using(){
    // Rest of arranged Size for each created Tasks is checked (Remaining Size).
    Serial.println("");
    uxHighWaterMark1 = uxTaskGetStackHighWaterMark(gps_);
    Serial.print("Remaining Size of GPS_Task: ");
    Serial.println(uxHighWaterMark1);
    uxHighWaterMark2 = uxTaskGetStackHighWaterMark(sd_);
    Serial.print("Remaining Size of SDCARD_Task: ");
    Serial.println(uxHighWaterMark2);
    uxHighWaterMark3 = uxTaskGetStackHighWaterMark(lcd_);
    Serial.print("Remaining Size of LCD_Task: ");
    Serial.println(uxHighWaterMark3);
    Serial.println("");
}
