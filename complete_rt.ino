#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal_I2C.h>
#include <dht11.h>
#include <Wire.h>
#include <semphr.h>
#include <task.h> 

LiquidCrystal_I2C lcd(0x27,16,2);
SemaphoreHandle_t semaphoreContinuu;
SemaphoreHandle_t semaphoreRef;
SemaphoreHandle_t semaphoreBut;
//TaskHandle_t Tasktemp_ref_Handle; Nu functioneaza eTaskGetState :(((
//TaskHandle_t Tasktemp_continuu_Handle;
bool Tasktemp_ref_running = false;//V2
#define DHT11PIN 2

volatile float valorSensorTemperatura;
volatile int T = 22;
volatile bool vizibil = false;
void Taskprint( void *pvParameters );
dht11 DHT11;
const int trigPin = 9;
const int echoPin = 10;
long duration;
int distance;



void setup() {
    Serial.begin(9600);
    Serial.println(F("Start!"));
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    semaphoreContinuu = xSemaphoreCreateBinary();
    semaphoreRef = xSemaphoreCreateBinary();
    semaphoreBut = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphoreContinuu);
    pinMode(50, OUTPUT);
    pinMode(51, INPUT);
    pinMode(52, INPUT);
    pinMode(53, INPUT);
    lcd.init();
    xTaskCreate(Taskprint,"task2",configMINIMAL_STACK_SIZE,NULL, 0, NULL);
    xTaskCreate(tarefaSensorTemperatura, "TarefaSensorTemperatura", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Tasktemp_ref,"task3_ref",configMINIMAL_STACK_SIZE,NULL, 2, NULL);
    xTaskCreate(Tasktemp_continuu,"task3_ct",configMINIMAL_STACK_SIZE,NULL, 3, NULL);
    xTaskCreate(Taskbutton,"task4",configMINIMAL_STACK_SIZE,NULL, 4, NULL);
    xTaskCreate(Taskreferinta,"task5",configMINIMAL_STACK_SIZE,NULL, 5, NULL);
    xTaskCreate(Taskdist,"task6",configMINIMAL_STACK_SIZE,NULL, 6, NULL);
    vTaskStartScheduler();
}

void loop() {
}
void Taskreferinta(void *pvParameters) {
  while (1) {
    int sensorValue = analogRead(A0);
    T = 18 + 0.0098 * (sensorValue + 1 );
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
int ok = 0;
void Taskdist(void *pvParameters) {
    while (1) {
    digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2;
        
        if (distance < 20) {
          ok = 1;
        } else {
          ok = 0;
          digitalWrite(50, HIGH);
          xSemaphoreTake(semaphoreBut, 0);
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

void tarefaSensorTemperatura(void *pvParameters) {
  while (1) {
    if (ok=1) {
      int chk = DHT11.read(DHT11PIN);
      valorSensorTemperatura = DHT11.temperature;
      xSemaphoreGive(semaphoreBut); // Give back the semaphore to allow Tasktemp_continuu to resume'
      vTaskDelay(50000 / portTICK_PERIOD_MS);
    }
}


void Taskprint(void *pvParameters)  {
  while(1)
  { 
    if(vizibil == true){
      if(distance < 20 ){
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(0,0);
        lcd.print("Target:");
        lcd.setCursor(7,0);
        lcd.print(T);
        lcd.setCursor(0,1);
        lcd.print("Temp:");
        lcd.setCursor(5,1);
        lcd.print(valorSensorTemperatura);
        lcd.setCursor(11,1);
        lcd.print("M:");
        lcd.setCursor(13,1);
        if (Tasktemp_ref_running) {
            lcd.print("REF");
        } else {
            lcd.print("CON");
        }
        //Serial.println(F("ecran merge"));
      }
      else{
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(0,0);
        lcd.print("Nu este nimeni");
        lcd.setCursor(0,1);
        lcd.print("in camera");
      }
    }else{
      lcd.clear();
      lcd.noBacklight();
    }
  vTaskDelay(800 / portTICK_PERIOD_MS);

}
}

void Tasktemp_ref(void *pvParameters) {
  while(1) {
    // Wait for the semaphore to be taken
    if(xSemaphoreTake(semaphoreRef, portMAX_DELAY) == pdTRUE ) {
      Serial.println(T);
      Tasktemp_ref_running = true;
      // Run continuously until interrupted
      while(Tasktemp_ref_running) {
        if(valorSensorTemperatura < T+1 && xSemaphoreTake(semaphoreBut, portMAX_DELAY) == pdTRUE) {
          digitalWrite(50, LOW);
          //Serial.println(F("am activat"));
        }
        if(valorSensorTemperatura > T) {   
          digitalWrite(50, HIGH);
          //Serial.println(F("am dezactivat"));
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Add a small delay to avoid tight loop
      } 
    }
    Serial.println(F("urmaritul referintei merge"));
    Serial.println(T);
  }
}


void Tasktemp_continuu(void *pvParameters) {
  while(1) {
    if(xSemaphoreTake(semaphoreContinuu, portMAX_DELAY) == pdTRUE ) {
      Serial.println("s-a facut fals");
      Tasktemp_ref_running = false;
      while(Tasktemp_ref_running == false) {
        Serial.println(F("am intrat in continuu"));
        if(valorSensorTemperatura < 35 && xSemaphoreTake(semaphoreBut, portMAX_DELAY) == pdTRUE) {
          
          digitalWrite(50, LOW);
          
        }
        if(valorSensorTemperatura > 35) {
          
          digitalWrite(50, HIGH);
          //Serial.println(F("am dezactivat"));
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Add a small delay to avoid tight loop
      }
      
    }
  }
}

void Taskbutton(void *pvParameters) {
  int buton1;
  int buton2;
  int buton3;
  bool previousState = false; 
  while(1){
    buton1 = digitalRead(51);
    buton2 = digitalRead(52);
    buton3 = digitalRead(53);
    if(buton1 == HIGH && distance < 20){
      Tasktemp_ref_running = true;
      xSemaphoreGive(semaphoreRef); // Eliberează semaforul pentru Tasktemp_ref
      xSemaphoreTake(semaphoreContinuu, 0); // Blochează semaforul pentru Tasktemp_continuu
      Serial.println(F("buton1 apasat"));
  }
    if(buton2 == HIGH && distance < 20){
      Tasktemp_ref_running = false;
      xSemaphoreGive(semaphoreContinuu); // Eliberează semaforul pentru Tasktemp_continuu
      xSemaphoreTake(semaphoreRef, 0); // Blochează semaforul pentru Tasktemp_ref
      Serial.println(F("buton2 apasat"));
    }
    if(buton3 == HIGH){
      if (!previousState) {
        vizibil = !vizibil; // Invertează starea vizibil dacă butonul 3 este apăsat
        previousState = true; // Actualizează starea anterioară a butonului 3
      }
    } else {
      previousState = false; // Actualizează starea anterioară a butonului 3 când butonul nu este apăsat
    }
      vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}