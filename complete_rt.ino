#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal_I2C.h>
#include <dht11.h>
#include <Wire.h>
#include <semphr.h>
#include <task.h> 

LiquidCrystal_I2C lcd(0x27,16,2); // Initialize the LCD
SemaphoreHandle_t semaphoreContinuu; // Semaphore for continuous temperature task
SemaphoreHandle_t semaphoreRef; // Semaphore for reference temperature task
SemaphoreHandle_t semaphoreBut; // Semaphore for button task
bool Tasktemp_ref_running = false; // Flag to indicate if the reference temperature task is running

// Initialize pins for ultrasonic sensor
#define trigPin 9
#define echoPin 10
long duration;
int distance;

// Function prototypes
void Taskprint( void *pvParameters );
void tarefaSensorTemperatura(void *pvParameters);
void Tasktemp_ref(void *pvParameters);
void Tasktemp_continuu(void *pvParameters);
void Taskbutton(void *pvParameters);
void Taskreferinta(void *pvParameters);
void Taskdist(void *pvParameters);

volatile float valorSensorTemperatura; // Temperature sensor value
volatile int T = 22; // Initial reference temperature
volatile bool vizibil = false; // Flag to indicate LCD visibility
dht11 DHT11;

void setup() {
    Serial.begin(9600);
    Serial.println(F("Start!"));

    // Set up ultrasonic sensor pins
    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT);

    // Create semaphores
    semaphoreContinuu = xSemaphoreCreateBinary();
    semaphoreRef = xSemaphoreCreateBinary();
    semaphoreBut = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphoreContinuu); // Give the semaphore for continuous temperature task

    // Set up relay
    pinMode(50, OUTPUT);
    // Set up buttons
    pinMode(51, INPUT);
    pinMode(52, INPUT);
    pinMode(53, INPUT);
    lcd.init();

    // Create tasks
    xTaskCreate(Taskprint,"task2",configMINIMAL_STACK_SIZE,NULL, 0, NULL);
    xTaskCreate(tarefaSensorTemperatura, "TarefaSensorTemperatura", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Tasktemp_ref,"task3_ref",configMINIMAL_STACK_SIZE,NULL, 2, NULL);
    xTaskCreate(Tasktemp_continuu,"task3_ct",configMINIMAL_STACK_SIZE,NULL, 3, NULL);
    xTaskCreate(Taskbutton,"task4",configMINIMAL_STACK_SIZE,NULL, 4, NULL);
    xTaskCreate(Taskreferinta,"task5",configMINIMAL_STACK_SIZE,NULL, 5, NULL);
    xTaskCreate(Taskdist,"task6",configMINIMAL_STACK_SIZE,NULL, 6, NULL);
    vTaskStartScheduler(); // Start the scheduler
}

void loop() {
}

// Task to read reference temperature from a potentiometer
void Taskreferinta(void *pvParameters) {
  while (1) {
    int sensorValue = analogRead(A0);
    T = 18 + 0.0098 * (sensorValue + 1 );
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay
  }
}

// Task to detecting the presence of people
void Taskdist(void *pvParameters) {
    while (1) {
    digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2;
        
        // Check if someone is within the specified distance, else close the execution element
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

// Task to read temperature sensor values
void tarefaSensorTemperatura(void *pvParameters) {
  while (1) {
    if (ok=1) {
      int chk = DHT11.read(DHT11PIN);
      valorSensorTemperatura = DHT11.temperature;
      xSemaphoreGive(semaphoreBut); // Give back the semaphore to allow Tasktemp_continuu to resume
      vTaskDelay(50000 / portTICK_PERIOD_MS);
    }
}

// Task to print values on the LCD
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

// Task to control temperature based on reference
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
          //Serial.println(F("am activat elementul de executie"));
        }
        if(valorSensorTemperatura > T) {   
          digitalWrite(50, HIGH);
          //Serial.println(F("am dezactivat elementul de executie"));
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Add a small delay to avoid tight loop
      } 
    }
    Serial.println(F("urmaritul referintei merge"));
    Serial.println(T);
  }
}

// Task to control temperature continuously
void Tasktemp_continuu(void *pvParameters) {
  while(1) {
    if(xSemaphoreTake(semaphoreContinuu, portMAX_DELAY) == pdTRUE ) {
      Serial.println("s-a facut fals");
      Tasktemp_ref_running = false;
      while(Tasktemp_ref_running == false) {
        Serial.println(F("am intrat in continuu"));
        if(valorSensorTemperatura < 35 && xSemaphoreTake(semaphoreBut, portMAX_DELAY) == pdTRUE) {  //Add overheating protection
          
          digitalWrite(50, LOW);
          //Serial.println(F("am activat elementul de executie"));
        }
        if(valorSensorTemperatura > 35) {
          
          digitalWrite(50, HIGH);
          //Serial.println(F("am dezactiv atelementul de executie"));
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Add a small delay to avoid tight loop
      }
      
    }
  }
}

// Task to handle button presses
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
      xSemaphoreGive(semaphoreRef); // Release semaphore for Tasktemp_ref
      xSemaphoreTake(semaphoreContinuu, 0); // Block semaphore for Tasktemp_continuu
      Serial.println(F("buton1 apasat"));
  }
    if(buton2 == HIGH && distance < 20){
      Tasktemp_ref_running = false;
      xSemaphoreGive(semaphoreContinuu); // Release semaphore for Tasktemp_continuu
      xSemaphoreTake(semaphoreRef, 0); // Block semaphore for Tasktemp_ref
      Serial.println(F("buton2 apasat"));
    }
    if(buton3 == HIGH){
      if (!previousState) {
        vizibil = !vizibil; // Toggle visibility state if button 3 is pressed
        previousState = true; // Update previous state of button 3
      }
    } else {
      previousState = false; // Update previous state of button 3 when button is not pressed
    }
      vTaskDelay(500 / portTICK_PERIOD_MS); // Delay
  }
}
