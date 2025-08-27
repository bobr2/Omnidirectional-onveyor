// Unified script: receive motor commands, drive 4 motors, read 4 encoders, send sensor data
#include <Arduino.h>
#include <GyverMotor.h>

#define NUM_MOTORS 4
#define PULSES_PER_REV 670 // Пульсов на оборот

// Serial settings
const unsigned long SEND_INTERVAL = 1000; // 1 секунда для отправки данных
unsigned long lastSendTime = 0;

// Packet settings
#define PACKET_SIZE 12
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
bool syncing = false;

// Motor command values (parsed from packet)
int16_t motorValues[NUM_MOTORS] = {0, 0, 0, 0};

// Motor driver objects using GyverMotor
GMotor motors[NUM_MOTORS] = {
  GMotor(DRIVER2WIRE, 6, 7, HIGH),   // front-right
  GMotor(DRIVER2WIRE, 8, 9, HIGH),   // back-right
  GMotor(DRIVER2WIRE, 3, 11, HIGH), // back-left
  GMotor(DRIVER2WIRE, 12, 4, HIGH)  // front-left
};


// Encoder pins and counts
const uint8_t encoderPinA[4] = {18, 19, 20, 21}; // Канал A
const uint8_t encoderPinB[4] = {22, 17, 16, 15}; // Канал B
volatile long encoderCount[4] = {0, 0, 0, 0};
long lastEncoderCount[4] = {0, 0, 0, 0};
float currentSpeed[4] = {0, 0, 0, 0}; // Скорость в об/с

// PID control variables
const unsigned long CONTROL_INTERVAL = 10; // 10 мс (100 Гц) для точного управления
unsigned long lastControlTime = 0;
float Kp = 60.0, Ki = 10, Kd = 0;
float integral[NUM_MOTORS] = {0};
float lastError[NUM_MOTORS] = {0};
const float INTEGRAL_LIMIT = 100.0; // Ограничение интегральной составляющей

// Function declarations
void sendSensorData(Stream &s, uint16_t lt, uint16_t rt, uint16_t lb, uint16_t rb);
void parsePacket();
void readSerialPacket();
void setupEncoders();
void updateCurrentSpeed(float dt);
void updateMotorControl(uint8_t i, float dt);

void handleEncoder0();
void handleEncoder1();
void handleEncoder2();
void handleEncoder3();

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  // Инициализация моторов
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].setMode(AUTO);
  }

  // Настройка энкодеров
  setupEncoders();
}

void loop() {
  unsigned long now = millis();

  // Отправка данных каждую секунду
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;
    // Временные заглушки, замените на реальные данные (например, encoderCount или currentSpeed)
    //uint16_t lt = encoderCount[0] & 0xFFFF;
    //uint16_t rt = encoderCount[1] & 0xFFFF;
    //uint16_t lb = encoderCount[2] & 0xFFFF;
    //uint16_t rb = encoderCount[3] & 0xFFFF;
    //sendSensorData(Serial3, lt, rt, lb, rb);
  }

  // Чтение команд от Serial3
  readSerialPacket();

  // Обновление скорости и управление моторами
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    float dt = (now - lastControlTime) / 1000.0; // Время в секундах
    lastControlTime = now;

    noInterrupts(); // Защита от изменения encoderCount во время чтения
    updateCurrentSpeed(dt);
    interrupts();

    for (int i = 0; i < NUM_MOTORS; ++i) {
      updateMotorControl(i, dt);
    }
  }

}

void updateCurrentSpeed(float dt) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    long delta = encoderCount[i] - lastEncoderCount[i];
    currentSpeed[i] = (float)delta / PULSES_PER_REV / dt; // Скорость в об/с
    lastEncoderCount[i] = encoderCount[i];
  }
}

void updateMotorControl(uint8_t i, float dt) {
  float target = motorValues[i] / 10.0; // Целевая скорость в об/с
  float error = target - currentSpeed[i];
  integral[i] += error * dt;
  integral[i] = constrain(integral[i], -INTEGRAL_LIMIT, INTEGRAL_LIMIT); // Ограничение интеграла
  float derivative = (error - lastError[i]) / dt;
  float output = Kp * error + Ki * integral[i] + Kd * derivative;
  lastError[i] = error;

  output = constrain(output, -255, 255); // Ограничение ШИМ
  motors[i].setSpeed(output);

  // Логирование (опционально, можно отключить)

  Serial.print("Motor "); Serial.print(i);
  Serial.print(" | Target: "); Serial.print(target, 2);
  Serial.print(" | Speed: "); Serial.print(currentSpeed[i], 2);
  Serial.print(" | Error: "); Serial.print(error, 2);
  Serial.print(" | Integral: "); Serial.print(integral[i], 2);
  Serial.print(" | Derivative: "); Serial.print(derivative, 2);
  Serial.print(" | Output: "); Serial.println(output, 2);

} 

// Настройка энкодеров
void setupEncoders() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(encoderPinA[i], INPUT_PULLUP); // Встроенные подтягивающие резисторы
    pinMode(encoderPinB[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(encoderPinA[0]), handleEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[1]), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[2]), handleEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA[3]), handleEncoder3, CHANGE);
}

// Обработчики прерываний для энкодеров (по подходу из второго кода)
void handleEncoder0() {
  if (digitalRead(encoderPinB[0]) == digitalRead(encoderPinA[0])) {
    encoderCount[0]--;
  } else {
    encoderCount[0]++;
  }
}
void handleEncoder1() {
  if (digitalRead(encoderPinB[1]) == digitalRead(encoderPinA[1])) {
    encoderCount[1]--;
  } else {
    encoderCount[1]++;
  }
}
void handleEncoder2() {
  if (digitalRead(encoderPinB[2]) == digitalRead(encoderPinA[2])) {
    encoderCount[2]--;
  } else {
    encoderCount[2]++;
  }
}
void handleEncoder3() {
  if (digitalRead(encoderPinB[3]) == digitalRead(encoderPinA[3])) {
    encoderCount[3]--;
  } else {
    encoderCount[3]++;
  }
}

void readSerialPacket() {
  while (Serial3.available()) {
    uint8_t byteIn = Serial3.read();
    if (!syncing) {
      if (byteIn == 0xAA) {
        rxBuffer[0] = byteIn;
        rxIndex = 1;
        syncing = true;
      }
    } else {
      rxBuffer[rxIndex++] = byteIn;
      if (rxIndex == PACKET_SIZE) {
        parsePacket();
        syncing = false;
        rxIndex = 0;
      }
    }
  }
}

void parsePacket() {
  uint8_t crc = 0;
  for (int i = 1; i <= 10; i++) crc ^= rxBuffer[i];
  if (rxBuffer[11] != crc) {
    Serial.println("CRC error");
    return;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    uint8_t hi = rxBuffer[3 + i*2];
    uint8_t lo = rxBuffer[4 + i*2];
    motorValues[i] = (int16_t)((hi << 8) | lo);
  }
  Serial.print("Received motorValues: ");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(motorValues[i]);
    if (i < NUM_MOTORS - 1) Serial.print(", ");
  }
  Serial.println();
}

void sendSensorData(Stream &s, uint16_t lt, uint16_t rt, uint16_t lb, uint16_t rb) {
  uint8_t packet[12];
  packet[0] = 0xAA;
  packet[1] = 0x03;
  packet[2] = 0x10;
  packet[3] = lt >> 8; packet[4] = lt & 0xFF;
  packet[5] = rt >> 8; packet[6] = rt & 0xFF;
  packet[7] = lb >> 8; packet[8] = lb & 0xFF;
  packet[9] = rb >> 8; packet[10] = rb & 0xFF;
  uint8_t crc = 0;
  for (int i = 1; i <= 10; i++) crc ^= packet[i];
  packet[11] = crc;
  s.write(packet, 12);
}