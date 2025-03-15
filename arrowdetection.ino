#include "Adafruit_VL53L0X.h"
#include <math.h>

// Define unique addresses for each sensor
#define LOX1_ADDRESS 0x30 // Sensor 1 (Yellow, 0°)
#define LOX2_ADDRESS 0x31 // Sensor 2 (Green, 45°)
#define LOX3_ADDRESS 0x32 // Sensor 3 (Orange, 90°)
#define LOX4_ADDRESS 0x33 // Sensor 4 (Red, 135°)

// Define shutdown pins for each sensor
#define SHT_LOX1 7 // Sensor 1 (Yellow, 0°)
#define SHT_LOX2 6 // Sensor 2 (Green, 45°)
#define SHT_LOX3 5 // Sensor 3 (Orange, 90°)
#define SHT_LOX4 4 // Sensor 4 (Red, 135°)

// Board radius in mm
#define BOARD_RADIUS_MM 107.95 // 4.25 inches * 25.4 mm

// Define the recalculated maximum valid distance (subtracting 35 mm)
#define MAX_VALID_DISTANCE_MM (200 - 35) // Adjusted max valid distance

// Sensor positions for 0°, 45°, 90°, 135° (relative to board center)
struct Sensor {
  double x, y;    // Sensor position
  uint16_t range; // Distance reading
};

Sensor sensors[4] = {
  {BOARD_RADIUS_MM, 0, 0},                          // 0° (right)
  {BOARD_RADIUS_MM / sqrt(2), BOARD_RADIUS_MM / sqrt(2), 0}, // 45° (top-right)
  {0, BOARD_RADIUS_MM, 0},                          // 90° (top)
  {-BOARD_RADIUS_MM / sqrt(2), BOARD_RADIUS_MM / sqrt(2), 0}  // 135° (top-left)
};

// Instantiate sensor objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;

// Function to initialize each sensor with unique addresses
void setID() {
  // Reset all sensors
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);

  // Initialize each sensor sequentially

  // Sensor 1 (Yellow, 0°)
  digitalWrite(SHT_LOX1, HIGH);
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to initialize sensor 1 (Yellow, 0°)"));
    while(1);
  }
  delay(10);

  // Sensor 2 (Green, 45°)
  digitalWrite(SHT_LOX2, HIGH);
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to initialize sensor 2 (Green, 45°)"));
    while(1);
  }
  delay(10);

  // Sensor 3 (Orange, 90°)
  digitalWrite(SHT_LOX3, HIGH);
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to initialize sensor 3 (Orange, 90°)"));
    while(1);
  }
  delay(10);

  // Sensor 4 (Red, 135°)
  digitalWrite(SHT_LOX4, HIGH);
  if(!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to initialize sensor 4 (Red, 135°)"));
    while(1);
  }
  delay(10);
}

// Function to read all four sensors
void read_four_sensors() {
  // Sensor 1 (Yellow, 0°)
  lox1.rangingTest(&measure1, false);
  sensors[0].range = (measure1.RangeStatus != 4) ? (measure1.RangeMilliMeter - 35) : 8190;

  // Sensor 2 (Green, 45°)
  lox2.rangingTest(&measure2, false);
  sensors[1].range = (measure2.RangeStatus != 4) ? (measure2.RangeMilliMeter - 35) : 8190;

  // Sensor 3 (Orange, 90°)
  lox3.rangingTest(&measure3, false);
  sensors[2].range = (measure3.RangeStatus != 4) ? (measure3.RangeMilliMeter - 35) : 8190;

  // Sensor 4 (Red, 135°)
  lox4.rangingTest(&measure4, false);
  sensors[3].range = (measure4.RangeStatus != 4) ? (measure4.RangeMilliMeter - 35) : 8190;

  // // Debugging output for sensor ranges
  // for (int i = 0; i < 4; i++) {
  //   Serial.print(F("Sensor "));
  //   Serial.print(i + 1);
  //   Serial.print(F(": "));
  //   Serial.print(sensors[i].range);
  //   Serial.print(F(" mm "));
  // }
  // Serial.println();
}

// Calculate arrow position and score
void processArrowData() {
  double x = 0, y = 0, weightSum = 0;
  int validSensorCount = 0;
  double smoothingFactor = 50.0; // Smoothing factor to reduce dramatic weighting

  for (int i = 0; i < 4; i++) {
    // Check if the sensor reading is valid and not detecting the board edge
    if (sensors[i].range > 0 && sensors[i].range < MAX_VALID_DISTANCE_MM) {
      double weight = 1.0 / (sensors[i].range + smoothingFactor); // Adjusted weight with smoothing
      x += sensors[i].x * weight;
      y += sensors[i].y * weight;
      weightSum += weight;
      validSensorCount++;
    }
  }

  if (weightSum > 0) {
    // Calculate weighted average position
    x /= weightSum;
    y /= weightSum;

    // Adjust for bullseye offset based on observed center (5, 85)
    x -= 30;  // Subtract 5 from x
    y -= 65; // Subtract 85 from y

    // Calculate distance from the adjusted center
    double radius = sqrt(pow(x, 2) + pow(y, 2));

    // Determine score
    int score = getScore(radius);

    // Output results
    Serial.print("Arrow Position (relative to center): (");
    Serial.print(x, 2);
    Serial.print(", ");
    Serial.print(y, 2);
    Serial.println(")");
    Serial.print("Radius from bullseye: ");
    Serial.println(radius, 2);
    Serial.print("Score: ");
    Serial.println(score);
  } else if (validSensorCount == 0) {
    // No valid sensors detected an arrow
    Serial.println("No arrow detected or readings indicate board edges.");
  } else {
    // Some sensors detected edge-like distances
    Serial.println("Uncertain detection; possible board edge misread.");
  }
}




int getScore(double radius) {
  // Recalibrated scoring based on adjusted distances
  if (radius <= 10.0) return 10; // Bullseye (within 40 mm radius)
  if (radius <= 35.0) return 7; // 20 - 30 mm
  if (radius <= 50.0) return 5;  // 40–70 mm radius
  if (radius <= 60.0) return 2; // 70–100 mm radius
  return 0; // Miss
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(1); }

  // Initialize shutdown pins
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);

  Serial.println(F("Initializing sensors..."));

  // Reset all sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  
  setID(); // Set unique IDs for each sensor
}

void loop() {
  read_four_sensors();
  processArrowData();
  delay(1000); // Adjust as needed
}
