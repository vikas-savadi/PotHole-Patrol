#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <arduinoFFT.h>

// ============= PIN DEFINITIONS =============
#define TRIG_PIN 18
#define ECHO_PIN 19
#define MPU_SDA 21
#define MPU_SCL 22

// ============= CONFIGURATION =============
#define SAMPLES 128              // FFT samples
#define SAMPLING_FREQUENCY 100   // Hz
#define GROUND_CLEARANCE 4.5     // Sensor height above ground in cm

// Severity multipliers (BREADBOARD TESTING - Balanced Sensitivity)
#define YELLOW_MULTIPLIER 3.5    // 3.5x baseline = Yellow (light tap)
#define ORANGE_MULTIPLIER 5.5    // 5.5x baseline = Orange (medium tap)  
#define RED_MULTIPLIER 8.5       // 8.5x baseline = Red (hard tap)

#define DETECTION_WINDOW 3000    // 3 seconds window for ONE detection
#define COOLDOWN_PERIOD 2000     // 2 seconds cooldown

// ============= OBJECTS =============
MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// ============= FFT VARIABLES =============
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
unsigned long lastSampleTime = 0;
int sampleIndex = 0;

float baselineAmplitude = 0;
float currentAmplitude = 0;
int calibrationCount = 0;
bool isCalibrated = false;

// MPU6050 calibration offsets
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;

// Detection window tracking
bool inDetectionWindow = false;
unsigned long windowStartTime = 0;
unsigned long lastSpikeTime = 0;
int spikeCount = 0;
float maxAmplitude = 0;
String maxSeverity = "";
float potholeDistance = 0;  // Store distance at first spike

// Test mode
unsigned long lastInfoPrint = 0;

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin(MPU_SDA, MPU_SCL);
  
  delay(1000);
  
  // Print header
  Serial.println("\n");
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║   POTHOLE DETECTION - BREADBOARD TEST      ║");
  Serial.println("║         Hardware Verification Mode         ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize MPU6050
  Serial.println("🔧 INITIALIZING SENSORS...");
  Serial.println("─────────────────────────────────────────────");
  Serial.print("MPU6050: ");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("❌ FAILED!");
    Serial.println("\n⚠  Check connections:");
    Serial.println("   • SDA → GPIO 21");
    Serial.println("   • SCL → GPIO 22");
    Serial.println("   • VCC → 3.3V");
    Serial.println("   • GND → GND");
    while(1);
  }
  Serial.println("✅ OK");
  
  // Configure MPU6050
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g range
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);
  
  // Calibrate MPU6050 offsets
  Serial.print("Calibrating: ");
  Serial.println("Keep STILL for 3 seconds...");
  delay(2000);
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  for(int i = 0; i < 100; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += (az - 16384); // Remove gravity (1g = 16384 at ±2g)
    delay(10);
  }
  
  ax_offset = ax_sum / 100;
  ay_offset = ay_sum / 100;
  az_offset = az_sum / 100;
  
  Serial.println("✅ MPU6050 Calibrated");
  
  // Initialize ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.print("Ultrasonic: ");
  
  float testDist = measureDistance();
  if (testDist > 0 && testDist < 400) {
    Serial.println("✅ OK (" + String(testDist, 1) + " cm)");
  } else {
    Serial.println("⚠  WARNING (Check wiring)");
  }
  
  // GPS Status
  Serial.print("GPS Module: ");
  Serial.println("✅ Connected (Serial2)");
  
  Serial.println("─────────────────────────────────────────────");
  
  // Calculate sampling period
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  // Baseline calibration
  Serial.println("\n📋 BASELINE CALIBRATION STARTING...");
  Serial.println("⚠  Keep breadboard STILL and QUIET");
  Serial.println("⏳ Collecting 30 samples...\n");
  delay(2000);
}

// ============= MAIN LOOP =============
void loop() {
  unsigned long currentTime = millis();
  
  // Read GPS data
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }
  
  // Sample accelerometer at fixed rate
  if (micros() - lastSampleTime >= samplingPeriod) {
    lastSampleTime = micros();
    
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Remove offsets and convert to g-force
    float accelZ = (az - az_offset) / 16384.0; // ±2g = 16384 LSB/g
    
    // Store absolute value (magnitude, not direction)
    vReal[sampleIndex] = abs(accelZ);
    vImag[sampleIndex] = 0;
    sampleIndex++;
    
    // Perform FFT when buffer full
    if (sampleIndex >= SAMPLES) {
      performFFTAnalysis();
      sampleIndex = 0;
    }
  }
  
  // Print test instructions every 8 seconds
  if (isCalibrated && currentTime - lastInfoPrint >= 8000) {
    lastInfoPrint = currentTime;
    printTestInstructions();
  }
  
  // Check if detection window ended
  if (inDetectionWindow) {
    unsigned long timeSinceLastSpike = currentTime - lastSpikeTime;
    if (timeSinceLastSpike >= DETECTION_WINDOW) {
      endDetectionWindow();
    }
  }
}

// ============= FFT ANALYSIS =============
void performFFTAnalysis() {
  // Perform FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  
  // Calculate average magnitude in vibration range
  float totalMagnitude = 0;
  int count = 0;
  
  // Focus on vibration frequencies (5-30 Hz)
  int startBin = 5;
  int endBin = 30;
  
  for (int i = startBin; i < endBin && i < (SAMPLES/2); i++) {
    totalMagnitude += vReal[i];
    count++;
  }
  
  currentAmplitude = (count > 0) ? (totalMagnitude / count) : 0;
  
  // CALIBRATION PHASE
  if (!isCalibrated) {
    if (calibrationCount < 30) {
      baselineAmplitude += currentAmplitude;
      calibrationCount++;
      
      if (calibrationCount % 10 == 0) {
        Serial.println("Calibrating... " + String(calibrationCount) + "/30");
      }
      
      if (calibrationCount == 30) {
        baselineAmplitude = baselineAmplitude / 30.0;
        isCalibrated = true;
        
        Serial.println("\n✅ CALIBRATION COMPLETE!\n");
        Serial.println("📊 Baseline Statistics:");
        Serial.println("   Amplitude: " + String(baselineAmplitude, 4));
        Serial.println("\n🎯 Detection Thresholds:");
        Serial.println("   🟡 Yellow: " + String(baselineAmplitude * YELLOW_MULTIPLIER, 4) + " (" + String(YELLOW_MULTIPLIER, 1) + "x baseline)");
        Serial.println("   🟠 Orange: " + String(baselineAmplitude * ORANGE_MULTIPLIER, 4) + " (" + String(ORANGE_MULTIPLIER, 1) + "x baseline)");
        Serial.println("   🔴 Red:    " + String(baselineAmplitude * RED_MULTIPLIER, 4) + " (" + String(RED_MULTIPLIER, 1) + "x baseline)");
        Serial.println("\n" + String('─', 45));
        Serial.println("✅ SYSTEM READY FOR TESTING!");
        Serial.println(String('─', 45) + "\n");
        
        printTestInstructions();
      }
    }
    return;
  }
  
  // Plot graph
  plotGraph();
  
  // SPIKE DETECTION
  float yellowThreshold = baselineAmplitude * YELLOW_MULTIPLIER;
  
  if (currentAmplitude > yellowThreshold) {
    handleSpike();
  }
}

// ============= PLOT GRAPH =============
void plotGraph() {
  // Calculate detection threshold (middle threshold for Indian road conditions)
  // Using Orange threshold as reference since Indian roads have natural bumps
  float detectionThreshold = baselineAmplitude * ORANGE_MULTIPLIER;
  
  // Apply moving average for smooth continuous line
  static float smoothBuffer[20] = {0};
  static int smoothIndex = 0;
  static bool bufferFilled = false;
  
  // Update circular buffer
  smoothBuffer[smoothIndex] = currentAmplitude;
  smoothIndex++;
  
  if (smoothIndex >= 20) {
    smoothIndex = 0;
    bufferFilled = true;
  }
  
  // Calculate smooth average with weighted moving average
  float smoothSpike = 0;
  int count = bufferFilled ? 20 : smoothIndex;
  
  for (int i = 0; i < count; i++) {
    smoothSpike += smoothBuffer[i];
  }
  smoothSpike = (count > 0) ? (smoothSpike / count) : currentAmplitude;
  
  // Scale values for better visualization (multiply by 1000 for better Y-axis scale)
  float scaledSpike = smoothSpike * 1000.0;
  float scaledThreshold = detectionThreshold * 1000.0;
  float scaledBaseline = baselineAmplitude * 1000.0;
  
  // Plot 3 continuous lines with scaled values
  Serial.print("Vibration:");
  Serial.print(scaledSpike, 2);
  Serial.print(",");
  
  Serial.print("Threshold:");
  Serial.print(scaledThreshold, 2);
  Serial.print(",");
  
  Serial.print("Baseline:");
  Serial.println(scaledBaseline, 2);
}

// ============= HANDLE SPIKE =============
// *** MODIFIED FUNCTION ***
void handleSpike() {
  unsigned long currentTime = millis();
  
  // START new detection window
  if (!inDetectionWindow) {
    inDetectionWindow = true;
    windowStartTime = currentTime;
    spikeCount = 0;
    maxAmplitude = 0;
    maxSeverity = "";
    
    // *** FIX: Measure distance at the start of the impact ***
    potholeDistance = measureDistance(); 
    
    Serial.println("\n");
    Serial.println("╔════════════════════════════════════════════╗");
    Serial.println("║         🚨 IMPACT DETECTED!                ║");
    Serial.println("╚════════════════════════════════════════════╝");
    // Added a debug print to confirm the distance was read
    Serial.println("   Distance at impact: " + String(potholeDistance, 1) + " cm"); 
  }
  
  // Update tracking
  lastSpikeTime = currentTime;
  spikeCount++;
  
  // Track max amplitude
  if (currentAmplitude > maxAmplitude) {
    maxAmplitude = currentAmplitude;
  }
  
  // Determine severity
  float yellowThreshold = baselineAmplitude * YELLOW_MULTIPLIER;
  float orangeThreshold = baselineAmplitude * ORANGE_MULTIPLIER;
  float redThreshold = baselineAmplitude * RED_MULTIPLIER;
  
  String currentSeverity = "";
  if (currentAmplitude >= redThreshold) {
    currentSeverity = "RED";
  } else if (currentAmplitude >= orangeThreshold) {
    currentSeverity = "ORANGE";
  } else if (currentAmplitude >= yellowThreshold) {
    currentSeverity = "YELLOW";
  }
  
  // Keep worst severity
  if (maxSeverity == "" || 
      (currentSeverity == "RED") ||
      (currentSeverity == "ORANGE" && maxSeverity != "RED") ||
      (currentSeverity == "YELLOW" && maxSeverity == "")) {
    maxSeverity = currentSeverity;
  }
}

// ============= END DETECTION WINDOW =============
// *** MODIFIED FUNCTION ***
void endDetectionWindow() {
  inDetectionWindow = false;
  
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║         📊 DETECTION REPORT                ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  // Impact statistics
  Serial.println("📈 IMPACT STATISTICS:");
  Serial.println("   Spikes detected: " + String(spikeCount));
  Serial.println("   Peak amplitude: " + String(maxAmplitude, 4));
  Serial.println("   Baseline: " + String(baselineAmplitude, 4));
  Serial.println("   Multiplier: " + String(maxAmplitude / baselineAmplitude, 1) + "x baseline");
  Serial.println();
  
  // Severity
  String severityText = "";
  String emoji = "";
  String description = "";
  
  float yellowThreshold = baselineAmplitude * YELLOW_MULTIPLIER;
  float orangeThreshold = baselineAmplitude * ORANGE_MULTIPLIER;
  float redThreshold = baselineAmplitude * RED_MULTIPLIER;
  
  // *** FIX: Use the distance saved at the start of the impact ***
  float distance = potholeDistance;
  bool isSpeedBreaker = false;

  // --- CONSOLIDATED ULTRASONIC LOGIC ---
  Serial.println("📏 ULTRASONIC READING:");
  
  if (distance > 0) {
    Serial.println("   Distance: " + String(distance, 1) + " cm"); // Fixed unit to cm
    
    // Use consistent 2.0 cm threshold
    if (distance > GROUND_CLEARANCE + 2.0) { 
      float depth = distance - GROUND_CLEARANCE;
      Serial.println("   Type: 🕳  POTHOLE DETECTED");
      Serial.println("   Depth: " + String(depth, 1) + " cm below ground level"); // Fixed unit to cm
    } else if (distance < GROUND_CLEARANCE - 2.0) { 
      float height = GROUND_CLEARANCE - distance;
      Serial.println("   Type: 🚧 SPEED BREAKER DETECTED");
      Serial.println("   Height: " + String(height, 1) + " cm above ground level"); // Fixed unit to cm
      isSpeedBreaker = true; // Set flag here
    } else {
      Serial.println("   Type: Minor surface irregularity");
    }
  } else {
    Serial.println("   Distance: Sensor error");
  }
  Serial.println();
  // --- END CONSOLIDATED LOGIC ---
  
  // Only show severity for potholes and road anomalies, not speed breakers
  if (!isSpeedBreaker) {
    if (maxSeverity == "RED") {
      severityText = "HIGH";
      emoji = "🔴";
      description = "Hard impact - Pothole/Anomaly detected";
    } else if (maxSeverity == "ORANGE") {
      severityText = "MODERATE";
      emoji = "🟠";
      description = "Medium impact - Pothole/Anomaly detected";
    } else if (maxSeverity == "YELLOW") {
      severityText = "LOW";
      emoji = "🟡";
      description = "Light impact - Pothole/Anomaly detected";
    }
    
    // Only print severity if one was detected
    if (severityText != "") { 
      Serial.println("⚠  SEVERITY: " + emoji + " " + severityText);
      Serial.println("   " + description);
      Serial.println("   Threshold: " + String(maxAmplitude, 4) + " / " + 
                     String(maxSeverity == "RED" ? redThreshold : 
                            maxSeverity == "ORANGE" ? orangeThreshold : yellowThreshold, 4));
      Serial.println();
    }
  }
  
  // GPS status
  Serial.println("📍 GPS STATUS:");
  if (gps.location.isValid()) {
    Serial.println("   ✅ GPS Lock Acquired");
    Serial.println("   Latitude: " + String(gps.location.lat(), 7));
    Serial.println("   Longitude: " + String(gps.location.lng(), 7));
    
    if (gps.satellites.isValid()) {
      Serial.println("   Satellites: " + String(gps.satellites.value()));
    }
    
    if (gps.hdop.isValid()) {
      Serial.println("   Accuracy: " + String(gps.hdop.hdop(), 2));
    }
    
    if (gps.date.isValid() && gps.time.isValid()) {
      Serial.print("   Time: ");
      if (gps.date.day() < 10) Serial.print("0");
      Serial.print(gps.date.day());
      Serial.print("/");
      if (gps.date.month() < 10) Serial.print("0");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.print(gps.date.year());
      Serial.print(" ");
      if (gps.time.hour() < 10) Serial.print("0");
      Serial.print(gps.time.hour());
      Serial.print(":");
      if (gps.time.minute() < 10) Serial.print("0");
      Serial.print(gps.time.minute());
      Serial.print(":");
      if (gps.time.second() < 10) Serial.print("0");
      Serial.print(gps.time.second());
      Serial.println(" UTC");
    }
    
    Serial.println("   Map: https://maps.google.com/?q=" + 
                   String(gps.location.lat(), 7) + "," + 
                   String(gps.location.lng(), 7));
  } else {
    Serial.println("   ⚠  No GPS signal");
    Serial.println("   (Normal for indoor testing)");
  }
  
  Serial.println("\n" + String('─', 45));
  Serial.println("✅ Ready for next test\n");
  
  delay(500);
}

// ============= MEASURE DISTANCE =============
float measureDistance() {
  float distances[3];
  int validCount = 0;
  
  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    
    if (duration > 0 && duration < 25000) {
      float dist = (duration * 0.0343) / 2.0; // Distance in cm
      distances[validCount] = dist;
      validCount++;
    }
    delay(60);
  }
  
  if (validCount == 0) return 0;
  
  // Calculate average
  float sum = 0;
  for (int i = 0; i < validCount; i++) {
    sum += distances[i];
  }
  
  return sum / validCount;
}

// ============= PRINT TEST INSTRUCTIONS =============
void printTestInstructions() {
  Serial.println("\n" + String('═', 45));
  Serial.println("🧪 BREADBOARD TEST GUIDE");
  Serial.println(String('═', 45));
  Serial.println("\n💡 Try these tests:\n");
  Serial.println("   1️⃣  LIGHT TAP:");
  Serial.println("     Gently tap the breadboard/table");
  Serial.println("     Expected: 🟡 YELLOW severity\n");
  
  Serial.println("   2️⃣  MEDIUM TAP:");
  Serial.println("     Tap the breadboard harder");
  Serial.println("     Expected: 🟠 ORANGE severity\n");
  
  Serial.println("   3️⃣  HARD SHAKE:");
  Serial.println("     Shake the breadboard firmly");
  Serial.println("     Expected: 🔴 RED severity\n");
  
  Serial.println("   4️⃣  ULTRASONIC TEST:");
  Serial.println("     While tapping, place your hand");
  Serial.println("     ~2 cm from the sensor (for speed breaker)");
  Serial.println("     or ~10 cm away (for pothole)\n");
  
  Serial.println(String('─', 45));
  Serial.println("📊 Switch to Serial Plotter for live graph");
  Serial.println(String('─', 45) + "\n");
}