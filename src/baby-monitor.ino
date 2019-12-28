/*
 * Project baby-monitor
 * Description: cellular-enabled, noise triggered, cloud-connected baby monitor
 * Author: Matt Messinger
 * Date: December 2019
 */

#include <Adafruit_Sensor.h>
#include <Adafruit_ADT7410.h> // temp sensor
#include <Adafruit_ADXL343.h> // accel sensor
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // override "#define SSD1306_128_32" in Adafruit_SSD1306.h
#include <math.h>

// application behavior
#define SMART_CELLULAR 1  // when enabled reduces power consumption
#define USE_TEMP_SENSOR 1
#define USE_ACCEL_SENSOR 1
#define OUTPUT_OLED 1

double ACCEL_THRESHOLD = 3.0;  // arbitrary acceleration threshold to determine if car is moving
int MOVEMENT_DELAY_START = 1000 * 60 * 3; // this is the time to wait without movement before reacting to baby cry (accounting for stoplights, etc)
int MOVEMENT_DELAY_STOP = 1000 * 60 * 20; // this is the time to wait after movement to ignore sounds (to avoid false positives)
int CRYING_DELAY = 1000 * 60 * 5;  // this is the time that a crying signal remains valid to avoid sending multiple crying alerts
int LOOP_PERIOD_MS = 100;    // program loop speed
int LOG_PERIOD_SECONDS = 5;  // time between logging sensor data to serial out and OLED
int SENSOR_CHECK_PERIOD_SECONDS = 60 * 5; // time to check non-critical sensors and battery
int LOW_BATTERY_ALERT_THRESHOLD = 20;  // below this value will signal low battery to cloud

// parameters to retry Cellular/Particle connections
int PARTICLE_PUBLISH_RECONNECT_RETRIES = 5;  // number of times to retry publish by doing a cellular connect
int PARTICLE_PUBLISH_RETRIES = 10; // number of times to retry a Particle.publish() failure
int PARTICLE_PROCESS_RETRIES = 1;  // number of times to call Particle.process() when in Manual cellular mode
int PARTICLE_PROCESS_DELAY_MS = 1000; // number of milliseconds between Particle.process() calls
int CELLULAR_CONNECT_RETRY_LIMIT = 20; // number of times to connect to cellular before restarting modem
int CELLULAR_START_RETRY_LIMIT=5; // number of times to retry startCellular before rebooting the device

// battery info
long currentBatteryCharge = 0;
bool lowBatteryAlerted = false;

// sensor measurements
double tempC, tempF;
float currX, currY, currZ;
float lastX, lastY, lastZ;
unsigned long lastMovementTime = 0;
unsigned long lastCryingTime = 0;

enum AppState { DISABLED_MOVING, MONITORING, ALERT, DISABLED_IDLE  };
const String STATE_MONITORING_STR = "Status: Monitoring";
const String STATE_DISABLED_MOVING_STR = "Disabled: Moving";
const String STATE_DISABLED_IDLE_STR = "Disabled: Idle";
const String STATE_ALERT_STR = "Status: Alert";
const String STATE_UNKNOWN_TEMP_STR = "Unknown";

AppState prevState = MONITORING, currState = MONITORING;
bool moving = false;
bool crying = false;    // means signal has been determined as "cry"
bool monitoring = true; // means a "cry" will send an alert

String deviceId;
char buf[512]; // used to send a message to the cloud

// noise signal thresholds
int MIN_CRYING_THRESHOLD = 1000; // any sample over this value and greater than BABY_CRY_SIGMA is considered crying
int MAX_CRYING_THRESHOLD = 1800; // any sample over this value is considered "crying" even if it's not in SIGMA
float NOISE_REJECTION_SIGMA = 2; // any sample over this value is considered noise in one "sample buffer"
float BABY_CRY_SIGMA = 3;        // a "sample buffer" with mean greater than BABY_CRY_SIGMA is considered a "cry"

// curr noise buffer holds individual noise samples to be averaged
const int CURR_NOISE_BUFFER_SIZE = 10;
int currNoiseBuffer[CURR_NOISE_BUFFER_SIZE];
int currNoiseBufferIndex = 0;
float currNoiseBufferMean = 0;
float currNoiseBufferStd = 0;

// full noise buffer holds the averages of several curr noise buffers
const int FULL_NOISE_BUFFER_SIZE = 10;
int fullNoiseBuffer[FULL_NOISE_BUFFER_SIZE];
int fullNoiseBufferIndex = 0;
float fullNoiseBufferMean = 0;
float fullNoiseBufferStd = 0;
bool fullNoiseBufferIsFull = false;

// cry buffer holds the number of times a curr noise buffer has determined to have a "cry" signal
const int CRY_BUFFER_SIZE=5;
int CRY_BUFFER_THRESHOLD = 4; // number of "cry" samples to send alert
int cryBuffer[CRY_BUFFER_SIZE];
int cryBufferIndex = 0;

#if USE_TEMP_SENSOR
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();
#endif

#if USE_ACCEL_SENSOR
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
#endif

#if OUTPUT_OLED
Adafruit_SSD1306 display(-1);
bool oledOn = false;
#endif

FuelGauge fuel;
int noiseSensor = A0; // noise sensor is soldered to A0 analog input pin

#if SMART_CELLULAR
  SYSTEM_MODE(MANUAL);
#else
  SYSTEM_MODE(AUTOMATIC);
#endif

String getStateString(AppState state);

// setup() runs once, when the device is first turned on.
void setup() {

  deviceId = System.deviceID();

  Serial.begin();
  while (!Serial);
  Serial.println("Initializing");

#if USE_TEMP_SENSOR
  // init temp sensor
  if (!tempsensor.begin()) {
    Serial.println("couldn't find ADT7410");
    while(1);
  }
#endif

#if USE_ACCEL_SENSOR
   // init accel sensor
  if(!accel.begin()) {
    Serial.println("couldnt find ADXL343.");
    while(1);
  }

  // set accel range
  accel.setRange(ADXL343_RANGE_2_G);

  // sensors take 250 ms to get first readings
  delay(250);

  // get initial accel readings
  sensors_event_t event;
  accel.getEvent(&event);
  lastX = event.acceleration.x;
  lastY = event.acceleration.y;
  lastZ = event.acceleration.z;
#endif

#if OUTPUT_OLED
  // set display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oledOn = true;
#endif

  pinMode(noiseSensor, INPUT);

#if SMART_CELLULAR
  Cellular.off();
#endif

  checkNonCriticalSensors();

  // trigger last movement time so that "idle" can be properly calculated
  lastMovementTime =  millis();

  // initialize sound buffers
  resetSoundBuffers();
}

long loopCounter = 0;
void loop() {

  unsigned long now = millis();

#if USE_ACCEL_SENSOR
  // read accel
  sensors_event_t event;
  accel.getEvent(&event);
  currX = event.acceleration.x;
  currY = event.acceleration.y;
  currZ = event.acceleration.z;

  // check movement thresholds
  float deltaX = abs(abs(currX) - abs(lastX));
  float deltaY = abs(abs(currY) - abs(lastY));
  float deltaZ = abs(abs(currZ) - abs(lastZ));
  if (deltaX > ACCEL_THRESHOLD ||
      deltaY > ACCEL_THRESHOLD ||
      deltaZ > ACCEL_THRESHOLD) {

    // threshold met, assume car is moving
    Serial.print("moving: x:");Serial.print(deltaX);
    Serial.print(", y:");Serial.print(deltaY);
    Serial.print(", z:");Serial.println(deltaZ);

    lastX = currX;
    lastY = currY;
    lastZ = currZ;
    lastMovementTime = now;
    moving = true;
    monitoring = false;

    // any movement resets sound buffers and crying
    resetSoundBuffers();
    crying = false;
  }
  else if (now > (lastMovementTime + MOVEMENT_DELAY_START)) {
    // not moving - check if this is in the monitoring window
    //              based on lastMovementTime
    moving = false;
    monitoring = true;
    if (now > (lastMovementTime + MOVEMENT_DELAY_STOP)) {
      monitoring = false;
    }
  }

#else
  moving = false;
#endif

  if (moving || monitoring) {
    // process noise sensor to detect crying
    int soundVal = analogRead(noiseSensor);
    if (soundVal > 0) {
      // add new sound to temp noise buffer
      if (currNoiseBufferIndex < CURR_NOISE_BUFFER_SIZE) {
        currNoiseBuffer[currNoiseBufferIndex] = soundVal;
        currNoiseBufferIndex++;
      }
      if (currNoiseBufferIndex == CURR_NOISE_BUFFER_SIZE) {
        if (processTempNoiseBuffer()) {
          Serial.println("CRYING!!!");
          crying = true;
          lastCryingTime = now;
        }
        else if (now > (lastCryingTime + CRYING_DELAY)) {
          crying = false;
        }
        currNoiseBufferIndex = 0;
      }
    }
  }

  // evaluate app state
  bool publishAlert = false;
  if (moving) {
    currState = DISABLED_MOVING;
  }
  else if (!monitoring) {
    currState = DISABLED_IDLE;
  }
  else if (crying) {
    currState = ALERT;
    if (prevState != currState) {
      publishAlert = true;
    }
  }
  else {
    currState = MONITORING;
  }
  bool stateChanged = prevState != currState;
  prevState = currState;

  // update output every LOG_PERIOD_SECONDS
  if (stateChanged || loopCounter % ( LOG_PERIOD_SECONDS * 1000 / LOOP_PERIOD_MS) == 0) {
    Serial.println("======================================");
    String stateString = getStateString(currState);
    Serial.println(stateString);
    Serial.print("temp: ");Serial.println(tempF);
    Serial.print("battery: ");Serial.println(currentBatteryCharge);
    Serial.print("lowBatteryAlerted: ");Serial.println(lowBatteryAlerted);
    Serial.print("crying: ");Serial.println(crying);
    Serial.print("moving: ");Serial.println(moving);
    Serial.print("monitoring: ");Serial.println(monitoring);
    Serial.print("noise buffer: ");
    if (fullNoiseBufferIsFull) {
      Serial.println("full");
    }
    else {
      Serial.print(fullNoiseBufferIndex);Serial.print("/");Serial.println(FULL_NOISE_BUFFER_SIZE);
    }
    Serial.println("======================================");

#if OUTPUT_OLED
    if (oledOn && !monitoring && !moving) {
      // put oled into powersave
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      oledOn = false;
    }
    else if (!oledOn && (moving || monitoring)) {
      // take oled out of powersave
      display.ssd1306_command(SSD1306_DISPLAYON);
      oledOn = true;
    }

    if (oledOn) {
      // update LED
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);

      snprintf(buf, sizeof(buf), "%.1f F  Battery:%d%%", tempF, (int)currentBatteryCharge);
      display.println(buf);

      if (currentBatteryCharge < LOW_BATTERY_ALERT_THRESHOLD) {
        display.println("LOW BATTERY");
      }
      display.println("");
      display.println(stateString);

      display.display();
    }
#endif

  }

  if (publishAlert) {
    // publish alert message
    sendEventToCloud("alert");
  }

  // periodically check battery
  if (loopCounter % ( SENSOR_CHECK_PERIOD_SECONDS * 1000 / LOOP_PERIOD_MS) == 0) {
    checkNonCriticalSensors();
    if (currentBatteryCharge < LOW_BATTERY_ALERT_THRESHOLD && !lowBatteryAlerted) {
      sendEventToCloud("battery");
      lowBatteryAlerted = true;
    }
  }

  // always try a Process
  if (Particle.connected()) {
    Particle.process();
  }

  delay(LOOP_PERIOD_MS);
  loopCounter++;
}

String getStateString(AppState state) {
  switch(state) {
    case MONITORING:
      return STATE_MONITORING_STR;
    case DISABLED_MOVING:
      return STATE_DISABLED_MOVING_STR;
    case DISABLED_IDLE:
      return STATE_DISABLED_IDLE_STR;
    case ALERT:
      return STATE_ALERT_STR;
  }
  return STATE_UNKNOWN_TEMP_STR;
}

long getBatteryPercent() {
  return min(( fuel.getSoC() * 100 ) / 80, 100);
}

void checkNonCriticalSensors() {
  currentBatteryCharge = getBatteryPercent();
  if (currentBatteryCharge > (LOW_BATTERY_ALERT_THRESHOLD + 1)) {
    lowBatteryAlerted = false;
  }

#if USE_TEMP_SENSOR
  // read temp
  tempC = tempsensor.readTempC();
  tempF = (tempC * 9/5) + 32;
#endif
}

void startCellular() {
#if SMART_CELLULAR

  if (Particle.connected()) {
    particleProcessMessage();
    return;
  }

  // starting cellular and getting connected to Particle is not always super
  // easy and reliable.  the retry logic here is to give us the best chance
  // at making it work.
  Serial.println("");
  bool succeeded = false;
  for (int i = 0; i < CELLULAR_START_RETRY_LIMIT && !succeeded; i++) {
    Serial.print("startCellular(): attempt ");Serial.print(i+1);Serial.print(" of "); Serial.println(CELLULAR_START_RETRY_LIMIT);

    // turn radio on
    if (!Cellular.ready()) {
      Serial.println("turning on radio...");
      Cellular.on();
      delay(1000);
      Serial.println("cellular on, connecting...");
      Cellular.connect();
      delay(1000);
    }

    // wait for cellular ready
    Serial.print("waiting for cellular ready (max: ");Serial.print(CELLULAR_CONNECT_RETRY_LIMIT);Serial.print(")...");
    for (int j = 0; j < CELLULAR_CONNECT_RETRY_LIMIT && !Cellular.ready(); j++) {
      Serial.print(j);Serial.print("...");
      delay(1000);
    }
    Serial.println("done");
    if (!Cellular.ready()) {
      Serial.println("cellular not responding, turning radio off and trying again...");
      Cellular.off();
      delay(1000);
      continue;
    }
    delay(1000);

    // connect to Particle
    if (!Particle.connected()) {
      Serial.println("connecting to particle...");
      Particle.connect();
    }
    Serial.print("waiting for Particle.connected (max: ");Serial.print(CELLULAR_CONNECT_RETRY_LIMIT);Serial.print(")...");
    for (int j = 0; j < CELLULAR_CONNECT_RETRY_LIMIT && !Particle.connected(); j++) {
      Serial.print(j);Serial.print("...");
      delay(1000);
    }
    Serial.println("");
    if (!Particle.connected()) {
      Serial.println("particle didn't connect, turning radio off and trying again...");
      Cellular.off();
      delay(1000);
      continue;
    }
    delay(1000);

    // tell Particle to process messages
    particleProcessMessage();
    succeeded = true;
  }

  if (!succeeded) {
    // connect is stuck, only workaround is a reboot
    Serial.println("error: cannot connect, rebooting");
    System.reset();
  }

#endif
}


void stopCellular() {
#if SMART_CELLULAR
  Serial.println("stopCellular()");
  particleProcessMessage();
  Serial.println("disconnecting from particle");
  Particle.disconnect();
  Serial.println("turning cellular off");
  Cellular.off();
  Serial.println("cellular is off");
#endif
}

void particleProcessMessage() {
  Serial.print("calling Particle.process (num times: ");Serial.print(PARTICLE_PROCESS_RETRIES);Serial.print(")...");
  for (int i = 0; i < PARTICLE_PROCESS_RETRIES; i++) {
    Serial.print(i);Serial.print("...");
    Particle.process();
    delay(PARTICLE_PROCESS_DELAY_MS);
  }
  Serial.println("done");
};

void sendEventToCloud(String type) {
  checkNonCriticalSensors();
  Serial.println("particle connected. publishing message...");
  snprintf(buf, sizeof(buf), "{\"device\":\"%s\", \"tempF\":%.1f, \"lastMovement\":%lu, \"battery\":%d}", deviceId.c_str(), tempF, lastMovementTime, (int)currentBatteryCharge);
  Serial.println("publishing event: ");
  Serial.println(buf);
  sendBufToCloud(type);
}

void sendBufToCloud(String type) {
  // all of this retry logic is to try our best to get this message out
  bool succeded = false;
  for (int i = 0; i < PARTICLE_PUBLISH_RECONNECT_RETRIES && !succeded; i++) {
    startCellular();
    for (int j = 0; j < PARTICLE_PUBLISH_RETRIES; j++) {
      if (!Particle.publish(type, buf, PRIVATE)) {
        Serial.print("publish failed. retrying ");Serial.print(j);Serial.print(" of ");Serial.println(PARTICLE_PUBLISH_RETRIES);
        delay(1000);
      }
      else {
        succeded = true;
        break;
      }
    }
    stopCellular();
  }
}

// this algorithm determines if the noise is a baby cry
bool processTempNoiseBuffer() {

  // calculate current mean of full noiseBuffer
  int sum = 0;
  int numSamples = fullNoiseBufferIsFull ? FULL_NOISE_BUFFER_SIZE : fullNoiseBufferIndex;
  for (int i = 0; i < numSamples; i++) {
    sum += fullNoiseBuffer[i];
  }
  fullNoiseBufferMean = numSamples > 0 ? sum / numSamples : 0;

  // calculate standard deviation of full noiseBuffer
  long squares = 0;
  for (int i = 0; i < numSamples; i++) {
    squares += pow(fullNoiseBuffer[i] - fullNoiseBufferMean, 2);
  }
  fullNoiseBufferStd = numSamples > 0 ? sqrt(squares / numSamples) : 0;
  Serial.print("noiseBuffer: mean:");Serial.print(fullNoiseBufferMean);Serial.print(", std:");Serial.println(fullNoiseBufferStd);

  if (fullNoiseBufferIndex == FULL_NOISE_BUFFER_SIZE) {
    fullNoiseBufferIndex = 0; // reset ring buffer
    fullNoiseBufferIsFull = true;
  }

  // calculate mean of sample buffer
  sum = 0;
  for (int i = 0; i < CURR_NOISE_BUFFER_SIZE; i++) {
    sum += currNoiseBuffer[i];
  }
  currNoiseBufferMean = sum / CURR_NOISE_BUFFER_SIZE;

  // calculate std of sample buffer
  long sampleSquares = 0;
  for (int i = 0; i < CURR_NOISE_BUFFER_SIZE; i++) {
    sampleSquares += pow(currNoiseBuffer[i] - currNoiseBufferMean, 2);
  }
  currNoiseBufferStd = sqrt(sampleSquares / CURR_NOISE_BUFFER_SIZE);
  Serial.print("currNoiseBuffer: mean:");Serial.print(currNoiseBufferMean);Serial.print(", std:");Serial.println(currNoiseBufferStd);

  // remove any anomolies within the sample set
  sum = 0; // reset sample sum to only include valid numbers
  numSamples = 0;
  for (int i = 0; i < CURR_NOISE_BUFFER_SIZE; i++) {
    int val = currNoiseBuffer[i];
    float rejectionVal = NOISE_REJECTION_SIGMA * currNoiseBufferStd;
    if (val > (currNoiseBufferMean - rejectionVal) && val < (currNoiseBufferMean + rejectionVal)) {
      sum += val;
      numSamples++;
      Serial.print("addNoiseSamples: adding val:");Serial.println(val);
    }
    else {
      Serial.print("addNoiseSamples: ignoring val:");Serial.println(val);
    }
  }

  int lastCalcNoiseSample = sum / numSamples;

  // is new value high enough for a baby crying?
  bool isSampleCrying  = false;
  if (fullNoiseBufferIsFull &&
     (lastCalcNoiseSample > MAX_CRYING_THRESHOLD ||
     (lastCalcNoiseSample > MIN_CRYING_THRESHOLD && lastCalcNoiseSample > (fullNoiseBufferMean + fullNoiseBufferStd * BABY_CRY_SIGMA)))
     ) {
    isSampleCrying = true;
  }

  if (!isSampleCrying) {
    // put a non-crying sample into the noise buffer
    Serial.print("addNoiseSamples: new calc sample:");Serial.println(lastCalcNoiseSample);
    fullNoiseBuffer[fullNoiseBufferIndex] = lastCalcNoiseSample;
    fullNoiseBufferIndex++;
  }

  // process cryBuffer
  if (cryBufferIndex == CRY_BUFFER_SIZE) cryBufferIndex = 0; // reset ring buffer
  cryBuffer[cryBufferIndex] = isSampleCrying ? lastCalcNoiseSample : 0;
  cryBufferIndex++;
  int cryCount = 0;
  for (int i = 0; i < CRY_BUFFER_SIZE; i++) {
    // count number of "cry" values in cryBuffer
    if (cryBuffer[i] > 0) cryCount++;
  }

  Serial.print("cryBuffer: [");
  for (int i = 0; i < CRY_BUFFER_SIZE; i++) { Serial.print(" ");Serial.print(cryBuffer[i]);}
  Serial.println("]");

  bool isBufferCrying = (cryCount >= CRY_BUFFER_THRESHOLD);
  if (isBufferCrying) {
    resetCryBuffer(); // do not double-count cries
  }
  return isBufferCrying;
}

void resetCryBuffer() {
  for (int i = 0; i < CRY_BUFFER_SIZE; i++) {
    cryBuffer[i] = 0;
  }
}

void resetSoundBuffers() {
  for(int i = 0; i < FULL_NOISE_BUFFER_SIZE; i++) {
    fullNoiseBuffer[i] = 0;
  }
  fullNoiseBufferIndex = 0;
  fullNoiseBufferIsFull = false;
  fullNoiseBufferMean = fullNoiseBufferStd = 0;

  for(int i = 0; i < CURR_NOISE_BUFFER_SIZE; i++) {
    currNoiseBuffer[i] = 0;
  }
  currNoiseBufferIndex = 0;
  currNoiseBufferMean = currNoiseBufferStd = 0;

  resetCryBuffer();
}