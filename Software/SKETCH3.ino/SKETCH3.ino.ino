#include <LCDWIKI_GUI.h>
#include <LCDWIKI_SPI.h>
#include <SPI.h>
#include <Servo.h>
#include <util/atomic.h>

// --- TFT Pins ---
#define CS_PIN   A5
#define CD_PIN   A3
#define RST_PIN  A4
#define LED_PIN  A0

// Initialize with reduced SPI speed (8MHz instead of default 24MHz)
LCDWIKI_SPI mylcd(ILI9341, CS_PIN, CD_PIN, RST_PIN, LED_PIN);

// --- Corrected Colors ---
#define DARK_BLUE    0x000D
#define MEDIUM_BLUE  0x01DF
#define LIGHT_BLUE   0x02FF
#define DARK_GREEN   0x03E0
#define MEDIUM_GREEN 0x07E0
#define LIGHT_GREEN  0xAFE5
#define DARK_RED     0x7800
#define MEDIUM_RED   0xF800
#define LIGHT_RED    0xFC08
#define DARK_GRAY    0x2945
#define MEDIUM_GRAY  0x528A
#define LIGHT_GRAY   0x7BCF
#define DARK_YELLOW  0xBC40
#define MEDIUM_YELLOW 0xFFE0
#define LIGHT_YELLOW 0xFFF3
#define WHITE        0xFFFF
#define BLACK        0x0000

// --- Rotary Encoder Pins ---
#define ENC_A 2
#define ENC_B 3
#define ENC_SW 4

// --- Motor PWM Pin ---
#define PWM_PIN 9

// --- Servo Pin ---
#define SERVO_PIN 10
Servo myservo;

// --- Buttons ---
#define START_STOP_BTN 8
#define RESET_BTN 7

// --- Optical Sensor Pin ---
#define SENSOR_PIN 5

// --- System Variables ---
volatile unsigned long counter = 0;
volatile unsigned long targetCount = 1000;
volatile bool isRunning = false;
int motorPWM = 128;
int offset = 30;
int distance = 60;
int currentServoPos = 0;

// Double buffering for display stability
unsigned long atomicCounter = 0;
unsigned long atomicTarget = 0;
bool atomicRunning = false;
unsigned long displayCounter = 0;
unsigned long displayTarget = 0;
bool displayRunning = false;

// Encoder handling
volatile int8_t encoderSteps = 0;
unsigned long lastEncoderProcessTime = 0;
const int encoderFastThreshold = 50; // ms
const int fastIncrement = 10;

enum MenuState { MAIN, SET_TARGET, SET_SPEED, SET_OFFSET, SET_DISTANCE };
MenuState currentMenu = MAIN;

// Separate debounce timers for each button
unsigned long lastEncButtonTime = 0;
unsigned long lastStartStopTime = 0;
unsigned long lastResetTime = 0;
const unsigned long debounceDelay = 250;  // Increased debounce time

// Display state tracking
unsigned long lastCounter = 0;
unsigned long lastTarget = 0;
bool lastRunning = false;
int lastMotorPWM = -1;
int lastOffset = -1;
int lastDistance = -1;
MenuState lastMenuState = MAIN;
int lastProgressWidth = -1;

// Slider tracking
long lastSliderValue = -1;
int lastSliderPos = -1;
const int SLIDER_MIN = 40;
const int SLIDER_MAX = 280;
const int SLIDER_Y = 150;
const int SLIDER_HANDLE_WIDTH = 30;
const int SLIDER_HANDLE_HEIGHT = 30;
const int SLIDER_TRACK_HEIGHT = 20;

// Sensor variables - COMPLETELY REWORKED
volatile unsigned long lastCountTime = 0;
const unsigned long sensorTimeout = 100; // 100ms timeout for valid counts
bool sensorActive = true;
bool sensorErrorDisplayed = false;
bool lastSensorState = HIGH;

// Buffer for number formatting
char numBuffer[20];

void setup() {
  // Initialize hardware
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);

  pinMode(START_STOP_BTN, INPUT_PULLUP);
  pinMode(RESET_BTN, INPUT_PULLUP);
  
  // Sensor setup
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  lastSensorState = digitalRead(SENSOR_PIN);  // Read initial state

  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);

  myservo.attach(SERVO_PIN);
  myservo.write(0);

  // Initialize display
  mylcd.Init_LCD();
  mylcd.Fill_Screen(BLACK);
  mylcd.Set_Text_Mode(0);
  mylcd.Set_Rotation(1);

  drawBaseUI();
  updateDisplay(true);
}

void loop() {
  // Update atomic variables
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    atomicCounter = counter;
    atomicTarget = targetCount;
    atomicRunning = isRunning;
  }

  // Handle encoder steps
  handleEncoderMovement();

  // Handle encoder button press
  if (digitalRead(ENC_SW) == LOW && millis() - lastEncButtonTime > debounceDelay) {
    lastEncButtonTime = millis();
    handleEncoderClick();
  }

  // Handle start/stop button
  static bool lastStartStopState = HIGH;
  bool currentStartStopState = digitalRead(START_STOP_BTN);
  
  if (lastStartStopState == HIGH && currentStartStopState == LOW) {
    if (millis() - lastStartStopTime > debounceDelay) {
      lastStartStopTime = millis();
      isRunning = !isRunning;
      analogWrite(PWM_PIN, isRunning ? motorPWM : 0);
      Serial.print("Start/Stop pressed. Running: ");
      Serial.println(isRunning);
      
      if (!isRunning) {
        myservo.write(0);
        currentServoPos = 0;
      } else {
        // Reset sensor state when starting
        lastSensorState = digitalRead(SENSOR_PIN);
      }
      updateDisplay(true);
    }
  }
  lastStartStopState = currentStartStopState;

  // Handle reset button
  static bool lastResetState = HIGH;
  bool currentResetState = digitalRead(RESET_BTN);
  
  if (lastResetState == HIGH && currentResetState == LOW) {
    if (millis() - lastResetTime > debounceDelay) {
      lastResetTime = millis();
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        counter = 0;
      }
      isRunning = false;
      analogWrite(PWM_PIN, 0);
      myservo.write(0);
      currentServoPos = 0;
      Serial.println("Reset pressed");
      updateDisplay(true);
    }
  }
  lastResetState = currentResetState;

  // NEW SENSOR POLLING MECHANISM (More reliable than interrupts)
  static unsigned long lastSensorCheck = 0;
  if (millis() - lastSensorCheck > 1) {  // Check sensor every 1ms
    lastSensorCheck = millis();
    
    bool currentSensorState = digitalRead(SENSOR_PIN);
    
    // Detect falling edge (object passing through)
    if (lastSensorState == HIGH && currentSensorState == LOW) {
      if (isRunning) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          counter++;
        }
        Serial.print("Counter: ");
        Serial.println(counter);
        lastCountTime = millis();
      }
    }
    lastSensorState = currentSensorState;
  }

  // Sensor validation check
  static unsigned long lastValidationCheck = 0;
  if (millis() - lastValidationCheck > 1000) {  // Check every second
    lastValidationCheck = millis();
    
    if (isRunning) {
      // Check if we've had a count recently
      if (millis() - lastCountTime > 2000) {  // 2 seconds without a count
        if (sensorActive) {
          sensorActive = false;
          sensorErrorDisplayed = true;
          mylcd.Set_Text_Size(1);
          mylcd.Set_Text_colour(LIGHT_RED);
          mylcd.Set_Text_Back_colour(BLACK);
          mylcd.Fill_Rect(10, 224, 120, 10, BLACK);
          mylcd.Print_String("SENSOR ERROR!", 10, 224);
        }
      } else if (!sensorActive || sensorErrorDisplayed) {
        sensorActive = true;
        sensorErrorDisplayed = false;
        mylcd.Fill_Rect(10, 224, 120, 10, DARK_GRAY);
        updateStatusBar();
      }
    } else if (!sensorActive || sensorErrorDisplayed) {
      sensorActive = true;
      sensorErrorDisplayed = false;
      mylcd.Fill_Rect(10, 224, 120, 10, DARK_GRAY);
      updateStatusBar();
    }
  }

  // Update display with double buffering
  if (atomicCounter != displayCounter || 
      atomicTarget != displayTarget || 
      atomicRunning != displayRunning) {
    displayCounter = atomicCounter;
    displayTarget = atomicTarget;
    displayRunning = atomicRunning;
    updateDisplay(false);
  }

  // Full display updates when needed
  if (currentMenu != lastMenuState || 
      motorPWM != lastMotorPWM || 
      offset != lastOffset || 
      distance != lastDistance) {
    updateDisplay(true);
    lastMenuState = currentMenu;
    lastMotorPWM = motorPWM;
    lastOffset = offset;
    lastDistance = distance;
  }

  // Servo control logic
  if (isRunning && atomicCounter < atomicTarget) {
    unsigned long currentCount;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currentCount = counter;
    }
    
    int progress = currentCount % (2 * distance);
    if (progress <= distance) {
      currentServoPos = offset + progress;
    } else {
      currentServoPos = offset + (2 * distance) - progress;
    }
    
    myservo.write(currentServoPos);
  } 
  else if (isRunning && atomicCounter >= atomicTarget) {
    isRunning = false;
    analogWrite(PWM_PIN, 0);
    myservo.write(0);
    currentServoPos = 0;
    updateDisplay(true);
  }

  // Small delay to prevent UI lag
  delay(10);
}

// REMOVED SENSOR ISR - Using polling instead for better reliability

// Encoder movement handler
void handleEncoderMovement() {
  int8_t steps = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    steps = encoderSteps;
    encoderSteps = 0;
  }

  if (steps != 0) {
    unsigned long now = millis();
    int multiplier = 1;
    
    if (now - lastEncoderProcessTime < encoderFastThreshold) {
      multiplier = fastIncrement;
    }
    
    int change = steps * multiplier;
    
    switch (currentMenu) {
      case SET_TARGET:
        targetCount = constrain(targetCount + change, 0, 999999);
        updateSettingValue(targetCount, "", 0, 999999);
        break;
      case SET_SPEED:
        motorPWM = constrain(motorPWM + change, 0, 255);
        if (isRunning) analogWrite(PWM_PIN, motorPWM);
        updateSettingValue(motorPWM, "PWM", 0, 255);
        break;
      case SET_OFFSET:
        offset = constrain(offset + change, 0, 180);
        if (!isRunning) myservo.write(offset);
        updateSettingValue(offset, "deg", 0, 180);
        break;
      case SET_DISTANCE:
        distance = constrain(distance + change, 0, 180);
        if (!isRunning) myservo.write(offset + distance);
        updateSettingValue(distance, "deg", 0, 180);
        break;
      default:
        break;
    }
    
    lastEncoderProcessTime = now;
  }
}

// Encoder ISR - minimal processing
void read_encoder() {
  static uint8_t old_AB = 3;
  static int8_t encval = 0;
  static const int8_t enc_states[] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
   -1, 0, 0, 1,
    0, 1, -1, 0
  };

  old_AB <<= 2;
  if (digitalRead(ENC_A)) old_AB |= 0x02;
  if (digitalRead(ENC_B)) old_AB |= 0x01;
  encval += enc_states[(old_AB & 0x0F)];

  if (encval > 3) {
    encoderSteps++;
    encval = 0;
  } 
  else if (encval < -3) {
    encoderSteps--;
    encval = 0;
  }
}

void drawBaseUI() {
  // Draw header
  mylcd.Fill_Rect(0, 0, 320, 40, DARK_BLUE);
  
  mylcd.Set_Text_Size(2);
  mylcd.Set_Text_colour(WHITE);
  mylcd.Set_Text_Back_colour(DARK_BLUE);
  mylcd.Print_String("COIL WINDER CONTROLLER", 40, 12);
  
  // Draw footer
  mylcd.Fill_Rect(0, 220, 320, 20, DARK_GRAY);
  updateStatusBar();
}

void updateStatusBar() {
  mylcd.Fill_Rect(0, 220, 320, 20, DARK_GRAY);
  mylcd.Set_Text_Size(1);
  mylcd.Set_Text_colour(LIGHT_GRAY);
  mylcd.Set_Text_Back_colour(DARK_GRAY);
  
  switch (currentMenu) {
    case MAIN:
      mylcd.Print_String("Main Menu  |  Press encoder to enter settings", 30, 224);
      break;
    case SET_TARGET:
      mylcd.Print_String("Setting Target Turns  |  Range: 0-999999", 40, 224);
      break;
    case SET_SPEED:
      mylcd.Print_String("Setting Motor Speed  |  Range: 0-255 PWM", 40, 224);
      break;
    case SET_OFFSET:
      mylcd.Print_String("Setting Servo Offset  |  Range: 0-180 deg", 40, 224);
      break;
    case SET_DISTANCE:
      mylcd.Print_String("Setting Travel Distance  |  Range: 0-180 deg", 40, 224);
      break;
  }
}

void drawMainMenu() {
  // Clear content area
  mylcd.Fill_Rect(0, 50, 320, 170, BLACK);

  // Draw status panel
  mylcd.Set_Draw_color(DARK_GRAY);
  mylcd.Fill_Round_Rectangle(10, 60, 310, 120, 5);

  // Draw counters
  mylcd.Set_Text_Size(2);
  mylcd.Set_Text_colour(LIGHT_YELLOW);
  mylcd.Print_String("Current:", 20, 70);
  sprintf(numBuffer, "%06lu", atomicCounter);
  mylcd.Print_String(numBuffer, 140, 70);

  mylcd.Print_String("Target:", 20, 95);
  sprintf(numBuffer, "%06lu", atomicTarget);
  mylcd.Print_String(numBuffer, 140, 95);

  // Draw progress bar background
  mylcd.Set_Draw_color(DARK_GRAY);
  mylcd.Fill_Round_Rectangle(20, 130, 300, 150, 5);
  mylcd.Set_Draw_color(BLACK);
  mylcd.Fill_Rect(20, 130, 280, 20, BLACK);
  updateProgressBar(atomicCounter, atomicTarget);

  // Draw motor status area
  mylcd.Set_Draw_color(atomicRunning ? MEDIUM_GREEN : MEDIUM_RED);
  mylcd.Fill_Round_Rectangle(70, 160, 250, 190, 5);
  updateMotorStatus(atomicRunning);

  // Draw navigation hint
  mylcd.Set_Text_Size(1);
  mylcd.Set_Text_colour(LIGHT_GRAY);
  mylcd.Print_String("Rotate encoder to adjust settings", 60, 200);
}

// Unified UI for all settings pages
void drawSettingMenu(const char* title, long value, const char* unit, long minVal, long maxVal) {
  // Clear content area
  mylcd.Fill_Rect(0, 50, 320, 170, BLACK);
  
  // Draw title
  mylcd.Set_Text_Size(2);
  mylcd.Set_Text_colour(LIGHT_BLUE);
  int titleWidth = strlen(title) * 12;
  mylcd.Print_String(title, 160 - titleWidth/2, 60);
  
  // Draw value in large font
  mylcd.Set_Text_Size(4);
  mylcd.Set_Text_colour(LIGHT_YELLOW);
  
  sprintf(numBuffer, "%ld %s", value, unit);
  int textWidth = strlen(numBuffer) * 18;
  mylcd.Print_String(numBuffer, 160 - textWidth/2, 90);
  
  // Draw slider track
  mylcd.Set_Draw_color(DARK_GRAY);
  mylcd.Fill_Round_Rectangle(SLIDER_MIN, SLIDER_Y, SLIDER_MAX, SLIDER_Y + SLIDER_TRACK_HEIGHT, 5);
  
  // Draw slider handle
  int sliderPos = map(value, minVal, maxVal, SLIDER_MIN, SLIDER_MAX);
  mylcd.Set_Draw_color(MEDIUM_BLUE);
  mylcd.Fill_Round_Rectangle(sliderPos - SLIDER_HANDLE_WIDTH/2, 
                            SLIDER_Y - (SLIDER_HANDLE_HEIGHT - SLIDER_TRACK_HEIGHT)/2,
                            sliderPos + SLIDER_HANDLE_WIDTH/2, 
                            SLIDER_Y + SLIDER_TRACK_HEIGHT + (SLIDER_HANDLE_HEIGHT - SLIDER_TRACK_HEIGHT)/2, 
                            5);
  lastSliderPos = sliderPos;
  
  // Draw min/max labels
  mylcd.Set_Text_Size(1);
  mylcd.Set_Text_colour(LIGHT_GRAY);
  sprintf(numBuffer, "%ld", minVal);
  mylcd.Print_String(numBuffer, SLIDER_MIN - 20, SLIDER_Y + 25);
  sprintf(numBuffer, "%ld", maxVal);
  mylcd.Print_String(numBuffer, SLIDER_MAX - 10, SLIDER_Y + 25);
  
  // Draw navigation hint
  mylcd.Set_Text_Size(1);
  mylcd.Set_Text_colour(LIGHT_GRAY);
  mylcd.Print_String("Rotate to adjust  |  Press to confirm", 50, 200);
  
  // Save current setting value
  lastSliderValue = value;
}

void updateCounterDisplay(unsigned long currentCount, unsigned long currentTarget) {
  // Only update the counter numbers
  mylcd.Set_Text_Size(2);
  mylcd.Set_Text_colour(LIGHT_YELLOW);
  mylcd.Set_Text_Back_colour(DARK_GRAY);
  
  // Update current counter
  sprintf(numBuffer, "%06lu", currentCount);
  mylcd.Print_String(numBuffer, 140, 70);
  
  // Update target counter if changed
  if (currentTarget != lastTarget) {
    sprintf(numBuffer, "%06lu", currentTarget);
    mylcd.Print_String(numBuffer, 140, 95);
  }
}

void updateProgressBar(unsigned long currentCount, unsigned long currentTarget) {
  unsigned long progressWidth = 0;
  
  if (currentTarget > 0) {
    progressWidth = map(constrain(currentCount, 0, currentTarget), 0, currentTarget, 0, 280);
  }
  
  if (progressWidth != lastProgressWidth) {
    // Clear old progress if needed
    if (progressWidth < lastProgressWidth) {
      mylcd.Set_Draw_color(BLACK);
      mylcd.Fill_Rect(20 + progressWidth, 130, lastProgressWidth - progressWidth, 20, BLACK);
    }
  
    // Draw new progress
    if (progressWidth > 0) {
      // Smooth gradient effect
      for (int i = 0; i < 20; i++) {
        uint16_t color = interpolateColor(MEDIUM_GREEN, LIGHT_GREEN, i, 20);
        mylcd.Set_Draw_color(color);
        mylcd.Draw_Fast_HLine(20, 130 + i, progressWidth);
      }
    }
  
    lastProgressWidth = progressWidth;
  }
}

uint16_t interpolateColor(uint16_t color1, uint16_t color2, int step, int steps) {
  uint8_t r1 = (color1 >> 11) & 0x1F;
  uint8_t g1 = (color1 >> 5) & 0x3F;
  uint8_t b1 = color1 & 0x1F;
  
  uint8_t r2 = (color2 >> 11) & 0x1F;
  uint8_t g2 = (color2 >> 5) & 0x3F;
  uint8_t b2 = color2 & 0x1F;
  
  uint8_t r = r1 + (r2 - r1) * step / steps;
  uint8_t g = g1 + (g2 - g1) * step / steps;
  uint8_t b = b1 + (b2 - b1) * step / steps;
  
  return (r << 11) | (g << 5) | b;
}

void updateMotorStatus(bool running) {
  uint16_t bgColor = running ? MEDIUM_GREEN : MEDIUM_RED;
    
  mylcd.Set_Draw_color(bgColor);
  mylcd.Fill_Round_Rectangle(70, 160, 250, 190, 5);
  
  mylcd.Set_Text_Size(2);
  mylcd.Set_Text_colour(WHITE);
  mylcd.Set_Text_Back_colour(bgColor);
  
  if (running) {
    mylcd.Print_String("MOTOR: RUNNING", 75, 165);
  } else {
    mylcd.Print_String("MOTOR: STOPPED", 75, 165);
  }
}

void updateSlider(long value, long minVal, long maxVal) {
  // Calculate new slider position
  int newSliderPos = map(value, minVal, maxVal, SLIDER_MIN, SLIDER_MAX);
  
  // Only update if position changed
  if (newSliderPos == lastSliderPos) return;
  
  // Determine direction and steps
  int steps = abs(newSliderPos - lastSliderPos);
  int direction = (newSliderPos > lastSliderPos) ? 1 : -1;
  
  // Animate smoothly
  for (int i = 0; i < steps; i++) {
    int currentPos = lastSliderPos + direction;
    
    // Erase old slider handle
    mylcd.Set_Draw_color(DARK_GRAY);
    mylcd.Fill_Round_Rectangle(lastSliderPos - SLIDER_HANDLE_WIDTH/2, 
                             SLIDER_Y - (SLIDER_HANDLE_HEIGHT - SLIDER_TRACK_HEIGHT)/2,
                             lastSliderPos + SLIDER_HANDLE_WIDTH/2, 
                             SLIDER_Y + SLIDER_TRACK_HEIGHT + (SLIDER_HANDLE_HEIGHT - SLIDER_TRACK_HEIGHT)/2, 
                             5);
    
    // Repair the track
    mylcd.Set_Draw_color(DARK_GRAY);
    mylcd.Fill_Rect(lastSliderPos - SLIDER_HANDLE_WIDTH/2, SLIDER_Y, 
                   SLIDER_HANDLE_WIDTH, SLIDER_TRACK_HEIGHT, DARK_GRAY);
    
    // Draw new slider handle
    mylcd.Set_Draw_color(MEDIUM_BLUE);
    mylcd.Fill_Round_Rectangle(currentPos - SLIDER_HANDLE_WIDTH/2, 
                             SLIDER_Y - (SLIDER_HANDLE_HEIGHT - SLIDER_TRACK_HEIGHT)/2,
                             currentPos + SLIDER_HANDLE_WIDTH/2, 
                             SLIDER_Y + SLIDER_TRACK_HEIGHT + (SLIDER_HANDLE_HEIGHT - SLIDER_TRACK_HEIGHT)/2, 
                             5);
    
    lastSliderPos = currentPos;
    
    // Small delay for animation
    delay(5);
  }
}

void updateSettingValue(long value, const char* unit, long minVal, long maxVal) {
  // Update number display
  mylcd.Set_Text_Size(4);
  
  // Only update if value changed
  if (value != lastSliderValue) {
    // Clear previous number
    mylcd.Set_Text_colour(BLACK);
    mylcd.Set_Text_Back_colour(BLACK);
    sprintf(numBuffer, "%ld %s", lastSliderValue, unit);
    int textWidth = strlen(numBuffer) * 18;
    mylcd.Fill_Rect(160 - textWidth/2 - 5, 90, textWidth + 10, 40, BLACK);
    
    // Draw new number
    mylcd.Set_Text_colour(LIGHT_YELLOW);
    sprintf(numBuffer, "%ld %s", value, unit);
    textWidth = strlen(numBuffer) * 18;
    mylcd.Print_String(numBuffer, 160 - textWidth/2, 90);
    
    // Update slider with smooth animation
    updateSlider(value, minVal, maxVal);
    
    lastSliderValue = value;
  }
}

void updateDisplay(bool full) {
  if (full) {
    drawBaseUI();
    
    switch (currentMenu) {
      case MAIN:
        drawMainMenu();
        break;
      case SET_TARGET:
        drawSettingMenu("TARGET TURNS", targetCount, "", 0, 999999);
        break;
      case SET_SPEED:
        drawSettingMenu("MOTOR SPEED", motorPWM, "PWM", 0, 255);
        break;
      case SET_OFFSET:
        drawSettingMenu("SERVO OFFSET", offset, "deg", 0, 180);
        break;
      case SET_DISTANCE:
        drawSettingMenu("TRAVEL DISTANCE", distance, "deg", 0, 180);
        break;
    }
    
    updateStatusBar();
  } else {
    // Partial updates for main menu
    if (currentMenu == MAIN) {
      // Update counter if changed
      if (displayCounter != lastCounter || displayTarget != lastTarget) {
        updateCounterDisplay(displayCounter, displayTarget);
        lastCounter = displayCounter;
        lastTarget = displayTarget;
      }
      
      // Update progress bar if needed
      updateProgressBar(displayCounter, displayTarget);
      
      // Update motor status if changed
      if (displayRunning != lastRunning) {
        updateMotorStatus(displayRunning);
        lastRunning = displayRunning;
      }
    }
    // Partial updates for settings menus
    else {
      switch (currentMenu) {
        case SET_TARGET:
          updateSettingValue(targetCount, "", 0, 999999);
          break;
        case SET_SPEED:
          updateSettingValue(motorPWM, "PWM", 0, 255);
          break;
        case SET_OFFSET:
          updateSettingValue(offset, "deg", 0, 180);
          break;
        case SET_DISTANCE:
          updateSettingValue(distance, "deg", 0, 180);
          break;
        default:
          break;
      }
    }
  }
}

void handleEncoderClick() {
  MenuState oldState = currentMenu;
  currentMenu = static_cast<MenuState>((currentMenu + 1) % 5);
  
  // Reset servo position when leaving offset/distance settings
  if (oldState == SET_OFFSET || oldState == SET_DISTANCE) {
    myservo.write(0);
    currentServoPos = 0;
  }
  
  updateDisplay(true);
}