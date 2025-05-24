#include <LCDWIKI_GUI.h>
#include <LCDWIKI_SPI.h>
#include <Servo.h>
#include "tesla_bg.h" 

// ----- Hardware Configuration -----
#define MODEL ILI9341
#define CS    A5    
#define CD    A3
#define RST   A4
#define MISO  12
#define MOSI  11
#define SCK   13
#define LED   A0

LCDWIKI_SPI lcd(MODEL, CS, CD, MISO, MOSI, RST, SCK, LED);

// ----- Color Palette -----
#define DARK_BLUE    0x000D
#define CYAN         0x07FF
#define GOLD         0xFEA0
#define SILVER       0xC618
#define DARK_RED     0x8000
#define DARK_GRAY    0x2104

// ----- Control Pins -----
#define SERVO_PIN     9
#define DC_MOTOR_PIN  5
#define BUTTON_START  8
#define BUTTON_RESET  7
#define ENCODER_CLK   2
#define ENCODER_DT    3
#define ENCODER_BTN   4

// ----- System State -----
volatile long encoderValue = 0;
int setTurns = 0;
bool winding = false;
Servo traverseServo;

// ----- UI Constants -----
const int PROGRESS_Y = 180;
const int INFO_BOX_Y = 80;

// ----- Debounced Encoder ISR -----
volatile unsigned long lastEncInt = 0;
void updateEncoder() {
  if (millis() - lastEncInt < 5) return;
  static uint8_t oldAB = 3;
  oldAB <<= 2;
  oldAB |= (digitalRead(ENCODER_CLK) << 1 | digitalRead(ENCODER_DT);
  
  const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  encoderValue += enc_states[(oldAB & 0x0F)];
  lastEncInt = millis();
}

// ----- UI Components -----
void drawBackground() {
  lcd.Draw_Bitmap(0, 0, 240, 320, (uint16_t *)tesla_bg);
}

void drawProgressBar(int percent) {
  int w = map(percent, 0, 100, 0, 200);
  
  // Outer frame
  lcd.Set_Draw_color(SILVER);
  lcd.Fill_Round_Rectangle(20, PROGRESS_Y, 220, PROGRESS_Y+20, 5);
  
  // Inner fill
  lcd.Set_Draw_color(CYAN);
  lcd.Fill_Round_Rectangle(22, PROGRESS_Y+2, 22 + w, PROGRESS_Y+18, 3);
  
  // Percentage text
  lcd.Set_Text_Size(2);
  lcd.Set_Text_colour(WHITE);
  lcd.Print_Number_Int(percent, 200, PROGRESS_Y-30, 0, ' ');
  lcd.Print_String("%", 200, PROGRESS_Y-30);
}

void drawInfoBox(String text, int y) {
  lcd.Set_Draw_color(DARK_BLUE);
  lcd.Fill_Round_Rectangle(20, y, 220, y+40, 5);
  lcd.Set_Text_Size(2);
  lcd.Set_Text_colour(WHITE);
  lcd.Print_String(text, CENTER, y+12);
}

void drawButtonLegend() {
  lcd.Set_Text_Size(1);
  lcd.Set_Text_colour(SILVER);
  lcd.Print_String("[ ] Confirm   [ ] Cancel", CENTER, 280);
  lcd.Set_Text_colour(GOLD);
  lcd.Print_String(" A ", 70, 280);
  lcd.Print_String(" B ", 170, 280);
}

// ----- Display Modes -----
void showSetupScreen() {
  drawBackground();
  drawInfoBox("SET TURNS:", INFO_BOX_Y);
  
  lcd.Set_Text_Size(4);
  lcd.Set_Text_colour(GOLD);
  lcd.Print_Number_Int(abs(encoderValue), CENTER, INFO_BOX_Y + 60);
  
  drawButtonLegend();
}

void showWindingScreen() {
  drawBackground();
  drawInfoBox("WINDING COIL...", INFO_BOX_Y);
  
  int progress = (abs(encoderValue) * 100) / setTurns;
  drawProgressBar(progress);
  
  lcd.Set_Text_Size(2);
  lcd.Set_Text_colour(WHITE);
  lcd.Print_String("Completed: ", 30, 210);
  lcd.Print_Number_Int(abs(encoderValue), 160, 210);
}

void showCompleteScreen() {
  drawBackground();
  drawInfoBox("WINDING COMPLETE!", INFO_BOX_Y);
  
  lcd.Set_Text_Size(3);
  lcd.Set_Text_colour(GOLD);
  lcd.Print_String("\x07", CENTER, 150); // Checkmark symbol
  
  lcd.Set_Text_Size(2);
  lcd.Print_String("Total Turns: " + String(setTurns), CENTER, 200);
}

// ----- Core Functions -----
void setup() {
  Serial.begin(115200);
  
  // LCD Initialization
  lcd.Init_LCD();
  lcd.Fill_Screen(BLACK);
  lcd.Set_Rotation(3);
  
  // Encoder Setup
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), updateEncoder, CHANGE);
  
  // Control Setup
  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_RESET, INPUT_PULLUP);
  traverseServo.attach(SERVO_PIN);
  
  // Initial UI
  showSetupScreen();
}

void loop() {
  static unsigned long lastUpdate = 0;
  
  if (!winding) {
    if (millis() - lastUpdate > 250) {
      showSetupScreen();
      lastUpdate = millis();
    }
    
    if (digitalRead(BUTTON_START) == LOW) {
      setTurns = abs(encoderValue);
      if (setTurns > 0) {
        encoderValue = 0;
        winding = true;
        showWindingScreen();
        delay(500);
      }
    }
  } 
  else {
    if (abs(encoderValue) < setTurns) {
      analogWrite(DC_MOTOR_PIN, 180);
      traverseServo.write(map(abs(encoderValue) % 100, 0, 99, 30, 150));
      
      if (millis() - lastUpdate > 500) {
        showWindingScreen();
        lastUpdate = millis();
      }
    } 
    else {
      analogWrite(DC_MOTOR_PIN, 0);
      traverseServo.write(90);
      showCompleteScreen();
      winding = false;
      delay(3000);
      encoderValue = 0;
      showSetupScreen();
    }
  }

  // Reset handling
  if (digitalRead(BUTTON_RESET) == LOW) {
    winding = false;
    encoderValue = 0;
    setTurns = 0;
    analogWrite(DC_MOTOR_PIN, 0);
    traverseServo.write(90);
    showSetupScreen();
    delay(500);
  }

  // Emergency stop
  if (digitalRead(BUTTON_RESET) && winding) {
    analogWrite(DC_MOTOR_PIN, 0);
    traverseServo.detach();
    drawInfoBox("EMERGENCY STOPPED!", INFO_BOX_Y);
    while(true);
  }
}