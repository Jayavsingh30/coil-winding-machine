#include <LCDWIKI_GUI.h>c:\Users\JAYAVARDHAN SINGH\OneDrive\Desktop\testing\coil-winding-machine-main\Software\sketch.ino
#include <LCDWIKI_SPI.h>
#include <Servo.h>

// ----- Pin Definitions -----
#define MODEL ILI9341
#define CS    A5    
#define CD    A3
#define RST   A4
#define MISO  12
#define MOSI  11
#define SCK   13
#define LED   A0

LCDWIKI_SPI lcd(MODEL, CS, CD, MISO, MOSI, RST, SCK, LED);

// ----- Colors -----
#define BLACK   0x0000
#define GRAY    0x7BEF
#define WHITE   0xFFFF
#define CYAN    0x07FF
#define GREEN   0x07E0
#define YELLOW  0xFFE0
#define RED     0xF800

// ----- Control Pins -----
#define SERVO_PIN     9
#define DC_MOTOR_PIN  5
#define BUTTON_START  8
#define BUTTON_RESET  7
#define ENCODER_CLK   2
#define ENCODER_DT    3
#define ENCODER_BTN   4

// ----- State -----
volatile long encoderValue = 0;
int lastEncoderState;
int setTurns = 0;
bool winding = false;
Servo traverseServo;

// ----- Gear Drawing -----
void drawGear(int cx, int cy, int r, float angle) {
  lcd.Set_Draw_color(GRAY);
  lcd.Fill_Circle(cx, cy, r / 3);
  lcd.Draw_Circle(cx, cy, r);
  for (int i = 0; i < 6; i++) {
    float a = angle + i * PI / 3;
    int x = cx + cos(a) * r;
    int y = cy + sin(a) * r;
    lcd.Draw_Line(cx, cy, x, y);
  }
}

// ----- Encoder ISR -----
void updateEncoder() {
  int cur = digitalRead(ENCODER_CLK);
  if (cur != lastEncoderState) {
    if (digitalRead(ENCODER_DT) != cur)
      encoderValue++;
    else
      encoderValue--;
  }
  lastEncoderState = cur;
}

// ----- Setup -----
void setup() {
  Serial.begin(9600);

  lcd.Init_LCD();
  lcd.Fill_Screen(BLACK);
  lcd.Set_Text_Back_colour(BLACK);
  lcd.Set_Text_Size(2);
  lcd.Set_Text_colour(WHITE);

  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), updateEncoder, CHANGE);
  lastEncoderState = digitalRead(ENCODER_CLK);

  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_RESET, INPUT_PULLUP);
  pinMode(DC_MOTOR_PIN, OUTPUT);

  traverseServo.attach(SERVO_PIN);
  traverseServo.write(90);

  lcd.Print_String("Set Turns:", 10, 40);
}

// ----- Loop -----
void loop() {
  if (!winding) {
    // Live turn count
    lcd.Fill_Rect(120, 40, 80, 20, BLACK);
    lcd.Print_Number_Int(abs(encoderValue), 120, 40, 0, ' ', 3);

    // Optional: reset count via encoder button
    if (digitalRead(ENCODER_BTN) == LOW) {
      encoderValue = 0;
      delay(200);
      lcd.Fill_Rect(120, 40, 80, 20, BLACK);
      lcd.Print_Number_Int(0, 120, 40, 0, ' ', 3);
    }

    // Confirm via START button
    if (digitalRead(BUTTON_START) == LOW && abs(encoderValue) > 0) {
      setTurns = abs(encoderValue);
      encoderValue = 0;
      delay(200);
      lcd.Fill_Screen(BLACK);
      lcd.Set_Text_colour(CYAN);
      lcd.Print_String("WINDING...", CENTER, 10);
      winding = true;
    }

  } else {
    if (abs(encoderValue) < setTurns) {
      analogWrite(DC_MOTOR_PIN, 180);
      traverseServo.write(map(abs(encoderValue) % 100, 0, 99, 30, 150));

      // Progress bar
      int w = map(abs(encoderValue), 0, setTurns, 0, 218);
      lcd.Fill_Rect(10, 80, 220, 20, GRAY);
      lcd.Fill_Rect(10, 80, w, 20, GREEN);

      // Done Turns
      lcd.Set_Text_colour(WHITE);
      lcd.Print_String("Done Turns:", 10, 110);
      lcd.Fill_Rect(140, 110, 80, 20, BLACK);
      lcd.Print_Number_Int(abs(encoderValue), 140, 110, 0, ' ', 3);

      // Gear
      float ang = (float)abs(encoderValue) / setTurns * TWO_PI * 3;
      drawGear(120, 160, 30, ang);

    } else {
      analogWrite(DC_MOTOR_PIN, 0);
      traverseServo.write(90);
      lcd.Fill_Rect(0, 140, 240, 40, BLACK);
      lcd.Set_Text_colour(RED);
      lcd.Print_String(" COMPLETE! ", CENTER, 150);
      winding = false;
    }
  }

  // Reset
  if (digitalRead(BUTTON_RESET) == LOW) {
    delay(200);
    encoderValue = 0;
    setTurns = 0;
    lcd.Fill_Screen(BLACK);
    lcd.Set_Text_colour(WHITE);
    lcd.Print_String("Set Turns:", 10, 40);
    winding = false;
  }

  delay(20);
}
