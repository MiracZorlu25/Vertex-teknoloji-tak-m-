// remote_main_modular.ino
// Modular main sketch for remote/joystick-side Deneyap Card
// Reads joystick/buttons, sends commands via Bluetooth, and displays status
// Designed for clarity, modularity, and easy expansion

// #include <BluetoothSerial.h> // Uncomment for ESP32/Deneyap
// #include <Wire.h>            // For I2C OLED
// #include <Adafruit_SSD1306.h> // For OLED display (optional)

// ----- Pin Definitions -----
#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define BUTTON_FWD 2
#define BUTTON_BWD 3
#define BUTTON_LEFT 4
#define BUTTON_RIGHT 5
#define BUTTON_STOP 6
// Add more buttons as needed

// ----- Bluetooth Serial -----
// BluetoothSerial SerialBT; // Uncomment for ESP32/Deneyap

// ----- OLED Display (optional) -----
// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ----- Global Variables -----
int joyX = 0, joyY = 0;
bool lastButtonState[5] = {false, false, false, false, false};
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // ms

// ----- Function Prototypes -----
void setupInputs();
void readJoystick();
void readButtons();
void sendCommand(char);
void updateDisplay(const char* status);

void setup() {
  Serial.begin(9600); // For HC-05 or Serial monitor
  // SerialBT.begin("RobotRemote"); // Uncomment for ESP32/Deneyap
  setupInputs();
  // OLED setup (optional)
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // display.clearDisplay();
  // updateDisplay("Ready");
}

void loop() {
  readJoystick();
  readButtons();
  // Optionally, update display with status
  // updateDisplay("Running");
  delay(10);
}

void setupInputs() {
  pinMode(BUTTON_FWD, INPUT_PULLUP);
  pinMode(BUTTON_BWD, INPUT_PULLUP);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);
  // Add more buttons as needed
}

void readJoystick() {
  joyX = analogRead(JOY_X_PIN);
  joyY = analogRead(JOY_Y_PIN);
  // Map joystick values to commands
  if (joyY > 800) sendCommand('F'); // Forward
  else if (joyY < 200) sendCommand('B'); // Backward
  else if (joyX < 200) sendCommand('L'); // Left
  else if (joyX > 800) sendCommand('R'); // Right
  else sendCommand('S'); // Stop
}

void readButtons() {
  bool buttonStates[5];
  buttonStates[0] = !digitalRead(BUTTON_FWD);
  buttonStates[1] = !digitalRead(BUTTON_BWD);
  buttonStates[2] = !digitalRead(BUTTON_LEFT);
  buttonStates[3] = !digitalRead(BUTTON_RIGHT);
  buttonStates[4] = !digitalRead(BUTTON_STOP);
  // Send command only on button press (not hold)
  char cmds[5] = {'F', 'B', 'L', 'R', 'S'};
  for (int i = 0; i < 5; i++) {
    if (buttonStates[i] && !lastButtonState[i]) {
      sendCommand(cmds[i]);
    }
    lastButtonState[i] = buttonStates[i];
  }
}

void sendCommand(char cmd) {
  unsigned long now = millis();
  if (now - lastSendTime > sendInterval) {
    Serial.write(cmd); // For HC-05 or Serial monitor
    // SerialBT.write(cmd); // Uncomment for ESP32/Deneyap
    lastSendTime = now;
  }
}

void updateDisplay(const char* status) {
  // Optional: update OLED with current status
  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 0);
  // display.print(status);
  // display.display();
}

// Expand with more buttons, joystick axes, or display features as needed. 