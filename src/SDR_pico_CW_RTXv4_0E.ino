/*
  pico 40M monoband CW Transceiver V4.0 2026年4月10日
  OLED 128x64 VFO + RX_Bandscope + Keyer
  Adjust the volume range
  Add a waterfall chart 
  pico  click 200MHz 607,609lines 10000 Bandwidth range（±10kHz）
  pico2 click 150MHz 607,609lines 15000 Bandwidth range（±15kHz）
  Button1 Frequency Step Switch / WPM Setting (Long Press)
  Button2 Keyer Mode Switch / Volume Control (Long Press)

  --Libraries used--
  Arduino.h
  Rotary.h       : https://github.com/brianlow/Rotary
  U8g2lib.h      : https://github.com/olikraus/U8g2_Arduino
  Wire.h
  arduinoFFT.h   : v2.0.2
  si5351.h       : https://github.com/etherkit/Si5351Arduino
  EEPROM.h
*/

#include <Arduino.h>
#include <Rotary.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <si5351.h>
#include <EEPROM.h>

// ==============================================================================
// [1] Pin Definitions and Basic Settings (HARDWARE & CONSTANTS)
// ==============================================================================

// --- Pin layout ---
#define I_IN              26  // I-signal analog input
#define Q_IN              27  // Q Signal Analog Input
#define inputPinI         26  // (Alias)
#define inputPinQ         27  // (Alias)
#define speakerPin        16  // Audio PWM Output 16/14
#define PIN_IN1           0   // Rotary Encoder A
#define PIN_IN2           1   // Rotary Encoder B
#define STEP_BUTTON       2   // Frequency Step Switch / WPM Setting (Long Press)
#define KEY_MODE_BUTTON   3   // Keyer Mode Switch / Volume Control (Long Press)
#define PADDLE_DOT        6   // Paddle input for dots
#define PADDLE_DASH       7   // Paddle input for dashing / Vertical-motion key input
#define RX_SW             15  // Transmit/Receive Switch (High during TX)
#define LED_INDICATOR     25  // Processing Indicator LED

// --- SDR・Signal Processing Settings ---
#define SAMPLES           256     // Number of samples in the FFT and shared buffer
#define sampleRate        40000   // Effective sampling rate (Hz)
#define pwmFrequency      44100   // PWM frequency (Hz)
#define CW_AUDIO_OFFSET   70000ULL// BFO offset frequency
#define CW_TONE           700     // Frequency of the CW monitor tone (Hz)

// --- Default Settings ---
const long LOW_FREQ     = 7000000;
const long HI_FREQ      = 7200000;
#define DEFAULT_WPM       20

// --- EEPROM address ---
const int EEPROM_ADDRESS_FREQ    = 0;
const int EEPROM_ADDRESS_STEP    = 4;
const int EEPROM_ADDRESS_WPM     = 8;
const int EEPROM_ADDRESS_KEYMODE = 12;
const int EEPROM_ADDRESS_VOL     = 16;


// ==============================================================================
// [2] Global variables
// ==============================================================================

// --- Frequency・VFO ---
unsigned long FREQ = 7000000;
unsigned long FREQ_OLD = FREQ;
unsigned long long FREQ_ULL = 700000000ULL;
unsigned long long pll_freq = 86400000000ULL;
int STEP = 1000;
int stepMode = 0; // 0:1kHz, 1:100Hz, 2:10Hz

// ★ For Customization: Band Scope Settings
#define SCOPE_SPAN_HZ     15000.0f // Single-sided display range of the scope (e.g., ±15 kHz for 15,000)
#define SCOPE_SENSITIVITY 15.0f    // Amplitude gain of the scope waveform (the higher the value, the higher the waveform rises)
#define SCOPE_OFFSET      3.2f     // Raising the scope's noise floor (to reveal invisible micro-noise)

// --- Message Handling and Status Management ---
bool transmitting = false;
volatile int muteCounter = 0;     // Counter for muting during frequency changes

// --- Audio and DSP Processing ---
volatile float volumeMultiplier = 1.0;
float agcGain = 1.0;
float dcOffsetI = 0.0;
float dcOffsetQ = 0.0;
unsigned long tonePhase = 0;
const unsigned long tonePhaseIncrement = (CW_TONE * 4294967296UL) / 40000;

// ===== Parameters for an IIR filter (40 kHz drive, 700 Hz center frequency, Q=5.0, two stages in series) =====
float iir_x1[2]={0}, iir_x2[2]={0}, iir_y1[2]={0}, iir_y2[2]={0};

// ===== Buffer for 90-degree delay of Q-signals (Hilbert transform) =====
// The optimal value for delaying 700 Hz by 90 degrees at 40 kHz is “14” samples
#define Q_DELAY_SAMPLES 14  
float qDelayBuffer[Q_DELAY_SAMPLES] = {0};
int qDelayIndex = 0;

// --- Data sharing between cores (Core 1 → Core 0) ---
float sharedBufferI[SAMPLES];
float sharedBufferQ[SAMPLES];
volatile bool sharedBufferReady = false;
int sharedIndex = 0;

// --- Screen Display and FFT-Related ---
double vReal[SAMPLES];
double vImag[SAMPLES];
static uint8_t peakR[64];
static uint8_t peakL[64];
static uint8_t peakDecayDiv = 0;
const uint8_t PEAK_DECAY_FRAMES = 1;  // Peak descent rate (the higher the value, the slower the descent)
const uint8_t DC_BLANK_BINS = 0;      // Range to exclude from the display (around the center)
const int VFO_MARKER_X = 63;          // X-coordinate of the VFO center marker
const int PEAK_Y_OFFSET = 10;
const int MARKER_TOP_MARGIN = 22;

// --- Key Management ---
int wpm = DEFAULT_WPM;
bool straightKeyMode = false;
bool sending = false;
bool sendingDot = false;
bool sendingDash = false;
unsigned long dotDuration, dashDuration;
unsigned long elementSpace, charSpace, wordSpace;
bool lastPaddleDot = false;
bool lastPaddleDash = false;

// --- Object Instance ---
Rotary r = Rotary(PIN_IN1, PIN_IN2);

// --- Variables for the waterfall ---
#define WATERFALL_HEIGHT 24
#define THRESHOLD 2
uint8_t waterfallHistory[WATERFALL_HEIGHT][128] = {0};
int waterfallIndex = 0;

Si5351 si5351;

//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); // 標準OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // 0.96 inch
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE); // 1.3 inch

ArduinoFFT<double> FFT;

// ==============================================================================
// [3] Hardware & EEPROM Control
// ==============================================================================

// --- Basic EEPROM Operations ---
void saveToEEPROM(int address, int data) {
  for (int i = 0; i < sizeof(data); i++) {
    EEPROM.write(address + i, (data >> (8 * i)) & 0xFF);
  }
  EEPROM.commit();
}

int readFromEEPROM(int address) {
  int data = 0;
  for (int i = 0; i < sizeof(data); i++) {
    data |= EEPROM.read(address + i) << (8 * i);
  }
  return data;
}

// --- Save/Load Custom Settings ---
void saveFrequencyToEEPROM(unsigned long freq) { saveToEEPROM(EEPROM_ADDRESS_FREQ, freq); }
unsigned long readFrequencyFromEEPROM()        { return readFromEEPROM(EEPROM_ADDRESS_FREQ); }

void saveStepToEEPROM(int step)                { saveToEEPROM(EEPROM_ADDRESS_STEP, step); }
int readStepFromEEPROM()                       { return readFromEEPROM(EEPROM_ADDRESS_STEP); }

void saveWPMToEEPROM(int wpmValue)             { saveToEEPROM(EEPROM_ADDRESS_WPM, wpmValue); }
int readWPMFromEEPROM() {
  int val = readFromEEPROM(EEPROM_ADDRESS_WPM);
  return (val >= 5 && val <= 40) ? val : DEFAULT_WPM;
}

void saveKeyModeToEEPROM(bool mode)            { saveToEEPROM(EEPROM_ADDRESS_KEYMODE, mode ? 1 : 0); }
bool readKeyModeFromEEPROM()                   { return readFromEEPROM(EEPROM_ADDRESS_KEYMODE) == 1; }

void saveVolToEEPROM(int volValue)             { saveToEEPROM(EEPROM_ADDRESS_VOL, volValue); }
int readVolFromEEPROM() {
  int val = readFromEEPROM(EEPROM_ADDRESS_VOL);
  // ★Change this to 10 and 30 as well, and if the value is out of range, return the default value of 10 (1.0x).
  return (val >= 10 && val <= 30) ? val : 10;
} 

// --- Si5351 Frequency Control ---
void Freq_Set() {
  si5351.set_freq_manual(FREQ_ULL - CW_AUDIO_OFFSET, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL - CW_AUDIO_OFFSET, pll_freq, SI5351_CLK1);
  int phase = pll_freq / (FREQ_ULL - CW_AUDIO_OFFSET) + 0.5; // Formula Correction
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase);
  si5351.pll_reset(SI5351_PLLA);
}

void startTransmit() {
  if (!transmitting) {
    transmitting = true;
    si5351.output_enable(SI5351_CLK0, 0); // RX LO OFF
    si5351.output_enable(SI5351_CLK1, 0);
    digitalWrite(RX_SW, HIGH);            // TX Switch ON
    si5351.output_enable(SI5351_CLK2, 1); // TX Carrier ON
    si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK2);
  }
}

void stopTransmit() {
  if (transmitting) {
    transmitting = false;
    si5351.output_enable(SI5351_CLK2, 0); // TX Carrier OFF
    si5351.output_enable(SI5351_CLK0, 1); // RX LO ON
    si5351.output_enable(SI5351_CLK1, 1);
    digitalWrite(RX_SW, LOW);             // RX Switch ON
  }
}

// ==============================================================================
// [4] DSP and Signal Processing
// ==============================================================================

// CW Demodulation (Ultra-High-Speed, High-Sensitivity Version Using an IIR Biquad Filter)
float cwDemodulate(float iSignal, float qSignal, unsigned long currentFreq) {
  // 1. Canceling the reverse sideband using the Hilbert transform approximation (90-degree delay)
  float qDelayed = qDelayBuffer[qDelayIndex];
  qDelayBuffer[qDelayIndex] = qSignal;
  qDelayIndex++;
  if (qDelayIndex >= Q_DELAY_SAMPLES) qDelayIndex = 0;

  float audioSignal = iSignal - qDelayed; 

  // 2. IIR Biquad Bandpass Filter (requires less than one-tenth the computational effort of an FIR filter and is extremely sharp)
  float out = audioSignal;
  for (int i = 0; i < 2; i++) { // Use a two-layer stack (4th order) to improve sharpness
    float y = 0.0077770f * out - 0.0077770f * iir_x2[i] 
              - (-1.9724520f) * iir_y1[i] - (0.9844460f) * iir_y2[i];
    iir_x2[i] = iir_x1[i]; 
    iir_x1[i] = out;
    iir_y2[i] = iir_y1[i]; 
    iir_y1[i] = y;
    out = y;
  }
  return out;
}

float applyAGC(float input) {
  const float targetAmplitude = 0.5;
  const float maxGain = 10.0;
  const float minGain = 0.1;
  const float attackRate = 0.01;
  const float decayRate = 0.0005;
  
  float error = targetAmplitude - fabs(input);
  if (error > 0) agcGain += attackRate * error;
  else           agcGain += decayRate * error;
  
  agcGain = constrain(agcGain, minGain, maxGain);
  return input * agcGain;
}

// ==============================================================================
// [5] Keyer processing 
// ==============================================================================

void calculateTiming() {
  dotDuration = 1200 / wpm;
  dashDuration = dotDuration * 3;
  elementSpace = dotDuration;
  charSpace = dotDuration * 3;
  wordSpace = dotDuration * 7;
}

void initKeyer() {
  calculateTiming();
  sending = false;
  sendingDot = false;
  sendingDash = false;
  transmitting = false;
}

void handleKeyer() {
  static unsigned long stateStartTime = 0;
  static int keyerState = 0; // 0: Idle, 1: Sending (Mark), 2: Space

  unsigned long currentTime = millis();

  // --- Vertical-stroke telegraph key mode ---
  if (straightKeyMode) {
    bool straightKeyDown = (digitalRead(PADDLE_DASH) == LOW);
    if (straightKeyDown && !transmitting)      startTransmit();
    else if (!straightKeyDown && transmitting) stopTransmit();
    return;
  }

  // --- Electric Mode ---
  bool currentDot  = (digitalRead(PADDLE_DOT)  == LOW);
  bool currentDash = (digitalRead(PADDLE_DASH) == LOW);

  // State 0: Standby (waiting for the paddle to be pressed)
  if (keyerState == 0) {
    if (currentDot) {
      keyerState = 1;
      sendingDot = true;
      sendingDash = false;
      stateStartTime = currentTime;
      startTransmit();
    } else if (currentDash) {
      keyerState = 1;
      sendingDot = false;
      sendingDash = true;
      stateStartTime = currentTime;
      startTransmit();
    } else {
      sendingDot = false;
      sendingDash = false;
    }
  } 
  // State 1: Sending (emits a sound for the duration of a short or long beep)
  else if (keyerState == 1) {
    unsigned long duration = sendingDash ? dashDuration : dotDuration;
    if (currentTime - stateStartTime >= duration) {
      stopTransmit();
      keyerState = 2; // Once the transmission is complete, be sure to return to the “blank” state.
      stateStartTime = currentTime;
    }
  } 
  // State 2: Blank (leave space between elements)
  else if (keyerState == 2) {
    if (currentTime - stateStartTime >= elementSpace) {
      keyerState = 0; // When the idle period ends, return to “Standby”
    }
  }
}

// ==============================================================================
// [6] UI and Screen Display
// ==============================================================================

static String fmtMHz(unsigned long f_hz){
  char buf[16];
  unsigned long mhz = f_hz / 1000000UL;
  unsigned long khz = (f_hz % 1000000UL) / 1000UL;
  snprintf(buf, sizeof(buf), "%lu.%03lu", mhz, khz);
  return String(buf);
}

int barLength(double d) {
  float fy = SCOPE_SENSITIVITY * (log10(d) + SCOPE_OFFSET);
  int y = constrain((int)fy, 0, 20);
  return y;
}

// --- Rotary Encoder Interrupt ---
void rotary_encoder() {
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) FREQ += STEP;
    else                  FREQ -= STEP;
  }
  FREQ = constrain(FREQ, LOW_FREQ, HI_FREQ);
  FREQ_ULL = FREQ * 100ULL;
}

// --- Volume Control Mode ---
void changeVolume() {
  detachInterrupt(0); detachInterrupt(1); // Encoder Pause
  
  // ★Change 1: Changed the range to 10–30 (1.0–3.0x)
  int currentVolInt = constrain(volumeMultiplier * 10, 10, 30);

  auto drawVol = [&](int val) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tr);
    u8g2.setCursor(18, 6);
    u8g2.print("VOLUME: ");
    u8g2.print(val / 10.0, 1);
    u8g2.print(" x");
    u8g2.drawFrame(14, 20, 100, 8);
    
    // ★Change 2: Modified the formula so that the value is 0 when the input is 10 and 100 pixels when the input is 30
    u8g2.drawBox(14, 20, (val - 10) * 5, 8);
    
    u8g2.sendBuffer();
  };
  
  drawVol(currentVolInt);
  bool btnPrev = HIGH;

  while (true) {
    unsigned char res = r.process();
    if (res) {
      if (res == DIR_CW) currentVolInt++;
      else               currentVolInt--;
      
      // ★Change 3: Limit the range to 10–30
      currentVolInt = constrain(currentVolInt, 10, 30);
      
      volumeMultiplier = currentVolInt / 10.0;
      drawVol(currentVolInt);
      delay(10);
    }

    bool btn = (digitalRead(KEY_MODE_BUTTON) == LOW);
    if (!btn && btnPrev == LOW) { // Click to exit
      saveVolToEEPROM(currentVolInt);
      break;
    }
    btnPrev = btn;
    delay(5);
  }

  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

// --- WPM Change Mode ---
void changeWPM() {
  detachInterrupt(0); detachInterrupt(1);
  int newWpm = wpm;

  auto draw = [&](int val){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tr);
    u8g2.setCursor(38, 6);
    u8g2.print("WPM: "); u8g2.print(val);
    u8g2.setFont(u8g2_font_micro_tr);
    u8g2.setCursor(15, 20); u8g2.print("Rotate: +/-");
    u8g2.setCursor(15, 26); u8g2.print("Press: Save  Hold: Cancel");
    u8g2.sendBuffer();
  };
  
  draw(newWpm);
  unsigned long holdStart = 0;
  bool btnPrev = HIGH;

  while (true) {
    unsigned char res = r.process();
    if (res) {
      if (res == DIR_CW) newWpm++;
      else               newWpm--;
      newWpm = constrain(newWpm, 5, 40);
      draw(newWpm);
      delay(10);
    }

    bool btn = (digitalRead(STEP_BUTTON) == LOW);
    unsigned long now = millis();
    if (btn && !btnPrev) {
      holdStart = now;
    } else if (!btn && btnPrev) {
      if (holdStart && (now - holdStart) >= 800) break; // 長押しキャンセル
      else {
        wpm = newWpm;
        saveWPMToEEPROM(wpm);
        initKeyer();
        break; // Save
      }
    }
    btnPrev = btn;
    delay(5);
  }

  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);
}

// --- Button Press Detection Routine ---
void handleKeyModeButton() {
  static unsigned long pressStartTime = 0;
  static bool isPressing = false;
  static bool longPressHandled = false;
  static bool lastReading = HIGH;
  const unsigned long debounceDelay = 30;
  const unsigned long longPressDelay = 800;

  bool reading = digitalRead(KEY_MODE_BUTTON);
  if (reading == LOW && lastReading == HIGH) {
    pressStartTime = millis();
    isPressing = true;
    longPressHandled = false;
  } else if (reading == LOW && isPressing) {
    if (!longPressHandled && (millis() - pressStartTime) > longPressDelay) {
      longPressHandled = true;
      changeVolume();
    }
  } else if (reading == HIGH && lastReading == LOW) {
    isPressing = false;
    if (!longPressHandled && (millis() - pressStartTime) > debounceDelay) {
      straightKeyMode = !straightKeyMode;
      saveKeyModeToEEPROM(straightKeyMode);
      
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_8x13B_tr);
      u8g2.setCursor(20, 4);  u8g2.print("KEY MODE");
      u8g2.setCursor(40, 18); u8g2.print(straightKeyMode ? "STRA" : "KEY");
      u8g2.sendBuffer();
      delay(600);
    }
  }
  lastReading = reading;
}

void Fnc_Stp() {
  unsigned long pressStart = millis();
  while (digitalRead(STEP_BUTTON) == LOW) {
    if (millis() - pressStart > 1000) { changeWPM(); return; }
    delay(10);
  }
  
  if (stepMode == 0)      { stepMode = 1; STEP = 100; }
  else if (stepMode == 1) { stepMode = 2; STEP = 10; }
  else                    { stepMode = 0; STEP = 1000; }
  
  saveStepToEEPROM(STEP);
  delay(10);
}

// --- Scope Meter Drawing ---
void showScope() {
  const int BASE_Y = 30; // Extend the height down by 6px
  const int PX_PER_SIDE = 63;
  const int MAX_D = 25;  // Increase the scope's maximum height by 4px
  const float BIN_HZ = (float)sampleRate / (float)SAMPLES;
  int binsTarget = constrain((int)(SCOPE_SPAN_HZ / BIN_HZ + 0.5f), 0, SAMPLES / 2);
  if (binsTarget > (SAMPLES / 2)) binsTarget = (SAMPLES / 2);
  float binsPerPixel = (float)binsTarget / (float)PX_PER_SIDE;

  peakDecayDiv++;
  bool doDecay = (peakDecayDiv >= PEAK_DECAY_FRAMES);
  if (doDecay) peakDecayDiv = 0;

  float offsetBins = (float)CW_TONE / BIN_HZ;

  // Clear the waterfall row for this frame
  for(int i=0; i<128; i++) waterfallHistory[waterfallIndex][i] = 0;

  //=======================================
  // Draw the right half (positive frequencies)
  //=======================================
  int prevX_now = -1, prevY_now = -1, prevX_pk = -1, prevY_pk = -1;
  
  // ★Change 1: Start xi at 0 and set the center of the x-coordinate to 63
  for (int xi = 0; xi <= PX_PER_SIDE; xi++) {
    float exactBin = (xi * binsPerPixel) + offsetBins;
    int bin = constrain((int)(exactBin + 0.5f), 0, (SAMPLES / 2 - 1));
    int d = 0;
    
    if (bin > DC_BLANK_BINS) { 
      d = constrain((barLength(vReal[bin]) + barLength(vImag[bin])) / 2, 0, MAX_D);
    } else {
      // ★Change 2: To smooth out DC spikes while connecting waveforms, interpolate using the value from the adjacent bin
      // (If you want to see the noise in the center, remove this if-else condition and calculate d.)
      int nextBin = DC_BLANK_BINS + 1;
      d = constrain((barLength(vReal[nextBin]) + barLength(vImag[nextBin])) / 2, 0, MAX_D);
    }

    uint8_t pk = peakR[xi];
    if ((uint8_t)d >= pk) pk = (uint8_t)d;
    else if (doDecay && pk > 0) pk--;
    peakR[xi] = pk;

    // ★Change 3: Draw to the right from the center (63)
    int x = 63 + xi;
    waterfallHistory[waterfallIndex][x] = d;

    int y_now = BASE_Y - d;
    int y_pk = constrain(BASE_Y - (int)pk - 1 + PEAK_Y_OFFSET, 0, BASE_Y);
    if (prevX_now >= 0) u8g2.drawLine(prevX_now, prevY_now, x, y_now);
    if (prevX_pk >= 0)  u8g2.drawLine(prevX_pk, prevY_pk, x, y_pk);
    prevX_now = x; prevY_now = y_now; prevX_pk = x; prevY_pk = y_pk;
  }

  //=======================================
  // Drawing the left half (negative frequencies)
  //=======================================
  prevX_now = -1; prevY_now = -1; prevX_pk = -1; prevY_pk = -1;
  
  // ★Change 4: Since the center (xi=0) was drawn on the right side, the left side starts at xi=1 to prevent overlap
  for (int xi = 1; xi <= PX_PER_SIDE; xi++) {
    float exactBin = -(xi * binsPerPixel) + offsetBins;
    int bin = constrain((int)(abs(exactBin) + 0.5f), 0, (SAMPLES / 2 - 1));
    int d = 0;
    
    if (bin > DC_BLANK_BINS) {
      if (exactBin >= 0) {
        d = constrain((barLength(vReal[bin]) + barLength(vImag[bin])) / 2, 0, MAX_D);
      } else {
        d = constrain((barLength(vReal[SAMPLES - bin]) + barLength(vImag[SAMPLES - bin])) / 2, 0, MAX_D);
      }
    } else {
      // Similarly, on the left side, the DC portion is supplemented by the adjacent bin.
      int nextBin = SAMPLES - (DC_BLANK_BINS + 1);
      d = constrain((barLength(vReal[nextBin]) + barLength(vImag[nextBin])) / 2, 0, MAX_D);
    }

    uint8_t pk = peakL[xi];
    if ((uint8_t)d >= pk) pk = (uint8_t)d;
    else if (doDecay && pk > 0) pk--;
    peakL[xi] = pk;

    // ★Change 5: Draw from the center (63) to the left
    int x = 63 - xi;
    waterfallHistory[waterfallIndex][x] = d;

    int y_now = BASE_Y - d;
    int y_pk = constrain(BASE_Y - (int)pk - 1 + PEAK_Y_OFFSET, 0, BASE_Y);
    if (prevX_now >= 0) u8g2.drawLine(prevX_now, prevY_now, x, y_now);
    if (prevX_pk >= 0)  u8g2.drawLine(prevX_pk, prevY_pk, x, y_pk);
    prevX_now = x; prevY_now = y_now; prevX_pk = x; prevY_pk = y_pk;
  }

  // The vertical line in the middle has been removed (commented out)
  u8g2.drawLine(VFO_MARKER_X, (BASE_Y - MAX_D) + MARKER_TOP_MARGIN, VFO_MARKER_X, BASE_Y);

  // Move the frequency scale to the bottom of the screen (Y=56)
  unsigned long leftHz  = (FREQ > 15000) ? (FREQ - 15000) : 0;
  unsigned long midHz   = FREQ;
  unsigned long rightHz = FREQ + 15000;
  
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(0, 56, fmtMHz(leftHz).c_str());
  
  String midS = fmtMHz(midHz);
  int midX = constrain(64 - (midS.length() * 4) / 2, 34, 128);
  u8g2.drawStr(midX, 56, midS.c_str());

  String rS = fmtMHz(rightHz);
  int rX = constrain(128 - (rS.length() * 4), 98, 128);
  u8g2.drawStr(rX, 56, rS.c_str());

  // Upper section information display
  u8g2.setFont(u8g2_font_8x13B_tr);
  String freqt = String(FREQ);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));

  u8g2.setFont(u8g2_font_micro_tr);
  if (transmitting) {
    u8g2.setCursor(90, 6);
    u8g2.print("TX ");
    if (sendingDot) u8g2.print("DOT");
    else if (sendingDash) u8g2.print("DASH");
    else u8g2.print("KEY");
    u8g2.print(" "); u8g2.print(wpm); u8g2.print("WPM");
  }

  u8g2.setCursor(78, 0); u8g2.print("STEP:");
  if (STEP == 1000)      u8g2.drawStr(102, 0, "1000");
  else if (STEP == 100)  u8g2.drawStr(102, 0, " 100");
  else                   u8g2.drawStr(106, 0, " 10");
}

void showS_meter() {
  for (int xi = 1; xi < 64; xi++) {
    int d = (barLength(vReal[xi*2]) + barLength(vImag[xi*2+1])) * 2.0;
    u8g2.drawBox(86, 6, d, 6);
  }
}

void displayWaterfall() {
  for (int y = 0; y < WATERFALL_HEIGHT; y++) {
    // Calculate the indices so that y=0 is the most recent (top) and y=26 is the oldest (bottom)
    int histY = (waterfallIndex - y + WATERFALL_HEIGHT) % WATERFALL_HEIGHT;
    for (int x = 0; x < 128; x++) {
      if (waterfallHistory[histY][x] > THRESHOLD) {
        // Plot in the range from Y=31 to Y=55
        u8g2.drawPixel(x, 31 + y);
      }
    }
  }
  // Advance the index for the next frame
  waterfallIndex = (waterfallIndex + 1) % WATERFALL_HEIGHT;
}

void showGraphics() {
  // Change the break line from Y=24 to 28
  u8g2.drawHLine(0, 30, 128);
  u8g2.drawHLine(0, 54, 128);
  u8g2.drawFrame(86, 6, 42, 6); 
  u8g2.drawBox(0, 30, 2, 2);
  u8g2.drawBox(126, 30, 2, 2);
  
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(78, 6, "S:");
  u8g2.drawStr(78, 0, "STEP:");
  u8g2.drawStr(122, 0, "Hz");
  
  String freqt = String(FREQ);
  u8g2.setFont(u8g2_font_8x13B_tr);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));
}

// ==============================================================================
// [7] SETUP & LOOP (Core 0: UI & Control)
// ==============================================================================

void setup() {
  pinMode(LED_INDICATOR, OUTPUT);
  pinMode(RX_SW, OUTPUT);
  pinMode(PADDLE_DOT, INPUT_PULLUP);
  pinMode(PADDLE_DASH, INPUT_PULLUP);
  pinMode(KEY_MODE_BUTTON, INPUT_PULLUP);
  pinMode(STEP_BUTTON, INPUT_PULLUP);
  
  EEPROM.begin(512);
  Wire.begin();
  
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25001042, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK2, 0);
  digitalWrite(RX_SW, LOW);
  
  r.begin();
  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);
  
  // Load EEPROM Settings
  unsigned long savedFreq = readFrequencyFromEEPROM();
  FREQ = (savedFreq >= LOW_FREQ && savedFreq <= HI_FREQ) ? savedFreq : 7000000;
  wpm = readWPMFromEEPROM();
  straightKeyMode = readKeyModeFromEEPROM();
  volumeMultiplier = readVolFromEEPROM() / 10.0;
  
  int s = readStepFromEEPROM();
  if (s == 10 || s == 100 || s == 1000) {
    STEP = s;
    if (STEP == 1000)      stepMode = 0;
    else if (STEP == 100)  stepMode = 1;
    else                   stepMode = 2;
  } else {
    STEP = 1000;
  }

  initKeyer();
  FREQ_OLD = FREQ;
  Freq_Set();
  
  analogReadResolution(12);
  
  u8g2.begin();
  u8g2.setFlipMode(0);
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  
  u8g2.clearBuffer();
  u8g2.drawStr(34, 8, "7MHz CW TRX v4.0");
  u8g2.drawStr(28, 16, "BandScope & Waterfall");
  u8g2.drawStr(52, 24, "JR3XNW");
  u8g2.sendBuffer();
  delay(1000);
}

void loop() {
  static unsigned long lastUIUpdate = 0;
  
  handleKeyModeButton();   
  handleKeyer();           

  if (!transmitting) {
    // Frequency Change Detection (Immediate Update to Prevent Freezing)
    if (FREQ != FREQ_OLD) {
      muteCounter = 500;  // Mute for reducing popping noises during tuning
      Freq_Set();
      FREQ_OLD = FREQ;
      saveFrequencyToEEPROM(FREQ);
    }

    if (digitalRead(STEP_BUTTON) == LOW) Fnc_Stp();
    
    // Once 256 samples of data have been collected from Core 1, copy them to the FFT buffer
    if (sharedBufferReady) {
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = sharedBufferI[i];
        vImag[i] = sharedBufferQ[i];
      }
      sharedBufferReady = false;

      // Screen refresh (every 50 ms to reduce rendering load)
      if (millis() - lastUIUpdate >= 50) {
        digitalWrite(LED_INDICATOR, HIGH);
        FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
        FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);

        u8g2.clearBuffer();
        showS_meter();
        showScope();
        displayWaterfall(); // Add Waterfall
        showGraphics();
        u8g2.sendBuffer();
        digitalWrite(LED_INDICATOR, LOW);
        lastUIUpdate = millis();
      }
    }
  } else {
    // Screen refresh upon sending
    if (millis() - lastUIUpdate >= 50) {
      u8g2.clearBuffer();
      showGraphics();
      u8g2.sendBuffer();
      lastUIUpdate = millis();
    }
  }
}

// ==============================================================================
// [8] SETUP1 & LOOP1 (Core 1: Audio & DSP processing)
// ==============================================================================

void setup1() {
  pinMode(speakerPin, OUTPUT);
  pinMode(inputPinI, INPUT);
  pinMode(inputPinQ, INPUT);
  
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(pwmFrequency);
}

void loop1() {
  static unsigned long lastSampleTime = 0;
  unsigned long now = micros();
  
  // ★Pacing: Execute at exactly 25 μs (40 kHz) intervals to eliminate jitter
  unsigned long targetInterval = 25; 
  if (now - lastSampleTime < targetInterval) return; 
  lastSampleTime += targetInterval; // By adding the default value instead of using `now`, we prevent the discrepancy from accumulating

  static bool wasTransmitting = false;
  static float smoothMute = 1.0f;

  // ★Addition: Variables for managing the envelope (smooth volume rise and fall)
  static float txEnvelope = 0.0f;
  const float envelopeStep = 0.002f; // The speed of the transition. Fades over approximately 12.5 milliseconds at 0.002

  // ★Additional: Smoothly increase or decrease the envelope value between 0.0 and 1.0 depending on the transmission status
  if (transmitting) {
    txEnvelope += envelopeStep;
    if (txEnvelope > 1.0f) txEnvelope = 1.0f;
  } else {
    txEnvelope -= envelopeStep;
    if (txEnvelope < 0.0f) txEnvelope = 0.0f;
  }

  // ★Change: Instead of toggling “transmitting” on and off, play the sound as long as the envelope value is greater than 0.
  if (txEnvelope > 0.0f) {
    tonePhase += tonePhaseIncrement;
    float sineValue = sin(2.0 * PI * (tonePhase / 4294967296.0));
    
    float monitorVolume = 0.1; // Preferred volume setting
    
    // ★Modification: Apply the monitor volume and envelope to a sine wave to create a smooth volume transition
    float currentVol = monitorVolume * txEnvelope;
    uint16_t pwmOutput = (uint16_t)(((sineValue * currentVol) + 1.0) * 2047.5);
    
    analogWrite(speakerPin, constrain(pwmOutput, 0, 4095));
    wasTransmitting = true;
    return;
  }

  if (wasTransmitting) {
    dcOffsetI = 0; dcOffsetQ = 0;
    sharedIndex = 0;
    wasTransmitting = false;
  }

  // 1. Sampling (oversampled twice to save time)
  long sumI = 0, sumQ = 0;
  for (int i = 0; i < 2; i++) {
    sumI += analogRead(inputPinI);
    sumQ += analogRead(inputPinQ);
  }
  float rawI = ((float)sumI / 2.0f / 2047.5f) - 1.0f;
  float rawQ = ((float)sumQ / 2.0f / 2047.5f) - 1.0f;

  // 2. Mute (Pop Noise Suppression) and DC Offset Control
  if (muteCounter > 0) {
    muteCounter--;
    smoothMute *= 0.80f; 
    rawI -= dcOffsetI; 
    rawQ -= dcOffsetQ;
  } else {
    smoothMute = (smoothMute * 0.80f) + (1.0f * 0.20f); 
    dcOffsetI = (dcOffsetI * 0.999f) + (rawI * 0.001f);
    dcOffsetQ = (dcOffsetQ * 0.999f) + (rawQ * 0.001f);
    rawI -= dcOffsetI; 
    rawQ -= dcOffsetQ;
  }

  float mutedI = rawI * smoothMute;
  float mutedQ = rawQ * smoothMute;

  // 3. Share data for the screen (FFT)
  sharedBufferI[sharedIndex] = mutedI;
  sharedBufferQ[sharedIndex] = mutedQ;
  sharedIndex++;
  if (sharedIndex >= SAMPLES) {
    sharedIndex = 0;
    sharedBufferReady = true;
  }

  // 4. CW Demodulation (★ Eliminate downsampling and pass the signal through an IIR filter at 40 kHz)
  float demodulated = cwDemodulate(mutedI, mutedQ, FREQ);

  // 5. AGC applied
  float agcOutput;
  if (smoothMute < 0.9f) {
    agcOutput = demodulated * agcGain; 
  } else {
    agcOutput = applyAGC(demodulated);
  }
  agcOutput *= volumeMultiplier;
  
  // 6. PWM Output (★To eliminate 10 kHz current fluctuation noise, the output is smoothly generated at 40 kHz every time)
  uint16_t pwmOutput = (uint16_t)((agcOutput + 1.0f) * 2047.5f);
  analogWrite(speakerPin, constrain(pwmOutput, 0, 4095));
}