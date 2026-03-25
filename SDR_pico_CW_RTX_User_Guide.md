# SDR_pico_CW_RTXv3.22 User Guide

## 1. Overview

This document explains how to use **SDR_pico_CW_RTXv3.22.ino**, a CW SDR transceiver for **Raspberry Pi Pico** developed in **Arduino IDE**.

This program includes the following main features:

- 7MHz band CW transceiver
- Si5351-based local oscillator and transmit carrier generation
- OLED display with frequency, S-meter, and band scope
- Rotary encoder tuning
- Electronic keyer and straight key mode
- Adjustable WPM and audio volume
- EEPROM storage of settings
- Dual-core processing on RP2040

This guide also explains how to customize the program by changing constants and variables, how to modify the CW band-pass filter, and how to adapt the program from **7MHz** to **14MHz** operation.

## 2. Required Hardware

Main hardware assumed by this program:

- Raspberry Pi Pico
- Si5351 module
- OLED display (SSD1306, 128x32)
- Rotary encoder
- Two push buttons
- CW paddle or straight key
- I/Q analog input circuit connected to ADC GPIO26 and GPIO27
- Audio output circuit connected to GPIO16 PWM output
- TX/RX switching circuit on GPIO15

## 3. Libraries Used

The source code uses the following libraries:

- `Arduino.h`
- `Rotary.h`
- `U8g2lib.h`
- `Wire.h`
- `arduinoFFT.h` (v2.0.2)
- `si5351.h` (Etherkit Si5351 library)
- `EEPROM.h`

## 4. Pin Assignment

The pin definitions in the sketch are as follows:

| Function | GPIO | Description |
|---|---:|---|
| I input | 26 | I signal ADC input |
| Q input | 27 | Q signal ADC input |
| Speaker PWM | 16 | Audio output |
| Encoder A | 0 | Rotary encoder A |
| Encoder B | 1 | Rotary encoder B |
| Step button | 2 | Step change / WPM setting |
| Key mode button | 3 | Key mode change / volume setting |
| Paddle dot | 6 | Dot input |
| Paddle dash | 7 | Dash input / straight key |
| RX/TX switch | 15 | TX control output |
| LED | 25 | Status indicator |

## 5. Basic Operation

### 5.1 Power On

When the Pico starts, the OLED displays the startup screen:

- `7MHz CW TRX v3.22`
- `BandScope & Keyer`
- `JR3XNW`

After startup, the device reads the saved settings from EEPROM:

- Frequency
- Step size
- WPM
- Key mode
- Volume

### 5.2 Frequency Tuning

Rotate the rotary encoder to change frequency.

Available step sizes:

- 1000 Hz
- 100 Hz
- 10 Hz

The frequency is limited by the band range defined in the constants:

- `LOW_FREQ`
- `HI_FREQ`

### 5.3 Step Button

Short press of `STEP_BUTTON` changes the tuning step in this order:

- 1000 Hz → 100 Hz → 10 Hz → 1000 Hz

Long press opens the WPM setting mode.

### 5.4 Key Mode Button

Short press of `KEY_MODE_BUTTON` toggles key mode:

- `KEY` = electronic keyer mode
- `STRA` = straight key mode

Long press opens the volume setting mode.

### 5.5 CW Transmission

In straight key mode, pressing the key directly turns TX on.

In electronic keyer mode:

- Dot paddle sends a dot
- Dash paddle sends a dash
- TX is automatically keyed according to WPM timing

## 6. Screen Display / 画面表示

The OLED shows:

- Current frequency
- STEP size
- S-meter bar
- Band scope
- TX status during sending
- WPM during TX

When transmitting, the display shows:

- `TX DOT`
- `TX DASH`
- `TX KEY`

## 7. Important Constants and Variables

The following constants and variables are especially important for customization.

## 8. Changing Functions by Editing Constants and Variables

### 8.1 Changing the Tuning Range

To change the tuning range, edit these constants:

```cpp
const long LOW_FREQ = 7000000;
const long HI_FREQ  = 7200000;
```

For example, if you want a narrower 7MHz CW segment:

```cpp
const long LOW_FREQ = 7000000;
const long HI_FREQ  = 7040000;
```

### 8.2 Changing the Default Frequency

Edit these variables:

```cpp
unsigned long FREQ = 7000000;
unsigned long long FREQ_ULL = 700000000ULL;
```

`FREQ_ULL` is `FREQ × 100` because the Si5351 function uses 0.01 Hz units.

### 8.3 Changing the CW Side Tone

Edit:

```cpp
#define CW_TONE 700
```

For example, to use an 800Hz tone:

```cpp
#define CW_TONE 800
```

This affects:

- CW monitor tone
- Band scope offset display behavior
- Audio tone heard during TX

### 8.4 Changing the Receive Offset 

Edit:

```cpp
#define CW_AUDIO_OFFSET 70000ULL
```

This value corresponds to **700 Hz** because the Si5351 frequency unit is 0.01 Hz.

If you change it to 600 Hz:

```cpp
#define CW_AUDIO_OFFSET 60000ULL
```

Then the LO offset changes accordingly.

### 8.5 Changing the Audio Output Level

The runtime volume adjustment range is currently:

- 1.0x to 3.0x

This is defined in the volume setting logic:

```cpp
int currentVolInt = constrain(volumeMultiplier * 10, 10, 30);
```

If you want a smaller range, for example 1.0x to 2.0x:

```cpp
int currentVolInt = constrain(volumeMultiplier * 10, 10, 20);
```

You should also adjust related range checks in `readVolFromEEPROM()` and `changeVolume()`.

### 8.6 Changing the WPM Range

The current WPM setting range is:

- 5 to 40 WPM

This is controlled by:

```cpp
return (val >= 5 && val <= 40) ? val : DEFAULT_WPM;
newWpm = constrain(newWpm, 5, 40);
```

### 8.7 Changing the Scope Display

These values affect the scope display:

```cpp
#define SCOPE_SPAN_HZ     15000.0f
#define SCOPE_SENSITIVITY 14.0f
#define SCOPE_OFFSET      3.1f
```

- `SCOPE_SPAN_HZ`: display width on each side
- `SCOPE_SENSITIVITY`: waveform height
- `SCOPE_OFFSET`: noise floor lift

## 9. How the CW Band-Pass Filter Works

This program uses a **two-stage IIR biquad band-pass filter** inside `cwDemodulate()`.

The comments in the code describe it as:

- 40kHz processing
- 700Hz center
- Q = 5.0
- two stages in series

The filtering section is:

```cpp
float y = 0.0108543f * out - 0.0108543f * iir_x2[i] 
          - (-1.9663346f) * iir_y1[i] - (0.9782914f) * iir_y2[i];
```

This means the current filter is optimized for a CW tone around **700Hz**.

## 10. How to Change the CW Band-Pass Filter

### 10.1 If You Want a Different Center Frequency

If you want to change the CW passband center from **700Hz** to another frequency such as **600Hz** or **800Hz**, you must modify **both** of the following:

1. `CW_TONE`
2. The IIR filter coefficients in `cwDemodulate()`

Changing only `CW_TONE` changes the monitor tone, but **does not retune the band-pass filter itself**.

### 10.2 If You Want a Wider or Narrower Filter

To make the filter wider or narrower, change the biquad coefficients.

A higher Q value gives:

- narrower filter
- better selectivity
- more ringing possible

A lower Q value gives:

- wider filter
- easier tuning
- less selectivity

### 10.3 Practical Method

A practical workflow is:

1. Decide the desired CW tone frequency (for example 700Hz, 600Hz, 800Hz)
2. Decide the desired bandwidth
3. Recalculate the IIR band-pass coefficients for:
   - sample rate = 40000 Hz
   - target center frequency = desired tone
   - desired Q
4. Replace the coefficient values in `cwDemodulate()`
5. Test with actual received CW signals

### 10.4 Important Note About Q Delay

The program also uses this setting:

```cpp
#define Q_DELAY_SAMPLES 14
```

This value approximates a 90-degree shift for 700Hz at 40kHz.
If you significantly change the CW audio frequency, this delay may also need adjustment.

## 11. Changing from 7MHz to 14MHz

### 11.1 What Must Be Changed

To adapt this sketch from **7MHz** operation to **14MHz**, modify at least the following:

1. Band limits
2. Default frequency
3. Frequency initialization values
4. Startup text on OLED
5. RF front-end / band-pass hardware if required

### 11.2 Example for 14MHz Band

For a 14MHz version, for example around the 20m amateur band CW segment, change:

```cpp
const long LOW_FREQ = 14000000;
const long HI_FREQ  = 14350000;

unsigned long FREQ = 14000000;
unsigned long long FREQ_ULL = 1400000000ULL;
```

Also update the fallback in `setup()`:

```cpp
FREQ = (savedFreq >= LOW_FREQ && savedFreq <= HI_FREQ) ? savedFreq : 14000000;
```

### 11.3 Update the Startup Screen

Change the OLED startup message:

```cpp
u8g2.drawStr(32, 8, "14MHz CW TRX v3.22");
```

### 11.4 About Si5351 Settings

The `pll_freq` value itself does not necessarily need to change when moving from 7MHz to 14MHz, because the code uses:

- `set_freq_manual()`
- calculated phase relationship

However, after changing bands you should verify:

- quadrature output quality
- receive sensitivity
- TX carrier purity
- actual output frequency accuracy

### 11.5 Hardware Caution

Even if the software is changed to 14MHz, the radio hardware must also support 14MHz.

Examples:

- RF band-pass filter
- low-pass filter for TX
- matching network
- front-end gain characteristics

Software alone is not enough if the analog hardware is optimized only for 7MHz.

## 12. EEPROM Settings

The following values are stored in EEPROM:

- Frequency
- Step
- WPM
- Key mode
- Volume

Addresses used:

```cpp
const int EEPROM_ADDRESS_FREQ    = 0;
const int EEPROM_ADDRESS_STEP    = 4;
const int EEPROM_ADDRESS_WPM     = 8;
const int EEPROM_ADDRESS_KEYMODE = 12;
const int EEPROM_ADDRESS_VOL     = 16;
```

If you significantly change the design, it may be useful to clear EEPROM once.

## 13. Summary

This sketch is a compact and practical CW SDR transceiver for Raspberry Pi Pico. Its behavior can be changed easily by editing constants and variables.

The most important points are:

- change `LOW_FREQ`, `HI_FREQ`, `FREQ`, and `FREQ_ULL` for band changes
- change `CW_TONE` and IIR coefficients together when changing CW audio center
- change `SCOPE_*` values for display tuning
- verify analog hardware when moving from 7MHz to 14MHz
