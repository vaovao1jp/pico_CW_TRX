# Raspberry Pi Pico SDR CW Transceiver v3.22 Documentation
### 1. Overview
This program is a 40m (7MHz) monoband CW SDR transceiver (version 3.22) designed for the Raspberry Pi Pico. It features a VFO, a receiver bandscope, and a built-in CW keyer, all displayed on a 128x32 OLED screen. The system uses a multicore architecture to separate signal processing from the user interface.

### 2. Hardware Specifications & Pinout
I/Q Analog Input: GPIO 26 (I Signal), GPIO 27 (Q Signal) 

Audio Output: GPIO 16 (PWM Output) 

Rotary Encoder: GPIO 0 (Pin A), GPIO 1 (Pin B) 

Button 1 (Step/WPM): GPIO 2 

Button 2 (Key Mode/Volume): GPIO 3 

CW Paddle: GPIO 6 (Dot), GPIO 7 (Dash / Straight Key) 

TX/RX Switch (RX_SW): GPIO 15 (High during TX) 

Status LED: GPIO 25 

### 3. UI & Operation
Frequency Tuning: Rotate the encoder to tune the frequency.

Change Frequency Step: A short press of the STEP button cycles through 1kHz, 100Hz, and 10Hz steps.

Change CW Speed (WPM): A long press (over 800ms) of the STEP button enters WPM setting mode, adjustable from 5 to 40 WPM via the encoder.

Change Keyer Mode: A short press of the KEY MODE button toggles between straight key and electronic keyer modes.

Adjust Volume: A long press of the KEY MODE button enters the volume adjustment mode, ranging from 1.0x to 3.0x.

Bandscope: Displays a spectrum spanning ±15.0kHz from the center frequency using FFT.

### 4. EEPROM Settings
Settings are stored in the EEPROM and restored upon boot.

Address 0: Frequency 

Address 4: Frequency Step 

Address 8: WPM (CW Speed) 

Address 12: Keyer Mode 

Address 16: Volume 

### 5. Technical Deep Dive: Multicore & Shared Buffers
The program leverages the dual-core architecture to separate highly real-time audio processing from heavy UI/FFT rendering tasks.

**Core 1** (DSP & Audio): Samples the I/Q signals at precise 25-microsecond intervals (40kHz). After processing (e.g., DC offset removal), data is stored in arrays sharedBufferI and sharedBufferQ. Once it reaches SAMPLES (256), the sharedBufferReady flag is set to true to notify Core 0.

**Core 0** (UI & FFT): Constantly monitors the flag. When true, it immediately copies data into the FFT arrays (vReal, vImag) , and proceeds to compute the FFT to render the bandscope.

### 6. Technical Deep Dive: CW Demodulation & IIR Filter
**Sideband Cancellation:** To approximate a 90-degree phase shift for the 700Hz tone at a 40kHz sampling rate, the Q signal is delayed by 14 samples. Subtracting this from the I signal effectively cancels the unwanted opposite sideband.

**IIR Biquad Filter:** Uses an IIR Biquad filter which requires significantly less computational power compared to FIR filters. Configured for 40kHz drive, 700Hz center, and Q=5.0. It runs in two series stages (4th-order) via a loop to improve sharpness.
The difference equation is:

*y[n] = b0 x[n]−b2 x[n−2]−a1 y[n−1]−a2 y[n−2]*

(Implementation: *y = 0.0108543f * out - 0.0108543f * iir_x2[i] - (-1.9663346f) * iir_y1[i] - (0.9782914f) * iir_y2[i];* )

**AGC** (Automatic Gain Control): Targets an amplitude of 0.5 to automatically adjust signal gain.