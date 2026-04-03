# Changes to the E-class amplifier in the final
### Replaced the E-class amplifier in the final stage from a toroidal core to a micro-inductor
Since I got good results, I’m uploading the schematic.
Please take a look at pico_40mCW_TRX_microL.pdf.

Voltage: 5 V
Output: 4.5 W
Efficiency: 80%

However, the C15 value is extremely critical. The rated values of the capacitors used in the Class E amplifier need to be adjusted to account for the difference between the rated values and the actual measured values.

Adjustments are necessary due to variations in the values of the micro-inductors and capacitors!

Testing the Class E Amplifier with a Toroidal Core, Part 2!
I switched the core to a T37-#6 (yellow). It takes more effort than using a micro-inductor, but this seems to provide better stability. The schematic is in “pico_40mCW_TRXClass E_V2.pdf”

JR3XNW 2026/3/31
