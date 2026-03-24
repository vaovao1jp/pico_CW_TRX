### Hardware Overview
The circuit is basically based on the uSDX circuits of WB2CBA, Barb, etc. The bandwidth has been widened to display the bandscope.
The final stage uses a single 2N7000 tube, with an output of 0.2W to 0.3W.
There are two types of Class E amplifiers: one using a toroidal core (which I believe yields better performance) and one using a microinductor (easier to build). At this output level, the microinductor core will likely not experience magnetic saturation.
The constants of the Class E amplifier are critical, and adjustments are particularly needed for C17 and C18. In the case of a toroidal core, it may be necessary to reduce the number of turns of L2 by one.
For spurious emissions from the second order onward, adjustments are needed for C21 in the case of a toroidal core, and for C21 and C30 in the case of a microinductor.