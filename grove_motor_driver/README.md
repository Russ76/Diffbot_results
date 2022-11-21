These Python files for Diffbot will control the Grove motor driver board v1.3

The Raspberry Pi can control the board without the Teensy or Arduino, using i2c.

Remove the jumper from the board and connect all four i2c lines up to the GPIO pins on Raspi, all together, near the first pins. Install the two Raspi software packages Franz recommends on his documentation site for Diffbot. If the Raspi doesn't see the board, restart the Grove board and try again.

