# plotter

## description
It was (yes, past) a plotter I made using with the servo part of an inkjet printer, controlled with an arduino.

The project is in 3 parts : 
* arduino code for controlling the servo with a software PID. Works like a stepper controller, with a direction and step pin.
* arduino code for the board controlling the above "servo" arduino, a stepper (for the other axis) and the pen.
* A program (linux only) for sending command (HPGL) to the board. This takes as an input an HPGL files, preprocess it (some otpimization, some "fixes" to make it understandable by the parser on the arduino side), and send it to the *machine*.

## Video!
You can see it in action on this video (later, the CDROM carriage was replaced by servo motor for moving the pen).
[![ScreenShot](http://img.youtube.com/vi/7M0CELHHC6E/0.jpg)](https://www.youtube.com/watch?v=7M0CELHHC6E)
