# Nano version of FireFly-Remote

Control your electric skateboard with an Arduino controlled remote. This repository contains the needed software for the remote and the receiver. You can find the 3D-models for the remote (STL files) on Thingiverse: https://www.thingiverse.com/thing:3303495 and read more about the project on: https://www.electric-skateboard.builders/t/simple-3d-printed-nrf-remote-arduino-controlled/28543

I have made a Wiki here on Github, with a few tips and guides on how to build the remote. The Wiki can be found here: https://github.com/DroidSector/FireFly-Nano-Remote/wiki

**Important**: This remote is still in development, and is far from perfect. Stay safe, and remember to wear protective gear!


This fork of the FireFly Nano remote implements the following changes :

- LED lights management : on the receiver side PIN_BACKLIGHT and PIN_FRONTLIGHT are outputting a PWM signal to drive the front and back/brake lights via a mosfet or similar device. The brightness settings and light mode can be changed from the remote's menu

- PPM signal : the receiver's PIN_PPM_THROTTLE pin is reserved for the output of a PPM/PWM standard signal of the throttle channel so the receiver can be connected to the VESC with the 3-pin cable and use the PPM apps instead of the VESC Remote(Nunchuck) standard App. The switch to PPM mode is available in the remote "Receiver -> APP Mode" menu. The 3.3v signal from the receiver has to be converted to 5V via a simple logic level converter. It's then possible to use the Firefly Nano with any of the VESC PPM mode apps, while still having the safety of the Auto-stop mode in case of a remote disconnection.

- Auto-stop mode is now compatible when "reverse mode" is enabled - the board simply stops without going backwards after a disconnection.

- Some settings (from globals.h) can be accessed and changed via the remote's menu. Everything is then saved in the receiver's flash memory.
