# Passenger-car-window-control
This README provides an overview of the project scope and its basic features. It outlines the implementation requirements and functionality of the front passenger door window control system using TivaC.
# Project Scope:
The project aims to implement a front passenger door window control system with both passenger and driver control panels. The implementation should include the use of FreeRTOS, limit switches to restrict the window motor's movement, and obstacle detection to prevent jamming. The highlighted areas in red on the provided figure represent the specific components of the car to be implemented.
# System Basic Features:
1.Manual open/close function When the power window switch is pushed or pulled continuously, the window opens or closes until the switch is released.
2.One touch auto open/close function When the power window switch is pushed or pulled shortly, the window fully opens or closes.
3.Window lock function When the window lock switch is turned on, the opening and closing of all windows except the driver’s window is disabled.
4.Jam protection function This function automatically stops the power window and moves it downward about 0.5 second if foreign matter gets caught in the window during one touch auto close operation
