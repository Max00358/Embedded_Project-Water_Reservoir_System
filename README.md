# Embedded_Project-Water_Reservoir_System
An embedded system design that simulates an irrigation system.

## High Level Pumping System Design
<img width="845" alt="Screen Shot 2023-11-24 at 9 09 47 PM" src="https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/543b882c-8977-44f6-98b0-4c4c85eecc29">

## Reservoir System Demo
https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/4ba04167-3baa-417d-b4a0-9cab5129776c

## Project Details
![Image 64](https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/4bd2bfbf-69b1-485c-8b77-cdbac29d73c6)
![Image 61](https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/f78ed64e-e29a-4393-ad9c-720eee70f4f9)
![Image 97](https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/82e2bc27-03a0-4e52-a492-b5fd5b5260d6)
![Image 35](https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/0209e8c0-7cc4-4964-b10b-12a9bf5edb6a)
![Image 71](https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/b021a1f2-58e5-41f0-a5cf-7ede6a91363e)
![Image](https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/f9f06e28-0540-44e4-bfc1-b2774a88d53c)

## Software & Hardware Used
  * Software
      * STM32Cube development software
      * PuTTY
  * Hardware
      * STM32 NUCLEO-F401RE
      * Micro Servo MG90S
      * DC Brushed Motor
      * Bourns® Digital Potentiometer
      * Distance Sensor: US-100 Ultrasonic Sonar
      * PWM Motor Controller L9110
      * RGB LED: Inolux 5mm Full-Color Through Hole Lamp HV-5RGBXX Series
      * CMOS Digital Buffer CD74HC4050
      * Digital Buffer HCT541
      * Timer Board
      * Speed Sensor
      * UART-USB Adaptor
    
## Irrigation Process
After the water reservoir is filled, the Controller can direct water from the water reservoir to any one of the three zones
for irrigation at a time.

The selection of the zone pipe connection is done by controlling the SERVO. When the SERVO has made a new selection
the RGB LED is Driven to the specified colour. Note that for the INLET Pipe, the RGB colour is set for PURPLE.
The Pump is driven by the MOTOR with a PWM signal from the Controller. The RPM of the pump can be controlled by
the PWM Pulse width to the Motor Controller. Note that the direction of water by the pump with the INLET is OPPOSITE
to the water flow to the ZONES.

The depth of water in the water RESERVOIR may be determined at any time by using the Ultrasonic Distance Sensor. You
should set in code, your determined limits for minimum and maximum Water Level distance values. Then your code
should calculate the percentages for distance values between the limits for display on the Timer Digit Display.

The Water depth must be seen on the Dual Seven-segment display on the Timer Board). The depth will be shown in
percentage terms (0% to 99%). Keep in mind that when the water level is HIGHER in the reservoir, the distance
measurement by the Distance Sensor will be LOWER in value. The Reservoir water level can be represented by some sort
of movable barrier (such as a piece of cardboard etc.) in front of the Distance Sensor.

A Manual Control (Potentiometer) can be used to set the RPM value of the motor for any zone or inlet connection, if the
option is chosen by the terminal, during the SETUP MODE of the embedded system. Alternatively, the RPM parameter
for driving each zone or inlet pipeline connection can be set with an option chosen by the terminal (using the System
Curves in the previous figure).

The Terminal is configured to run with a BI-DIRECTIONAL UART connection to the embedded system. It is used to enter
or display information when the embedded project is operating in each of its operating modes.

## PuTTY Output Example
![Capture](https://github.com/Max00358/Embedded_Project-Water_Reservoir_System/assets/125518862/983f252d-dd2e-434a-9105-6c021e536e2e)
