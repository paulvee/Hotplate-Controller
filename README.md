This is a project to modify a UYUE 946C model hotplate and convert it to a reflow soldering station.

The UYUE 946C hotplate is modified by replacing the original controller and by adding an additional heating element.
The new controller is ESP32-based and has a TFT color display.
The selection of an ESP32 is because it will allow a much more responsive display than an Arduino can accomplish.
Hardware SPI is used to communicate with the TFT display and the K-Type temperature sensor.
The software allows you to select different solder pastes, and it will set the reflow profile dynamically.
The main modes for the controller are: Reflow, Heat, Cooling and Warm-up. The parameters for these modes can be set by using a rotary encoder.
The best results will be with lower temperature solder paste (138 degrees). Higher temperatures will work, the hotplate will reach 250 degrees, but the ramp-up is a bit slower than the profile asks for.

The heating elements are controlled by an SSR, and there is a 12V DC output to connect one of more 12V fans to aid in the cooling phase.

The complete description is on my Blog that can be found here: 
https://www.paulvdiyblogs.net/2025/02/creating-hotplate-reflow-station.html

The project is also documented on gthe site of my sponsor, PCBWay, where PCB's can be ordered directly:
https://www.pcbway.com/project/shareproject/Modifying_a_Hotplate_to_a_Reflow_Solder_Station_4dc6241c.html

WARNING:
You will be dealing with live mains voltages, high currents and high temperatures.
Be extra careful, and if you are not qualified, ask someboday that is qualified to help you.

