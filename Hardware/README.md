# PLTOON Hardware

The hardware for PLTOON was designed using Kicad v8.0. 
All the Kicad files can be found in [this folder](/Hardware/Kicad/)

At the core of the PCB, there is an ESP32S3 devBoard [(Schematic)](https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-DevKitC-1_V1.1_20221130.pdf). It is powered using a DCDC converter module (DC-DC Buck Converter Mini 560) that takes 7.4V  (two 18350 Batteries in series) and gives a regulated 5V output voltage. The purpose of having a DCDC converter is to improve efficiency. Nevertheless, there is an option to use an LDO instead, which needs to be populated on the board. This option has not been used nor tested. 
The ESP32S3 internally regulates the 5V to generate 3.3V, which is available for the rest of the electronics.  

The 18350 batteries can be charged through the J2 connector by switching SW1 to the Off/Chg state. There is no protection for reverse polarity, so it is up to the user to connect it correctly. When no charger is connected, the PLTOON remains OFF with this SW1 position. 
The power from the battery is supplied to the rest of the circuit by moving the switch SW1 to ON, which turns the LED1 ON (if the batteries are connected correctly) and LED2 ON (if the 5V of the DCDC converter is being supplied). Resistors R1 and R6 were chosen to be 330 Ohm, but they can be increased in value if we want to dim the LED light. 

A TB6612FNG H-Bridge is connected to the ESP32S3, and two motors can be driven by connecting them to J8 and J9 connectors. 

A distance sensor like VL53L0X can be connected to J1 using I2C. Other GPIOs are available in case they are needed. Pull-up resistors R2-R5 are not populated in the board, but are available for use. 

A Velocity sensor can be connected to USB-OTG by connecting it directly to the right-hand side USB-c connector of the ESP32S3 devBoard or to connector J7. J7 can also be used to flash the ESP32S3 devBoard using USB-OTG.
An SPI velocity sensor can be connected to connectors J5, J4 and J6 (depending on the signals that are needed). 

A top view of the rendered 3d PCB design in Kicad is shown below. 

![3d Rendering](/Hardware/Pictures/3D_PCB_Kicad.png)

A bottom view of the rendered 3d PCB design in Kicad is shown below. 
![3d Rendering bottom](/Hardware/Pictures/3D_PCB_Kicad_Bottom.png)

# Finished PCB
Some pictures of the assembled PCBs are shown below. 

![PCB1](/Hardware/Pictures/PCB_Charger_Battery.png)


# Schematic in pdf
[Schematic in pdf](/Hardware/Kicad/PL_Toon.pdf)

# BOM 
The bill of Material can be found [here](/Hardware/Kicad/BOM_PLTOON)

# Production Files (Gerber)
The .zip file used for producing the PCBs can be found [here](/Hardware/Production_Files)

# 3d Step file
[3d Step File](/Hardware/Kicad/PL_Toon_3D.step)
