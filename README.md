# Baby Pukeko Flight Computer
<img src = "https://github.com/user-attachments/assets/de96c2da-d3a1-403d-b10a-bbc623dcf0df" width="250" height ="400" />


The baby Pukeko FC (Flight Computer) is a basic data logger for brain rot performance. It is also the only flight controller that looks like the bird it was named after. Kinda neat if you ask me!

# **Specifications:**
- STM32F412 CPU in a BGA form factor (requires external 12MHz clock for USB functionality)
- MS5607 24bit Pressure sensor
- H3LIS331DLTR 3-axis Accelerometer with ±400G capability
- LSM6DSO32TR is a 6-axis Accelerometer Gyro combo (±32G, 2000°/s)
- 1GBit flash
- USB Full-speed for Data pulling
- one (1) status LED
- 6 layer stackup
- 10mmx12mm size (Yes, that is smaller than a Micro SD Card)
- Minimum component size: 0402.

I hand-assembled this in my meth lab of a workstation. And Surprisingly, it works!


# **Hardware:**

<img src ="https://github.com/user-attachments/assets/5568417a-6d13-449f-b356-c32e133da979" width="420" height ="480" />

Please look at the .pdf document for a better in-depth description of my ideas. But ya, it's very simple.
It is powered by USB input or a 1s battery, but not both simultaneously. 

# **Software:**


# **Using this Repository:**

# **Improvement and Future work:**
- Reverse polarity protection LDO
- Adding BNO (it will be a little bit bigger)
- Pads for external UART or interfacing with Pyros
- USB-C
- Smaller Flash (maybe)
