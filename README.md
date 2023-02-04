# ESEH22GROUP03
Repository for the project work for the Coure of [Embedded Systems for E-Health](https://corsi.unisa.it/digital-health-and-bioinformatic-engineering/en/teaching/course-units?anno=2021&id=515158), Academic Year 2021/2022, present on Study Plan of Master's Degree in [Digital Health and Bioinformatic Engineering](https://corsi.unisa.it/digital-health-and-bioinformatic-engineering/en), disbursed by [DIEM](https://www.diem.unisa.it/en), [UniSA](https://web.unisa.it/en).

Realization of a smart sensor to measure the temperature and the blood oxygenation of a patient. 
GOALS:
-Measure and aggregate information: the measures sent and shown on the display must be the aggregated value in a minute;
-Show real-time information on a display: aggregated temperature and oxygenation have to be shown;
-Rise alarms on the display in case:
The temperature is higher than a configured threshold -
Blood oxygenation is lower than a configured threshold;
-Periodically send measures through the USB to the PC, each measure must have a timestamp;
-The threshold must be configured according to Covid19 regulations.
_________________________________________________________________________________________________________________________________________________________________________

The Source Code zip file contains all STM32CubeIDE project, configured to work with a STM32F401RET6 Nucleo Board.

The sensors utilized during the project are:
- Sparkfun Pulsoximeter MAX30101 with MAX32664 Smart Hub
- Huaban Temperature Sensor KY 028
- Adafruit Oled Display 128x32 SSD1306
- Elegoo Real Time CLock DS1307

In the zip file, in the Core/Inc and Core/Src directories, you can see all files for each sensor and internal peripherals of the Board.

For any details about the configuration, the system logic, and resources see Project Report PDF file.

All material is licensed by [Creative Commons](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode) and [GPL 3.0](https://www.gnu.org/licenses/gpl-3.0.txt).

For the SSD1306 Driver the code is provided by [taburyak](https://github.com/taburyak/STM32_OLED_SSD1306_HAL_DMA), but the used version is the one provided by [RobertoBenjami](https://github.com/RobertoBenjami/stm32_ssd1306_i2c_dma_hal) with 1 change in ssd1306_init() function, setting Color to White. 
