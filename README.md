# 🔌 Home Energy Management System prototype with Non-Intrusive SCT for Energy Monitoring

This project provides an end-to-end Energy Management System prototype, using Non-Intrusive device designed for the Japanese electrical environment (100V, 50/60Hz).
It integrates custom low-cost hardware and real household data to estimate energy usage in realtime.

The goal is to provide an accessible and practical prototype to undrstand principle for monitoring household energy usage with a strong focus on real-world deployment.

**Key functionnalities :**
- Forecasting realtime household consumption
- From custom analog circuit design to dashboard (Streamlit) for live monitoring
- Edge preprocessing on ESP32
- Support for Japanese standards (30A breaker, 100V, 50/60Hz)

## Used material

- Current transformer SCT-013-030 (30A/1V)
- 16-bit ADC ADS1115 (860 SPS)
- Adafruit 3.5mm TRRS jack breakout
- ESP32-WROOM-32E
- 2x 10kΩ resistors (divider)
- 1x 1kΩ resistor (anti-aliasing filter)
- 1x 100Ω resistor (protection)
- 1x 10µF capacitor (coupling)
- 1x 470nF capacitor (anti-aliasing filter)
- Male plug and female plug JP type
- AWG14 cables (can use AWG16 cables)
- Proster multimeter with clamper

See `docs/schematic-circuit.png` for complete circuit.

The SCT-013 current sensor connects via a 3.5mm TRRS jack.
The current components allows to get until the 8th harmonics, which can be limited for monitoring more advanced devices such as induction heater, solderer, ...

# Status : 🚧 **In progress**
- Dashboard for visualization
- Individual device signatures recording for classification library
- Consideration of PCM1808 to get more harmonics for advanced devices and Deep Learning Model evaluation for classification, prediction and anomaly detection

# How to run

- Install the project on your computer
- Upload the `main.cpp` program in the ESP32
- Run `streamlit run <path_to_project_folder>/EMS-dashboard.py` (=> the interactive dashboard should open in your browser)
- Follow instructions to setup the system (calibration without load, with load reference, launch monitoring system)

## Future features

Some devices can be tracked but because of the relative low precision of the ADS1115, the system needs to be improved with an audio card 48kHz (PCM1808 ?) to get more harmonics until 24kHz (Nyquist theorem) to classify specific devices such as induction heater, air conditionner, ...



