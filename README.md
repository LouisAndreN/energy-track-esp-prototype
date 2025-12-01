# Non-Intrusive Load Monitoring System & Energy Monitoring

This project provides a full end-to-end Non-Intrusive Load Monitoring (NILM) system designed for the Japanese electrical environment (100V, 50/60Hz).
It integrates custom low-cost hardware, real household data, and lightweight machine learning models to estimate appliance-level energy usage in real time.

The goal is to provide an accessible and practical system for monitoring household energy usage, detecting anomalies, and predicting future consumption patterns — with a strong focus on real-world deployment rather than academic demonstration.

**Key functionnalities :**
- Real-time appliance-level energy disaggregation (fridge, AC, IH cooking heater, computer, smartphone charging, lighting, etc.)
- Forecasting of total household consumption and prediction of device usage patterns
- From Custom analog conditioning circuit design to ML deployment
- Dashboard for live monitoring and historical analysis
- Edge preprocessing (ESP32) + Central ML (Raspberry Pi 5)
- Support for Japanese standards (seasonal patterns, 30A breaker, 100V, 50/60Hz)

## Used material

- Current transformers : SCT-013-005 (5A/1V) and SCT-013-030 (30A/1V)
- 16-bit ADC (ADS1115, 860 SPS)
- Adafruit 3.5mm TRRS jack breakout
- ESP32 S3 R16N8 (or ESP32-WROOM-32E)
- 2× 10kΩ resistors (divider)
- 1× 1kΩ resistor (anti-aliasing filter)
- 1× 100Ω resistor (protection)
- 1× 10µF capacitor (coupling)
- 1× 470nF capacitor (anti-aliasing filter)
- Male plug and female plug JP type
- AWG14 cables
- Proster multimeter

See `docs/schematic-circuit.png` for complete circuit.

The SCT-013 current sensor connects via a 3.5mm TRRS jack.


## Future features

Some devices can be tracked but because of the relative low precision of the ADS1115, the system needs to be improved with an audio card 48kHz (PCM1808 ?) to get more harmonics until 24kHz (Nyquist theorem) to detect specific devices such as induction heater.

# 日本語

## 概要



