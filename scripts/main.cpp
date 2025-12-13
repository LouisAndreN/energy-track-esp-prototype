#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <arduinoFFT.h>

// ================== HARDWARE ==================
Adafruit_ADS1115 ads;

// ================== ADC CONFIG =================
constexpr float ADS1115_LSB_MV = 0.0625;  // GAIN_TWO ±2.048V

// ================== FFT CONFIG =================
#define SAMPLES 128
#define SAMPLING_FREQ 860
#define FFT_BINS (SAMPLES / 2)

ArduinoFFT<double> FFT;

// ================== BUFFERS ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
float rawSamples[SAMPLES];

// ================== CALIBRATION =================
float current_multiplier = 1.0;   // A / V
float gain_calibrated   = 1.0;

// ================== GRID =======================
float grid_frequency = 50.0;  // Hz
float grid_voltage   = 100.0; // V RMS

// ================== TIMING =====================
unsigned long last_fft_time = 0;
const unsigned long FFT_INTERVAL = 500;  // ms


void performFFT() {
    unsigned long start_time = micros();

    // -------- ACQUISITION --------
    for (int i = 0; i < SAMPLES; i++) {
        int16_t adc = ads.readADC_SingleEnded(0);
        float voltage_mv = adc * ADS1115_LSB_MV;

        rawSamples[i] = voltage_mv;
        vReal[i] = voltage_mv;
        vImag[i] = 0;
    }

    unsigned long acquisition_time = micros() - start_time;

    // -------- VREF (DC OFFSET) --------
    float vref_mv = 0;
    for (int i = 0; i < SAMPLES; i++) {
        vref_mv += rawSamples[i];
    }
    vref_mv /= SAMPLES;

    // -------- RMS (TIME DOMAIN) --------
    float sum_sq = 0;
    for (int i = 0; i < SAMPLES; i++) {
        float ac = rawSamples[i] - vref_mv;
        sum_sq += ac * ac;
        vReal[i] = ac;  // Prepare FFT buffer
    }

    float rms_voltage_mv = sqrt(sum_sq / SAMPLES);
    float rms_voltage_v  = rms_voltage_mv / 1000.0;

    // -------- FFT --------
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    float freq_resolution = (float)SAMPLING_FREQ / SAMPLES;

    // -------- CURRENT & POWER --------
    float irms  = rms_voltage_v * current_multiplier * gain_calibrated;
    float power = irms * grid_voltage;

    // -------- THD --------
    int fundamental_bin = round(grid_frequency / freq_resolution);
    float fundamental = vReal[fundamental_bin];

    float harmonics_sq = 0;
    for (int h = 2; h <= 16; h++) {
        int bin = fundamental_bin * h;
        if (bin < FFT_BINS) {
            harmonics_sq += vReal[bin] * vReal[bin];
        }
    }

    float thd = (fundamental > 0) ? sqrt(harmonics_sq) / fundamental : 0;

    // -------- JSON OUTPUT --------
    Serial.print("{");
    Serial.print("\"timestamp\":"); 
    Serial.print(millis());
    Serial.print(",\"acquisition_us\":"); 
    Serial.print(acquisition_time);

    Serial.print(",\"v_diff_mv\":"); 
    Serial.print(rms_voltage_mv, 3);
    Serial.print(",\"v_rms\":"); 
    Serial.print(rms_voltage_v, 4);
    Serial.print(",\"i_rms\":"); 
    Serial.print(irms, 4);
    Serial.print(",\"power\":"); 
    Serial.print(power, 2);
    Serial.print(",\"thd\":"); 
    Serial.print(thd, 4);

    Serial.print(",\"gain\":"); 
    Serial.print(gain_calibrated, 4);
    Serial.print(",\"fft_bins\":"); 
    Serial.print(FFT_BINS);
    Serial.print(",\"freq_resolution\":"); 
    Serial.print(freq_resolution, 4);
    Serial.print(",\"sampling_freq\":"); 
    Serial.print(SAMPLING_FREQ);
    Serial.print(",\"vref_mv\":"); 
    Serial.print(vref_mv, 2);

    Serial.print(",\"fft\":[");
    for (int i = 0; i < FFT_BINS; i++) {
        Serial.print(vReal[i], 4);
        if (i < FFT_BINS - 1) Serial.print(",");
    }
    Serial.print("]}");

    Serial.println();
}


void serialEvent() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("CAL_CURRENT:")) {
        current_multiplier = cmd.substring(12).toFloat();
    }
    else if (cmd.startsWith("CAL_GAIN:")) {
        gain_calibrated = cmd.substring(9).toFloat();
    }
    else if (cmd.startsWith("GRID_FREQ:")) {
        grid_frequency = cmd.substring(10).toFloat();
    }
    else if (cmd.startsWith("GRID_VOLT:")) {
        grid_voltage = cmd.substring(10).toFloat();
    }
}



void setup() {
    Serial.begin(115200);

    if (!ads.begin()) {
        Serial.println("{\"error\":\"ADS1115 not found\"}");
        while (1);
    }

    ads.setGain(GAIN_TWO);
    ads.setDataRate(RATE_ADS1115_860SPS);

    Serial.println("{\"status\":\"ready\",\"mode\":\"single_ended_A0\",\"rms\":\"time_domain\"}");
}



void loop() {
    if (millis() - last_fft_time >= FFT_INTERVAL) {
        last_fft_time = millis();
        performFFT();
    }
    serialEvent();
}

