/*
  The following Code is for a tuning device, which prints it results using a 2,4''-Display.
  It computes the curent frequency and then prints the difference beetween the targeted frequency
  and the meassured frequency on the Display using a scale to display the value.
*/


#include <arduinoFFT.h>
#include <SPI.h>
#include <TFT_eSPI.h>

const uint8_t SENSOR = 35;

const int SAMPLES = 1024;
const double TARGETFREQUENCY = 442;
static const double TOLERANCE = 30.0;

double VREAL[SAMPLES];
double VIMAG[SAMPLES];

const int SMOOTHINGBUFFERSIZE = 5;
double SMOOTHINGBUFFER[SMOOTHINGBUFFERSIZE];

TFT_eSPI tft = TFT_eSPI();

void setup() {
  // setup TFT screen
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  Serial.begin(115200);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // print scale
  drawScale();

  // configure microfone pin 
  pinMode(SENSOR, INPUT);

  // print current target frequency
  tft.setCursor(100, 40, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(3);
  tft.print(String(TARGETFREQUENCY, 0) + " HZ");
}

double readSamplesFromMicrophone(int samples) {
  // Read sound input from microphone including time measurement  
  unsigned long startTime = millis();
  for (int i = 0; i < samples; i++) {
    const uint16_t value = analogRead(SENSOR);
    VREAL[i] = double(value);
    VIMAG[i] = 0.0;
  }
  unsigned long endTime = millis();
  
  // calculating sample frequency
  double sampleTime = endTime - startTime;
  double sampleFrequency = double(SAMPLES)/(sampleTime/1000.0 /*conversion to seconds*/);

  return sampleFrequency;
}

double computeDominantFrequency(double samplingFrequency) { 
  arduinoFFT FFT = arduinoFFT(VREAL, VIMAG, SAMPLES, samplingFrequency); // Create FFT object
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude(); // Compute magnitudes
  double frequency = FFT.MajorPeak(); // use the main frequency
  return frequency;
}

double smoothFrequency(double frequency) {
  double sum = 0.0;
  for (int i = SMOOTHINGBUFFERSIZE - 2; i >= 0; i--) {
    sum += SMOOTHINGBUFFER[i + 1] = SMOOTHINGBUFFER[i]; // move the values to the next position in the array
  }
  sum += SMOOTHINGBUFFER[0] = frequency; // add new value
  return sum / SMOOTHINGBUFFERSIZE; // calculate average
}

double calculateDeviation(double frequency, double targetFrequency) {
  double diff = frequency - targetFrequency; // calculating difference
  double perc = (diff / targetFrequency) * 100.0; // calculating the number in percent
  return round(perc);
}

void drawScale() {
  int centerX = tft.width() / 2;
  int centerY = tft.height() - 10;
  int radius = 100;
  int steps = 5;

  // Draw semicircle
  tft.drawCircle(centerX, centerY, radius, TFT_WHITE);

  // Draw ticks and labels
  for (int i = -TOLERANCE; i <= +TOLERANCE; i += steps) {
    float angle = map(i, -TOLERANCE, +TOLERANCE, 180, 0) * PI / 180.0;
    int x1 = centerX + radius * cos(angle);
    int y1 = centerY - radius * sin(angle);
    int x2 = centerX + (radius - 10) * cos(angle);
    int y2 = centerY - (radius - 10) * sin(angle);

    tft.drawLine(x1, y1, x2, y2, TFT_WHITE);
    tft.setCursor(x2 - 6, y2 - 10);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.print(i);
  }
}

void drawPointerLine(double value, uint32_t color) {
  static const double centerX = tft.width() / 2;
  static const double centerY = tft.height() - 10;
  static const double radius = 100;

  // compute x and y value
  double angle = map(value, -TOLERANCE, +TOLERANCE, 180, 0) * PI / 180.0;
  double x = centerX + (radius - 20) * cos(angle);
  double y = centerY - (radius - 20) * sin(angle);

  // draw line using the computed values
  if ((angle >= 0.0 && angle <= PI && color != TFT_BLACK) || color == TFT_BLACK) {
    tft.drawLine(centerX, centerY, x, y, color);
  }
}

void drawPointer(double value) {
  static int previousValue = 0;

  // Erase the old pointer
  drawPointerLine(previousValue, TFT_BLACK);

  // Draw the new pointer
  drawPointerLine(value, TFT_RED);

  previousValue = value;
}

void loop() {
  double samplingFrequency = readSamplesFromMicrophone(SAMPLES);
  double frequency = computeDominantFrequency(samplingFrequency);
  double smoothing = smoothFrequency(frequency);
  double deviation = calculateDeviation(smoothing, TARGETFREQUENCY);
  drawPointer(deviation);
}
