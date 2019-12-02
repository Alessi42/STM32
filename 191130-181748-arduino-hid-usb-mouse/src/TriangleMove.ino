#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputAnalog         adc1;           //xy=122,411
AudioSynthWaveformSine   sine1;          //xy=126,205
AudioAnalyzeRMS          rms1;           //xy=351,388
AudioOutputAnalog        dac1;           //xy=467,204
AudioConnection          patchCord1(adc1, rms1);
AudioConnection          patchCord2(sine1, dac1);
// GUItool: end automatically generated code

double amplitude = 0;
double frequency = 5.0;
int count = 0;
int numSamples = 100;
int sweepRange = 6000;

void setup() {
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(12);

  Serial.begin(9600);
  // Create a synthetic sine wave, for testing
  // To use this, edit the connections above
  sine1.amplitude(1.0);
  sine1.frequency(frequency);
}

void loop() {
  sine1.frequency(frequency);
  sine1.update();

  rms1.update();
  count += sweepRange/numSamples;
  if (count > sweepRange) {
    count = 50;
  }

  frequency = map(log10(sweepRange-count),log10(sweepRange),0,0,sweepRange);

  if (rms1.available()) {
    Serial.print(frequency);
    Serial.print(",");
    Serial.println(sweepRange*rms1.read());
  }

  if (count==1) {
    delay(300);
  }

  delay(map(frequency,0,sweepRange,300,5));
}