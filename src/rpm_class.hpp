#include <Arduino.h>
class ShaftFrequency {
 private:
  double currentFrequency;  // Frequency in Hz
  double decayRate;
  unsigned long lastPulseTimeMicros;  // Time when the last pulse was received
                                      // in microseconds
  const double maxFrequency;          // Maximum frequency
unsigned long timeIntervalMicros;
 public:
  ShaftFrequency(double initialFrequency, double decayRate)
      : currentFrequency(initialFrequency),
        decayRate(decayRate),
        lastPulseTimeMicros(micros()),
        maxFrequency(initialFrequency) {}

  // Calculate the difference in time, considering the rollover
  unsigned long timeDifferenceMicros(unsigned long currentTimeMicros,
                                     unsigned long previousTimeMicros) {
    if (currentTimeMicros >= previousTimeMicros) {
      return currentTimeMicros - previousTimeMicros;
    } else {
      // Handle rollover
      Serial.println(">>>>>>>>>>>>>>>>>>>>micros() rolled over");
      return (ULONG_MAX - previousTimeMicros) + currentTimeMicros + 1;
     }
  }

  // Update the frequency based on the current time in microseconds
  void update(unsigned long currentTimeMicros) {

        double timeSinceLastPulseMicros = timeDifferenceMicros(currentTimeMicros, lastPulseTimeMicros);
        double expectedTimeBetweenPulsesMicros = timeIntervalMicros;

        // If time since last pulse is more than 105% of the expected time, start faster decay
        if (timeSinceLastPulseMicros > 1.05 * expectedTimeBetweenPulsesMicros) {
            currentFrequency *= (1.0 - decayRate);  // Apply decay multiplier each update
            if (currentFrequency < 0.1) currentFrequency = 0;
        }
  }

  // Call this when a pulse is received
  void pulseReceived(unsigned long currentTimeMicros) {
    timeIntervalMicros = currentTimeMicros - lastPulseTimeMicros;

    // Avoid division by zero if the pulseReceived is called twice in a row very
    // quickly
    if (timeIntervalMicros > 0) {
      currentFrequency = 1000000.0 / timeIntervalMicros;
    }
    lastPulseTimeMicros = currentTimeMicros;
  }

  double getCurrentFrequency() const { return currentFrequency; }
};
