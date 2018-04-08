/* Simple stucture and functions for keeping track of time elapsed on events. Can be used to replace delays with non-blocking
functionality */

#include <Arduino.h>

struct timer {
  unsigned long previousMillis;
  long maxMS;
};
typedef struct timer NonBlockTimer;

void update(NonBlockTimer *tmr) {
  // Updates previousMillis to current time.
  tmr->previousMillis = millis();
}

boolean hasElapsed(NonBlockTimer *tmr) {
  // True if it has been maxMS milliseconds since last update.
  return (millis() - tmr->previousMillis) > tmr->maxMS;
}

int msSinceUpdate (NonBlockTimer *tmr) {
  return millis() - tmr->previousMillis;
}
