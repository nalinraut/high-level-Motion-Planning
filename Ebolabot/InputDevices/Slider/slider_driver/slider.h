/* 
 Required structures and methods for simple interface with sliders.
 */

// Motor parameters
#define DEAD_ZONE 2
#define MIN_PWM 60

// PID configs
#define kp 18
#define ki .001
#define kd 0
#define kScale .1
#define PID_MS 50
#define PID_OUT_MAG 300

// Touch parameters
#define TOUCH_SAMPLES 2
#define TOUCH_DETECT TOUCH_SAMPLES*10

// Control variables
boolean forceFollow;

struct slider {
  uint8_t potPin;
  uint8_t mot_spd;
  uint8_t mot_dir;
  uint8_t touchPin;
  PID *pid;
  double currPos;
  double pid_out;
  double target;
  int pwm;
  boolean touched;
  boolean wasTouched;
  boolean waiting;
  double releasedPos;
  uint8_t touch_count;
};
typedef struct slider Slider;

void updateState(Slider *sldr) {
  sldr->currPos = analogRead(sldr->potPin);
  sldr->touched = touchRead(sldr->touchPin) > TOUCH_DETECT;
}

void enableSlider(Slider *sldr) {
  pinMode(sldr->mot_spd, OUTPUT);
  pinMode(sldr->mot_dir, OUTPUT);
}

void forward(Slider *sldr) {
  analogWrite(sldr->mot_spd, sldr->pwm);
  digitalWrite(sldr->mot_dir, LOW);
}

void reverse(Slider *sldr) {
  analogWrite(sldr->mot_spd, sldr->pwm);
  digitalWrite(sldr->mot_dir, HIGH);
}

void halt(Slider *sldr) {
  digitalWrite(sldr->mot_spd, LOW);
  digitalWrite(sldr->mot_dir, LOW);
}

// For some reason, even though map() is a built in function, the code will not compile. A copy of the function is here.
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PIDmove(Slider *sldr) {
  if (abs(sldr->target - sldr->currPos) > DEAD_ZONE) {
    if (sldr->pid_out > 0) {
      sldr->pwm = map(sldr->pid_out, 0, PID_OUT_MAG, MIN_PWM, 255);
      forward(sldr);
    }
    else {
      sldr->pwm = map(sldr->pid_out, 0, -PID_OUT_MAG, MIN_PWM, 255);
      reverse(sldr);
    }
  }
  else {
    sldr->pwm = 0;
    halt(sldr);
  }
}

void stayUntilClose(Slider *sldr) {
  if (abs(sldr->target - sldr->currPos) < DEAD_ZONE) {
    sldr->waiting = false;
    PIDmove(sldr);
  }
  else { // still waiting
    sldr->pwm = 0;
    halt(sldr);
  }
}

void slideToTarget(Slider *sldr) {
  sldr->pid->Compute();

  if (sldr->touched) {
    sldr->pwm = 0;
    halt(sldr);
    sldr->wasTouched = true;
  }
  else { // not touched
    if (forceFollow) {
      sldr->waiting = false;
      PIDmove(sldr);
    }
    else { // no force follow override
      if (sldr->wasTouched) {
        sldr->wasTouched = false;
        sldr->waiting = true;
        sldr->releasedPos = sldr->currPos;
        stayUntilClose(sldr);
      }
      else{ // not was touced
        if (sldr->waiting) {
          stayUntilClose(sldr);
        }
        else { // not waiting
          PIDmove(sldr);
        }
      }
    }  
  }
}





