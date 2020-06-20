#include <Arduino.h>
#include <AccelStepper.h>
#include <string.h>
#include <stdio.h>

enum Token {
  // Command
  EnableToken,
  DisableToken,
  HomeToken,
  StopToken,
  FireToken,
  MoveToken,
  GetToken,
  SetToken,
  // Argument
  PitchToken,
  YawToken,
  LaserToken,
  GunToken,
  LightToken,
  AllToken,
  ClockWiseToken,
  CounterClockWiseToken,
  PitchSpeedToken,
  YawSpeedToken,
  PitchAccelerationToken,
  YawAccelerationToken,
  PitchPositiveLimitToken,
  PitchNegativeLimitToken,
  LightColor,
  NumberToken, // [+ -]#####[.####]
  // Protocol
  EndToken, // \n
};

enum PitchAxisState {
  Disabled,
  NotHomed,
  HomingPositive,
  HomingNegative,
  HomingMoveZero,
  Idle,
  MovingStopping,
  MovingClockWise,
  MovingCounterClockWise,
  MovingPosition
};

const char tokenSeparatorLiteral = ' ';
const char tokenEndLiteral = '\n';
const char* tokenEnableLiteral = "enable";
const char* tokenDisableLiteral = "disable";
const char* tokenHomeLiteral = "home";
const char* tokenStopLiteral = "stop";
const char* tokenFireLiteral = "fire";
const char* tokenMoveLiteral = "move";
const char* tokenGetLiteral = "get";
const char* tokenSetLiteral = "set";
const char* tokenPitchLiteral = "pitch";
const char* tokenYawLiteral = "yaw";
const char* tokenLaserLiteral = "laser";
const char* tokenGunLiteral = "gun";
const char* tokenLightLiteral = "light";
const char* tokenAllLiteral = "all";
const char* tokenClockWiseLiteral = "cw";
const char* tokenCounterClockWiseLiteral = "ccw";
const char* tokenPitchSpeedLiteral = "pitch_speed";
const char* tokenYawSpeedLiteral = "yaw_speed";
const char* tokenPitchAccelerationLiteral = "pitch_acceleration";
const char* tokenYawAccelerationLiteral = "yaw_acceleration";
const char* tokenPitchPositiveLimitLiteral = "pitch_positive_limit";
const char* tokenPitchNegativeLimitLiteral = "pitch_negative_limit";
const char* tokenLightColorLiteral = "light_color";

// pitch = turret
// constants
const int pitchAxisStepPin = 14;
const int pitchAxisDirectionPin = 13;
const int pitchAxisEnablePin = 12;
const int pitchAxisPositiveLimitSwitchPin = 16;
const int pitchAxisNegativeLimitSwitchPin = 23;
const int pitchAxisStepsPerRevolution = 200; // steps
const int pitchAxisMicroSteps = 16; 
const int pitchAxisMinPulseWidth = 2; // microseconds
const float pitchAxisGearRatio = 10.0;
const float pitchAxisStepsPerDegree = (float(pitchAxisStepsPerRevolution) * float(pitchAxisMicroSteps) * pitchAxisGearRatio) / 360.0;
const float pitchAxisHomingSpeed = 30.0 * pitchAxisStepsPerDegree; // steps/second
const float pitchAxisDefaultSpeed = 180 * pitchAxisStepsPerDegree; // steps/second
const float pitchAxisDefaultAcceleration = 360 * pitchAxisStepsPerDegree; // steps/second^2
const float pitchAxisMaximumSpeed = 720 * pitchAxisStepsPerDegree; // steps/second
const float pitchAxisMaxiumAcceleration = 2000 * pitchAxisStepsPerDegree; //steps/second^2
// variables
bool pitchAxisHomed = false;
bool pitchAxisEnabled = false;
long pitchAxisTargetPosition = 0;
int pitchAxisNegativeLimitPosition = 0; //steps
int pitchAxisPositiveLimitPosition = 0; //steps
int pitchAxisZeroPosition = 0; //steps
float pitchAxisSpeed = pitchAxisDefaultSpeed; // steps/second
float pitchAxisAcceleration = pitchAxisDefaultAcceleration; // steps/second^2
AccelStepper pitchAxisStepper;
PitchAxisState pitchAxisState;

// constants
const int yawAxisStepPin = 25;
const int yawAxisDirectionPin = 33;
const int yawAxisEnablePin = 32;
const int yawAxisStepsPerRevolution = 200; // steps
const int yawAxisMicroSteps = 16; 
const int yawAxisMinPulseWidth = 2; // microseconds
const float yawAxisGearRatio = 10.0;
const float yawAxisStepsPerDegree = (float(yawAxisStepsPerRevolution) * float(yawAxisMicroSteps) * yawAxisGearRatio) / 360.0;
const float yawAxisDefaultSpeed = 180 * yawAxisStepsPerDegree; // steps/second
const float yawAxisDefaultAcceleration = 360 * yawAxisStepsPerDegree; // steps/second^2
const float yawAxisMaximumSpeed = 720 * yawAxisStepsPerDegree; // steps/second
const float yawAxisMaxiumAcceleration = 720 * yawAxisStepsPerDegree; //steps/second^2

bool yawAxisEnabled = false;
long yawAxisTargetPosition = 0;
int yawAxisNegativeLimitPosition = 0; //steps
int yawAxisPositiveLimitPosition = 0; //steps
int yawAxisZeroPosition = 0; //steps
float yawAxisSpeed = yawAxisDefaultSpeed; // steps/second
float yawAxisAcceleration = yawAxisDefaultAcceleration; // steps/second^2
PitchAxisState yawAxisState;
AccelStepper yawAxisStepper;

int inputByte = 0;
int inputBufferCount = 0;
char inputBuffer[99];
bool inputBufferReadyForTokenization = false;
int tokenBufferCount = 0;
Token tokenBuffer[4];
float lastNumberTokenValue = 0.0;
bool tokenBufferReadyForParsing = false;

// Function declarations
void communicationTask(void *parameter);
void pitchMotorTask(void *parameter);
void yawMotorTask(void *parameter);
void readInput();
void resetInputBuffer();
void resetTokenBuffer();
void tokenizeInputBuffer();
void parseTokenBuffer();
void enablePitchAxis();
void disablePitchAxis();
void enableYawAxis();
void disableYawAxis();
void homePitchAxis();
void stopPitchAxis();
void stopYawAxis();
void stopGun();
void stopLaser();
void fireGun();
void fireLaser();
void getPitchPositiveLimit();
void getPitchNegativeLimit();
void movePitchAxisClockWise();
void movePitchAxisCounterClockWise();
void movePitchAxisToPosition(float position);
void moveYawAxisClockWise();
void moveYawAxisCounterClockWise();
void moveYawAxisToPosition(float position);

void setup() {

    pinMode(pitchAxisPositiveLimitSwitchPin, INPUT_PULLUP);
    pinMode(pitchAxisNegativeLimitSwitchPin, INPUT_PULLUP);
    pinMode(5, OUTPUT);
    pinMode(21, OUTPUT);
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);

    Serial.begin(115200);
   
    pitchAxisStepper = AccelStepper(AccelStepper::DRIVER, pitchAxisStepPin, pitchAxisDirectionPin, 0, 0, false); 
    pitchAxisStepper.setEnablePin(pitchAxisEnablePin);
    pitchAxisStepper.setMinPulseWidth(pitchAxisMinPulseWidth);
    pitchAxisStepper.setMaxSpeed(pitchAxisMaximumSpeed);
    pitchAxisStepper.setSpeed(pitchAxisSpeed);
    pitchAxisStepper.setAcceleration(pitchAxisAcceleration);
    pitchAxisStepper.setPinsInverted(false, false, true);
    pitchAxisStepper.disableOutputs();

    yawAxisStepper = AccelStepper(AccelStepper::DRIVER, yawAxisStepPin, yawAxisDirectionPin, 0, 0, false); 
    yawAxisStepper.setEnablePin(yawAxisEnablePin);
    yawAxisStepper.setMinPulseWidth(yawAxisMinPulseWidth);
    yawAxisStepper.setMaxSpeed(yawAxisMaximumSpeed);
    yawAxisStepper.setSpeed(yawAxisSpeed);
    yawAxisStepper.setAcceleration(yawAxisAcceleration);
    yawAxisStepper.setPinsInverted(false, false, true);
    yawAxisStepper.disableOutputs();


    
    xTaskCreatePinnedToCore(
      communicationTask, /* Function to implement the task */
      "CommunicationTask", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      configMAX_PRIORITIES-3,  /* Priority of the task */
      NULL,  /* Task handle. */
      0); /* Core where the task should run */

    xTaskCreatePinnedToCore(
      pitchMotorTask, /* Function to implement the task */
      "PitchMotorTask", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      configMAX_PRIORITIES-1,  /* Priority of the task */
      NULL,  /* Task handle. */
      1); /* Core where the task should run */

//    xTaskCreatePinnedToCore(
//      yawMotorTask, /* Function to implement the task */
//      "YawMotorTask", /* Name of the task */
//      10000,  /* Stack size in words */
//      NULL,  /* Task input parameter */
//      configMAX_PRIORITIES-1,  /* Priority of the task */
//      NULL,  /* Task handle. */
//      1); /* Core where the task should run */
}

void loop() {
  for(;;) {
    delay(100000);
  }
}

void communicationTask(void *parameter) {
  for (;;) {
    if (tokenBufferReadyForParsing) {
      parseTokenBuffer();
    } else if (inputBufferReadyForTokenization) {
      tokenizeInputBuffer();
    } else {
      readInput();
    }
    delay(10);
  }

}

void pitchMotorTask(void *parameter) {
  for(;;) {
    if (pitchAxisEnabled) {
      pitchAxisStepper.enableOutputs();

      if (pitchAxisState == HomingPositive) {
        if (digitalRead(pitchAxisPositiveLimitSwitchPin) == LOW) {
          pitchAxisStepper.setSpeed(pitchAxisHomingSpeed);
          pitchAxisStepper.runSpeed();
        }
        else {
          pitchAxisPositiveLimitPosition = pitchAxisStepper.currentPosition();
          pitchAxisState = HomingNegative;
        }
      } else if (pitchAxisState == HomingNegative) {
        if (digitalRead(pitchAxisNegativeLimitSwitchPin) == LOW) {
          pitchAxisStepper.setSpeed(-1.0 * pitchAxisHomingSpeed);
          pitchAxisStepper.runSpeed();
        }
        else {
          pitchAxisNegativeLimitPosition = pitchAxisStepper.currentPosition();
          pitchAxisState = HomingMoveZero;

          pitchAxisStepper.setSpeed(pitchAxisDefaultSpeed);
          pitchAxisStepper.setAcceleration(pitchAxisDefaultAcceleration);
          pitchAxisStepper.moveTo(pitchAxisNegativeLimitPosition + ((pitchAxisPositiveLimitPosition - pitchAxisNegativeLimitPosition) / 2));

        }
      } else if (pitchAxisState == HomingMoveZero) {
        pitchAxisStepper.run();

        if (!pitchAxisStepper.isRunning()) {
          pitchAxisNegativeLimitPosition -= pitchAxisStepper.currentPosition();
          pitchAxisPositiveLimitPosition -= pitchAxisStepper.currentPosition();
          pitchAxisStepper.setCurrentPosition(0);
          pitchAxisHomed = true;
          pitchAxisState = Idle;
        }
      } else if (pitchAxisHomed && digitalRead(pitchAxisPositiveLimitSwitchPin) == LOW && digitalRead(pitchAxisNegativeLimitSwitchPin) == LOW) {
        if (pitchAxisState == MovingStopping) {
        } else if (pitchAxisState == MovingClockWise) {
          pitchAxisTargetPosition = (pitchAxisPositiveLimitPosition - pitchAxisStepsPerDegree);
          pitchAxisState = MovingPosition;
          pitchAxisStepper.moveTo(pitchAxisTargetPosition);
        } else if (pitchAxisState == MovingCounterClockWise) {
          pitchAxisTargetPosition = (pitchAxisNegativeLimitPosition + pitchAxisStepsPerDegree);
          pitchAxisState = MovingPosition;
          pitchAxisStepper.moveTo(pitchAxisTargetPosition);
          
        } else if (pitchAxisState == MovingPosition) {

          if (!pitchAxisStepper.isRunning()) {
            pitchAxisState = Idle;
          }
        }
        pitchAxisStepper.run();
      }
    } else {
      pitchAxisHomed = false;
      pitchAxisStepper.disableOutputs();
    }

    if (yawAxisEnabled) {
      yawAxisStepper.enableOutputs();
  
      if (yawAxisState == MovingClockWise) {
        yawAxisTargetPosition = yawAxisStepper.currentPosition() + 360 * yawAxisStepsPerDegree;
        yawAxisState = MovingPosition;
        yawAxisStepper.moveTo(yawAxisTargetPosition);
      } else if (yawAxisState == MovingCounterClockWise) {
        yawAxisTargetPosition = yawAxisStepper.currentPosition() - 360 * yawAxisStepsPerDegree;
        yawAxisState = MovingPosition;
        yawAxisStepper.moveTo(yawAxisTargetPosition);
        
      } else if (yawAxisState == MovingPosition) {
  
        if (!yawAxisStepper.isRunning()) {
          yawAxisState = Idle;
        }
      }
      yawAxisStepper.run();
  
    } else {
      yawAxisStepper.disableOutputs();
    }
  }
}


void yawMotorTask(void *parameter) {
  for(;;) {
    delay(1);
  }
}


void readInput() {
  if (Serial.available() > 0) {
    inputByte = Serial.read();

    if (inputByte == tokenSeparatorLiteral) {
      inputBufferReadyForTokenization = true;
      return;
    } else if (inputByte == tokenEndLiteral) {
      inputBufferReadyForTokenization = true;
    }
    // add input byte to buffer;
    inputBuffer[inputBufferCount] = inputByte;
    inputBuffer[inputBufferCount + 1] = 0;
    inputBufferCount++;
  }
}

void resetInputBuffer() {
  inputBufferReadyForTokenization = false;
  inputBufferCount = 0;
  inputBuffer[0] = 0;
}

void resetTokenBuffer() {
  tokenBufferReadyForParsing = false;
  tokenBufferCount = 0;
}


void tokenizeInputBuffer() {
  bool containsEndToken = false;

  // detect end token
  if (inputBuffer[inputBufferCount - 1] == tokenEndLiteral) {
    containsEndToken = true;
    inputBufferCount--;
  }
  // parse input buffer to token
  if (strncmp(inputBuffer, tokenEnableLiteral, inputBufferCount) == 0) {
    // on
    tokenBuffer[tokenBufferCount] = EnableToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenDisableLiteral, inputBufferCount) == 0) {
    // off
    tokenBuffer[tokenBufferCount] = DisableToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenHomeLiteral, inputBufferCount) == 0) {
    // home
    tokenBuffer[tokenBufferCount] = HomeToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenStopLiteral, inputBufferCount) == 0) {
    // stop
    tokenBuffer[tokenBufferCount] = StopToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenFireLiteral, inputBufferCount) == 0) {
    // fire
    tokenBuffer[tokenBufferCount] = FireToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenMoveLiteral, inputBufferCount) == 0) {
    // move
    tokenBuffer[tokenBufferCount] = MoveToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenGetLiteral, inputBufferCount) == 0) {
    // get
    tokenBuffer[tokenBufferCount] = GetToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenSetLiteral, inputBufferCount) == 0) {
    // set
    tokenBuffer[tokenBufferCount] = SetToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenPitchLiteral, inputBufferCount) == 0) {
    // pitch
    tokenBuffer[tokenBufferCount] = PitchToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenYawLiteral, inputBufferCount) == 0) {
    // yaw
    tokenBuffer[tokenBufferCount] = YawToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenLaserLiteral, inputBufferCount) == 0) {
    // laser
    tokenBuffer[tokenBufferCount] = LaserToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenGunLiteral, inputBufferCount) == 0) {
    // gun
    tokenBuffer[tokenBufferCount] = GunToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenAllLiteral, inputBufferCount) == 0) {
    // all
    tokenBuffer[tokenBufferCount] = AllToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenClockWiseLiteral, inputBufferCount) == 0) {
    // cw
    tokenBuffer[tokenBufferCount] = ClockWiseToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenCounterClockWiseLiteral, inputBufferCount) == 0) {
    // ccw
    tokenBuffer[tokenBufferCount] = CounterClockWiseToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenPitchSpeedLiteral, inputBufferCount) == 0) {
    // pitch_speed
    tokenBuffer[tokenBufferCount] = PitchSpeedToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenYawSpeedLiteral, inputBufferCount) == 0) {
    // yaw_speed
    tokenBuffer[tokenBufferCount] = YawSpeedToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenPitchAccelerationLiteral, inputBufferCount) == 0) {
    // pitch_acceleration
    tokenBuffer[tokenBufferCount] = PitchAccelerationToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenYawAccelerationLiteral, inputBufferCount) == 0) {
    // yaw_acceleration
    tokenBuffer[tokenBufferCount] = YawAccelerationToken;
    tokenBufferCount++;
  } else if (strncmp(inputBuffer, tokenPitchPositiveLimitLiteral, inputBufferCount) == 0) {
    // pitch_positive_limit
    tokenBuffer[tokenBufferCount] = PitchPositiveLimitToken;
    tokenBufferCount++;    
  } else if (strncmp(inputBuffer, tokenPitchNegativeLimitLiteral, inputBufferCount) == 0) {
    // pitch_negative_limit
    tokenBuffer[tokenBufferCount] = PitchNegativeLimitToken;
    tokenBufferCount++;    
  } else if (sscanf(inputBuffer, "%f", &lastNumberTokenValue) == 1) {
    // NUMBER
    tokenBuffer[tokenBufferCount] = NumberToken;
    tokenBufferCount++;
  } else {
    resetInputBuffer();
    resetTokenBuffer();
    return;
  }

  resetInputBuffer();

  if (containsEndToken) {
    tokenBuffer[tokenBufferCount] = EndToken;
    tokenBufferCount++;
    tokenBufferReadyForParsing = true;
  }
}

void parseTokenBuffer () {

  if (tokenBufferCount == 3 && tokenBuffer[2] == EndToken)
  {
    if (tokenBuffer[0] == EnableToken) {
      if (tokenBuffer[1] == PitchToken) {
        // enable pitch
        enablePitchAxis();
      } else if (tokenBuffer[1] == YawToken) {
        // enable yaw
        enableYawAxis();
      }
    } else if (tokenBuffer[0] == DisableToken) {
      if (tokenBuffer[1] == PitchToken) {
        // disable pitch
        disablePitchAxis();
      } else if (tokenBuffer[1] == YawToken) {
        // disable yaw
        disableYawAxis();
      }
    } else if (tokenBuffer[0] == HomeToken) {
      if (tokenBuffer[1] == PitchToken) {
        // home pitch
        homePitchAxis();
      }
    } else if (tokenBuffer[0] == StopToken) {
      if (tokenBuffer[1] == PitchToken) {
        // stop pitch
        stopPitchAxis();
      } else if (tokenBuffer[1] == YawToken) {
        stopYawAxis();
      } else if (tokenBuffer[1] == GunToken) {
        stopGun();
      } else if (tokenBuffer[1] == LaserToken) {
        stopLaser();
      }
    } else if (tokenBuffer[0] == FireToken) {
      if (tokenBuffer[1] == GunToken) {
        fireGun();
      } else if (tokenBuffer[1] == LaserToken) {
        fireLaser();
      }
    }
    if (tokenBuffer[0] == GetToken) {
      if (tokenBuffer[1] == PitchPositiveLimitToken) {
        // get pitch_positive_limit
        getPitchPositiveLimit();
      } else if (tokenBuffer[1] == PitchNegativeLimitToken) {
        // get pitch_negative_limit
        getPitchNegativeLimit();
      }
    }
  } else if (tokenBufferCount == 4 && tokenBuffer[3] == EndToken) {
    if (tokenBuffer[0] == MoveToken) {
      if (tokenBuffer[1] == PitchToken) {
        if (tokenBuffer[2] == ClockWiseToken) {
          // move pitch cw
          movePitchAxisClockWise();
        } else if (tokenBuffer[2] == CounterClockWiseToken) {
          // move pitch ccw
          movePitchAxisCounterClockWise();
        } else if (tokenBuffer[2] == NumberToken) {
          // move pitch NUMBER
          movePitchAxisToPosition(lastNumberTokenValue);
        }
      } else if (tokenBuffer[1] == YawToken) {
        if (tokenBuffer[2] == ClockWiseToken) {
          // move yaw cw
          moveYawAxisClockWise();
        } else if (tokenBuffer[2] == CounterClockWiseToken) {
          // move yaw ccw
          moveYawAxisCounterClockWise();
        } else if (tokenBuffer[2] == NumberToken) {
          // move yaw NUMBER
          moveYawAxisToPosition(lastNumberTokenValue);
        }
      }
    }
  } 

  resetTokenBuffer();
}

void enablePitchAxis() {
  pitchAxisEnabled = true;
  Serial.print("enable pitch\n");
}

void disablePitchAxis() {
  pitchAxisEnabled = false;
  Serial.print("disable pitch\n");
}

void enableYawAxis() {
  yawAxisEnabled = true;
  Serial.print("enable yaw\n");
}

void disableYawAxis() {
  yawAxisEnabled = false;
  Serial.print("disable yaw\n");
}

void homePitchAxis() {
  pitchAxisState = HomingPositive;
}

void stopPitchAxis() {
  pitchAxisStepper.stop();
  pitchAxisState = MovingStopping;
}

void stopYawAxis() {
  yawAxisStepper.stop();
  yawAxisState = MovingStopping;
}

void stopGun() {
  digitalWrite(21, LOW);
}

void stopLaser() {

  digitalWrite(5, LOW);
}

void fireGun() {
  digitalWrite(21, HIGH);
  
}

void fireLaser() {
  digitalWrite(5, HIGH);
}

void getPitchPositiveLimit() {
  Serial.print("get pitch_positive_limit ");
  Serial.print(digitalRead(pitchAxisPositiveLimitSwitchPin));
  Serial.println();
}

void getPitchNegativeLimit() {
  Serial.print("get pitch_negative_limit ");
  Serial.print(digitalRead(pitchAxisNegativeLimitSwitchPin));
  Serial.println();
  
}

void movePitchAxisClockWise() {
    pitchAxisState = MovingClockWise;
}

void movePitchAxisCounterClockWise() {
    pitchAxisState = MovingCounterClockWise;
}


void movePitchAxisToPosition(float position) {
      pitchAxisTargetPosition = pitchAxisStepsPerDegree * position;

      long minPos = (long)(pitchAxisNegativeLimitPosition + pitchAxisStepsPerDegree);
      long maxPos = (long)(pitchAxisPositiveLimitPosition - pitchAxisStepsPerDegree);
      
      if (pitchAxisTargetPosition > maxPos) {
        pitchAxisTargetPosition = maxPos;
      }

      if (pitchAxisTargetPosition < minPos) {
        pitchAxisTargetPosition = minPos;
      }
      
      pitchAxisStepper.moveTo(pitchAxisTargetPosition);

      pitchAxisState = MovingPosition;
}

void moveYawAxisClockWise() {
    yawAxisState = MovingClockWise;
}

void moveYawAxisCounterClockWise() {
    yawAxisState = MovingCounterClockWise;
}


void moveYawAxisToPosition(float position) {
      yawAxisTargetPosition = yawAxisStepsPerDegree * position;
            
      yawAxisStepper.moveTo(yawAxisTargetPosition);

      yawAxisState = MovingPosition;
}