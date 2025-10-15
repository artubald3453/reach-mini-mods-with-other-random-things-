#include "I2Cdev.h"

#include "Wire.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_EULER
#define INTERRUPT_PIN 2


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


//Additional Variable Definitions (Some may not be used)
float x, y, z, xg, yg, zg, aangle, angle, speed2r, speed2l, delTr, delTl, err, offset2, P, I, D, Command, lastSensorTimer, lastSensorTimel, lasttime, lasttime2, dt, dt2, serror, deriv, err1, err2, err3, pastderiv, angle2, gx, ay, az;
float sperr, P2, I2, D2, ssperr, deriv2, turnr = 0, turnl = 0, mspd = 0, eangle;
float tspd = 10;
const int stepPin1 = 6;
const int dirPin1 = 5;
const int stepPin2 = 8;
const int dirPin2 = 7;
const int MS1R = 9;
const int MS2R = 10;
const int MS1L = 20;
const int MS2L = 21;
const int ledPin = 13;
int pr = 1;
int pl = 1;
int sign = 1;

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
String cmdspd, cmdangle, input;
int spdi, angi;
boolean newData = false;
boolean spd = false;
boolean state;


void setup() {
  Serial3.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(MS1R, OUTPUT);
  pinMode(MS2R, OUTPUT);
  pinMode(MS1L, OUTPUT);
  pinMode(MS2L, OUTPUT);
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);
  speed2r = 0;
  speed2l = 0;

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(1000000); // 1 MHz I2C clock.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(1000, true);
#endif

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own offsets here, scaled for min sensitivity    (MUST DO IMU CALIBRATION, USE IMU ZERO EXAMPLE FROM I2CDevLib)
  mpu.setXGyroOffset(3);
  mpu.setYGyroOffset(-18);
  mpu.setZGyroOffset(-52);
  mpu.setXAccelOffset(178);
  mpu.setYAccelOffset(499);
  mpu.setZAccelOffset(1598); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));


  }



}

void loop() {
  recvWithStartEndMarkers();
  showNewData(); //cmdspd, cmdangle, state
  if (state) {




    offset2 = -93.9 + aadj; //-99.8 w/o front cover on

    //PID Controller Balance (May need to adjust values for optimum performance)
    P = -0.0043;
    I = -0.000002;
    D = -0.0016;

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);

      //Serial.print("ypr\t");
      //Serial.print(ypr[0] * 180/M_PI);
      //Serial.print("\t");
      //Serial.print(ypr[1] * 180/M_PI);
      //Serial.print("\t");
      //Serial.println(ypr[2] * 180/M_PI);
#endif

    }


    angle2 = 0.96 * (angle2 + (gy.x * dt / 1310000000) * 180 / M_PI) + (0.04 * angle);

    err = angle2 + offset2;

    if (dt > 100) {

      if (cmdangle.toInt() == 180) {
        turnr = 0.0000;
        turnl = 0.0000;
      } else {

        if (cmdangle.toInt() == 0) {
          turnr = 0.000000;
          turnl = 0.000000;
        }
        else if (cmdangle.toInt() < 180) {
          turnr = ((cmdangle.toInt() - 90.00) * (tspd / 5));
          turnl = -((cmdangle.toInt() - 90.00) * (tspd / 5));
        } else {
          turnr = ((cmdangle.toInt() - 270.00) * (tspd / 5));
          turnl = -((cmdangle.toInt() - 270.00) * (tspd / 5));
        }
      }

      //Speed (Angle) Control PID values (Can modify to control the robot differently if desired)
      P2 = 0.003;
      I2 = 0.000;
      D2 = 0.000;
      mspd = sin(cmdangle.toInt() * 3.1415 / 180);
      sperr = (((cmdspd.toInt()) * mspd) * sign);
      ssperr = ssperr + sperr;
      aadj = (P2 * sperr) + (I2 * ssperr) + (D2 * deriv2);
      setMyVar((aadj / (180 / M_PI))); // (COMMENT OUT IF NOT MODIFYING THE .h FILE OF THE MPU6050 LIBRARY.)

      err3 = err2;
      err2 = err1;
      err1 = sperr;
      deriv2 = ((3 * err1) - (4 * err2) + (err3));




      angle = ypr[2] * 180 / M_PI;
      eangle = -(euler[2] * 180 / M_PI);


      serror = serror + (err);
      dt2 = micros() - lasttime2;
      lasttime2 = micros();

    }



    deriv = gy.x;


    Command = (P * err) + (I * serror) + (D * deriv);


    if (Command > 2) {
      Command = 2;
    }
    else if (Command < -2) {
      Command = -2;
    }

    accel(Command);


    dt = micros() - lasttime;
    lasttime = micros();




  }

}

void velocity(float speed2r, float speed2l) {
  if ((speed2r + turnr) < 0.0) {
    digitalWrite(dirPin1, HIGH);

  }
  else {
    digitalWrite(dirPin1, LOW);

  }

  if ((speed2l + turnl) < 0.0) {

    digitalWrite(dirPin2, LOW);
  }
  else {

    digitalWrite(dirPin2, HIGH);
  }

  if ((abs(speed2r + turnr)) < 250) {
    delTr = 100000.0 / (abs(speed2r + turnr));
    digitalWrite(MS1R, HIGH);
    digitalWrite(MS2R, HIGH);
  } else if ((abs(speed2r + turnr)) < 500) {
    delTr = 200000.0 / (abs(speed2r + turnr));
    digitalWrite(MS1R, LOW);
    digitalWrite(MS2R, HIGH);
  } else if ((abs(speed2r + turnr)) < 1000) {
    delTr = 400000.0 / (abs(speed2r + turnr));
    digitalWrite(MS1R, HIGH);
    digitalWrite(MS2R, LOW);
  } else if ((abs(speed2r + turnr)) < 1800) {
    delTr = 800000.0 / (abs(speed2r + turnr));
    digitalWrite(MS1R, LOW);
    digitalWrite(MS2R, LOW);
  } else if ((abs(speed2r + turnr)) > 1800) {
    delTr = 445;
    digitalWrite(MS1R, LOW);
    digitalWrite(MS2R, LOW);
  }

  if ((abs(speed2l + turnl)) < 250) {
    delTl = 100000.0 / (abs(speed2l + turnl));
    digitalWrite(MS1L, HIGH);
    digitalWrite(MS2L, HIGH);
  } else if ((abs(speed2l + turnl)) < 500) {
    delTl = 200000.0 / (abs(speed2l + turnl));
    digitalWrite(MS1L, LOW);
    digitalWrite(MS2L, HIGH);
  } else if ((abs(speed2l + turnl)) < 1000) {
    delTl = 400000.0 / (abs(speed2l + turnl));
    digitalWrite(MS1L, HIGH);
    digitalWrite(MS2L, LOW);
  } else if ((abs(speed2l + turnl)) < 1800) {
    delTl = 800000.0 / (abs(speed2l + turnl));
    digitalWrite(MS1L, LOW);
    digitalWrite(MS2L, LOW);
  } else if ((abs(speed2l + turnl)) > 1800) {
    delTl = 445;
    digitalWrite(MS1L, LOW);
    digitalWrite(MS2L, LOW);
  }

  if (micros() - lastSensorTimer >= delTr) {
    lastSensorTimer += delTr;


    if (pr == 2) {
      digitalWrite(stepPin1, HIGH);

      pr = 1;
    }
    else if (pr == 1) {
      digitalWrite(stepPin1, LOW);

      pr = 2;
    }
  }
  //Serial.println(delTr);

  if (micros() - lastSensorTimel >= delTl) {
    lastSensorTimel += delTl;


    if (pl == 2) {
      digitalWrite(stepPin2, HIGH);
      pl = 1;
    }
    else if (pl == 1) {

      digitalWrite(stepPin2, LOW);
      pl = 2;
    }
  }
}

void accel(float force) {
  speed2r = speed2r + force;
  speed2l = speed2l + force;
  if (speed2r > 1800) {
    speed2r = 1800;
  }
  else if (speed2r < -1800) {
    speed2r = -1800;
  }
  if (speed2l > 1800) {
    speed2l = 1800;
  }
  else if (speed2l < -1800) {
    speed2l = -1800;
  }
  velocity(speed2r, speed2l);
  //Serial.println(speed2);
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial3.available() > 0 && newData == false) {
    rc = Serial3.read();


    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        input += rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;

      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    //Serial.print("This just in ... ");

    cmdangle = "";
    cmdspd = "";
    if (input == "On") {
      state = true;
    } else if (input == "Off") {
      state = false;
    } else {
      for (int d = 0; d < sizeof(receivedChars); d++) {
        if (receivedChars[d] == 'R') {
          spd = true;
          continue;
        } else if (receivedChars[d] == 'A') {
          spd = false;
          continue;
        }
        if (spd == true) {
          cmdspd = cmdspd + receivedChars[d];
        } else {
          cmdangle = cmdangle + receivedChars[d];
        }
      }
      //Serial.println(cmdspd);
      //Serial.println(cmdangle);

    }
    //Serial.println(state);
    input = "";
    newData = false;
  }
}
