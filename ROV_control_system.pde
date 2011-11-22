#define DEBUG


#include <Wire.h>

#ifndef WiiClassic_h
#define WiiClassic_h

/* all the code for the wii Controller, should be fine, leave it alone */

class WiiClassic {    
    private:
        byte cnt;
        uint8_t status[4];		// array to store wiichuck output
        byte averageCounter;
        //int accelArray[3][AVERAGE_N];  // X,Y,Z

        uint8_t buttons[2];
        uint8_t lastButtons[2];


    public:

        void begin() 
        {
            Wire.begin();
            cnt = 0;
            averageCounter = 0;
            Wire.beginTransmission (0x52);	// transmit to device 0x52
            Wire.send (0x40);		// sends memory address
            Wire.send (0x00);		// sends memory address
            Wire.endTransmission ();	// stop transmitting
            lastButtons[0] = 0xff;
            lastButtons[1] = 0xff;
            buttons[0] = 0xff;
            buttons[1] = 0xff;

            update();         

        }


        void update() {

            Wire.requestFrom (0x52, 6);	// request data from nunchuck
            while (Wire.available ()) {
                // receive byte as an integer
                if (cnt < 4) {
                    status[cnt] = _nunchuk_decode_byte (Wire.receive()); //_nunchuk_decode_byte () 
                } else {
                    lastButtons[cnt-4] = buttons[cnt-4];
                    buttons[cnt-4] =_nunchuk_decode_byte (Wire.receive());
                }
                cnt++;
            }
            if (cnt > 5) {
                _send_zero(); // send the request for next bytes
                cnt = 0;                   
            }
        }

        byte * getRawStatus() {
            return status;
        }

        byte * getRawButtons() {
            return buttons;
        }

        boolean leftShoulderPressed() {
            return _PressedRowBit(0,5);
        }

        boolean rightShoulderPressed() {
            return _PressedRowBit(0,1);
        }

        boolean lzPressed() {
            return _PressedRowBit(1,7);
        }

        boolean rzPressed() {
            return _PressedRowBit(1,2);
        }

        boolean leftDPressed() {
            return _PressedRowBit(1,1);
        }

        boolean rightDPressed() {
            return _PressedRowBit(0,7);
        }

        boolean upDPressed() {
            return _PressedRowBit(1,0);
        }

        boolean downDPressed() {
            return _PressedRowBit(0,6);
        }

        boolean selectPressed() {
            return _PressedRowBit(0,4);
        }

        boolean homePressed() {
            return _PressedRowBit(0,3);
        }

        boolean startPressed() {
            return _PressedRowBit(0,2);
        }

        boolean xPressed() {
            return _PressedRowBit(1,3);
        }

        boolean yPressed() {
            return _PressedRowBit(1,5);
        }

        boolean aPressed() {
            return _PressedRowBit(1,4);
        }

        boolean bPressed() {
            return _PressedRowBit(1,6);
        }

        int rightShouldPressure() {
            return status[3] & 0x1f; //rightmost 5 bits
        }

        int leftShouldPressure() {
            return ((status[2] & 0x60) >> 2) + ((status[3] & 0xe0) >> 5); //rightmost 5 bits
        }

        int leftStickX() {
            return  ( (status[0] & 0x3f) );
        }

        int leftStickY() {
            return  ((status[1] & 0x3f));       
        }

        int rightStickX() {
            return ((status[0] & 0xc0) >> 3) + ((status[1] & 0xc0) >> 5) +  ((status[2] & 0x80) >> 7);

        }

        int rightStickY() {
            return status[2] & 0x1f;  	
        }


    private:
        boolean _PressedRowBit(byte row, byte bit) {
            byte mask = (1 << bit);
            return (!(buttons[row] & mask ));// && (lastButtons[row] & mask);
        }


        byte _nunchuk_decode_byte (byte x)
        {
            x = (x ^ 0x17) + 0x17;
            return x;
        }

        void _send_zero()
        {
            Wire.beginTransmission (0x52);	// transmit to device 0x52
            Wire.send (0x00);		// sends one byte
            Wire.endTransmission ();	// stop transmitting
        }

};


#endif



class Motor {
public:
  Motor () { 
    time=0;
    for(byte i=0;i<6;++i) {
      speeds[i] = 127;
    }
  }
  void operator () (){
    int PWM_delay = time - micros();
    if(PWM_delay > 5)
      delayMicroseconds(PWM_delay);
    // The value on the next line I might tell you to change
    time = micros() + 17000; // 16 ms (min is 15 ms)
    for(byte i=0;i<6;++i) {
      int length = speeds[i] * 4 + 1000;
      if(length > 2000) length=2000;
      if(length < 1000) length=1000;
      digitalWrite(i+2, HIGH);
      delayMicroseconds(length);
      digitalWrite(i+2, LOW);
    }
#ifdef DEBUG
    Serial.print(PWM_delay);
    Serial.print("\t");
#endif
  }
  byte &  operator [] (unsigned char num) {
    if(num < 6)
      return speeds[num];
  }
  void print() {
    for(byte i=0; i<6; ++i) {
      Serial.print((int)speeds[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
private:
  byte speeds[6];
  unsigned long time;
  
};

inline int dead (int in) {
  // both numbers should be the same except that one is neg
  // the goal here is to make values that are to small = 0
  if(in < 18 && in > -18)
    return 0;
  return in;
}

void convert (int & a, int & b) {
  // makes the values of outputs be in range but keeps the ratio between two values the same
  // this is used for when going foward and turning, it will slow down the other motor when one is all ready at max speed
  if(a > 127 || a < -127 || b > 127 || b < -127) {
    if(abs(a) > abs(b)) {
      b = map(b, -abs(a), abs(a), -127, 127);
      a = map(a, -abs(a), abs(a), -127, 127);
    }else{
      a = map(a, -abs(b), abs(b), -127, 127);
      b = map(b, -abs(b), abs(b), -127, 127);
    }
  }
}

Motor motor = Motor();
WiiClassic controller = WiiClassic();

struct powers {
  int left, right;
  int side;
  int front, back;
  int intake;  
} power;

unsigned char intakeToggle=0;
char pitch=0;


void setup () {
  Wire.begin();
#ifdef DEBUG
  Serial.begin(57600);
#endif
  controller.begin();
  controller.update();
  
  for(byte i=2; i < 8; ++i) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
}

void loop () {
#ifdef DEBUG
  motor.print();
#endif
  motor();
  controller.update();
  power.left = power.right = dead(map(controller.leftStickY(), 2, 58, -127, 127));
  power.side = dead(map(controller.leftStickX(), 2, 58, -127, 127));
  
  if(controller.leftShoulderPressed() || controller.rightShoulderPressed() || controller.rzPressed())
    pitch=0;
  if(controller.upDPressed() && pitch > -126)
    --pitch;
  else if(controller.downDPressed() && pitch < 126)
    ++pitch;

  power.front = power.back = controller.rightShoulderPressed() ? 127 : ( controller.rzPressed() ? -127 : pitch );
  
  if(controller.yPressed() && !(intakeToggle & 0x02)) {
    intakeToggle ^= 0x03;
  }else if(!controller.yPressed()) {
    intakeToggle &= ~0x02;
  }
  
  power.intake = intakeToggle & 0x01 ? -127 : 0;
  

  if(controller.bPressed()) {
    power.intake = -127;
    intakeToggle &= ~0x01;
  }else if(controller.aPressed()) {
    power.intake = 127;
    intakeToggle &= ~0x01;  
  }
  
  int turn = dead(map(controller.rightStickX(), 2, 30, -127, 127));
  power.right += turn;
  power.left -= turn;
  convert(power.right, power.left);
  
  int tip = dead(map(controller.rightStickY(), 2, 30, -127, 127));
  power.front += tip;
  power.back -= tip;
  convert(power.front, power.back);
  
  
  motor[0] = power.left + 127;
  motor[1] = power.right + 127;
  motor[2] = power.side + 127;
  motor[3] = power.front + 127;
  motor[4] = power.back + 127;
  motor[5] = power.intake + 127;
}


