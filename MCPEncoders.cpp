#include <Wire.h> 
#include "Adafruit_MCP23017.h"
#include "MCPEncoders.h"


  MCPEncoders::MCPEncoders(int num_encoders, int num_buttons, int num_expanders ) 
  {
    max_buttons = num_buttons;
    buttons = new bool[num_buttons];
    buttonsChanged = new bool [num_buttons];
    lastButtonChange = new uint32_t [num_buttons];
    buttonChange = new uint32_t [num_buttons];

    max_encoders = num_encoders;
    
    knobs = new int [num_encoders];

    minvals = new int [num_encoders];
    maxvals = new int [num_encoders];
    lastSeq= new uint8_t [num_encoders];

     knobChanged = new bool[num_encoders];

     stateLPos = new uint8_t [num_encoders];
     stateRPos = new uint8_t [num_encoders];
     lastInt = new uint32_t [num_encoders];

     encmap = new int [num_encoders];
     revencmap = new int [num_encoders];
     encrevmap = new int [num_encoders];

         // Velocity data
     _period = DEFAULT_PERIOD;
     _count = new uint16_t [num_encoders];
     _spd = new uint16_t [num_encoders];
     _timeLast = new uint32_t [num_encoders];
     _dir = new uint8_t [num_encoders];

     for (int x = 0; x < max_encoders; x++) {
      encmap [x] = x;
      revencmap[encmap[x]] = x;
      encrevmap [x] = 0;
     }
     btnmap = new int [num_buttons];
     revbtnmap = new int [num_buttons];
     for (int x = 0; x < max_buttons; x++) {
      btnmap [x] = x;
      revbtnmap[btnmap[x]] = x;
     }
     if (num_expanders > 0) max_expanders = num_expanders;
     else max_expanders = ((num_buttons -1)/14) +1 + ((num_encoders-1)/7) +1;
     expanders = new Adafruit_MCP23017[max_expanders];
    
  
  }

  void MCPEncoders::setencmap(const int * nencmap)
  {
    for (int x = 0; x < max_encoders; x++)
    {
      encmap[x] = nencmap[x];
      revencmap[encmap[x]] = x;
    }
  }

  void MCPEncoders::setencrmap(const int * nencrmap)
  {
    for (int x = 0; x < max_encoders; x++)
    {
      encrevmap[x] = nencrmap[x];
    }
  }

  void MCPEncoders::setbutnmap(const int * nbtnmap)
  {
    for (int x = 0; x < max_encoders; x++)
    {
      btnmap[x] = nbtnmap[x];
      revbtnmap[btnmap[x]] = x;
    }
  }

  void MCPEncoders::begin(int maxval,  const int expander_addrs[], int interrupt_pin)
  {
    for (int x =0; x < max_buttons; x++) {
      buttons[x] = 1;
      buttonsChanged[x] = false;
      buttonChange[x] = 0;
      lastButtonChange[x] = 0;
    }
    for (int x =0; x < max_encoders; x++) {
        maxvals[x] = maxval;
        knobs[x] = maxval /2;
        knobChanged[x] = false;
        minvals[x] =  0;
        lastSeq[x] = 0;
        stateLPos[x] = 0;
        stateRPos[x] = 0;
    }
#ifdef DEBUG
    for (int x = 0; x < max_encoders; x++) {
      Serial.print(encmap[x]);
      Serial.print(" ");
    }
    Serial.println();
#endif
    for (int x = 0; x < max_encoders; x++) {
      revencmap[encmap[x]] = x;
    }
#ifdef DEBUG
    for (int x = 0; x < max_encoders; x++) {
      Serial.print(revencmap[x]);
      Serial.print(" ");
    }
    Serial.println();
    for (int x = 0; x < max_buttons; x++) {
      Serial.print(btnmap[x]);
      Serial.print(" ");
    }
    Serial.println();
#endif
    for (int x = 0; x < max_buttons; x++) {
      revbtnmap[btnmap[x]] = x;
    }
#ifdef DEBUG
    for (int x = 0; x < max_buttons; x++) {
      Serial.print(revbtnmap[x]);
      Serial.print(" ");
    }
    Serial.println();
#endif
    for (int n = 0; n < max_expanders; n++) {
        expanders[n].begin(expander_addrs[n]);
        if (interrupt_pin >= 0) expanders[n].setupInterrupts(1, 1, LOW);
        for (int x =0; x < 7; x++) {
            expanders[n].pinMode(x, INPUT);
            if (interrupt_pin >= 0) expanders[n].setupInterruptPin(x, CHANGE);
         } 
         for (int x =8; x < 15; x++) {
            expanders[n].pinMode(x, INPUT);
            if (interrupt_pin >= 0) expanders[n].setupInterruptPin(x, CHANGE);
          }
          expanders[n].readGPIOAB();
     }
    isr_pin = interrupt_pin; 
    if (interrupt_pin >= 0) {
      pinMode(interrupt_pin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(interrupt_pin), ISRgateway, FALLING);
    } 
  }

  void MCPEncoders::ISRgateway() {
    //Serial.print("X");
    if (gotInt) return;
    gotInt = true;
    inttime = millis();
  }

  void MCPEncoders::handle() {
    uint8_t seq;
    //int lastPin = expander.getLastInterruptPin();
    //int lastPin2 = expander2.getLastInterruptPin();
        //Serial.printf("pins %d %d\n", lastPin, lastPin2);
        if (isr_pin == -1) inttime = millis();   
        //if (lastPin != 255) { 
        uint16_t pinsab [max_expanders];
        uint8_t pins[max_expanders*2];
        //Serial.print("A");
        for (int x = 0; x < max_expanders; x++) {
          pinsab[x] = expanders[x].readGPIOAB(); 
          pins[2*x] = (pinsab[x] & 0x7f);
          pins [2*x + 1] = (pinsab[x] >> 8) & 0x7f;
        }
        //Serial.print("B");
        //Serial.flush();
        int btn_ofs = ((max_encoders*2) / 14) * 2;
        for (int x = 0; x < max_buttons; x++) {
              //Serial.print(buttonss>>x & 1);
              int bval;
              bval = (pins[btn_ofs + (x/7)]>>(x%7)) & 1;
              int y = revbtnmap[x];
              if (bval != buttons[y] && (inttime - lastButtonChange[y])  > 10) {
                buttonsChanged[y] = true;
                buttons[y] = bval;
                lastButtonChange[y] = buttonChange[y];
                buttonChange[y] = inttime;
#ifdef DEBUG
                Serial.print ("Btn ");
                Serial.print (y);
                Serial.print(" : ");
                Serial.println(buttons[y]);
#endif
              }
            }
        //}
        //if (lastPin2 != 255) { 
          //Serial.print("1");
  
           //uint16_t pins = expander2.readGPIOAB();
               // Serial.print("2");
  
           //expander2.readINTCAPAB();
  
         for (int x = 0; x < max_encoders; x++) {
          seq = ((pins[2*(x/7)] & 1<<(x%7)) >>(x%7)) | (((pins[2*(x/7) +1] & 1<<(x%7)) >>(x%7)) <<1);
          int y = revencmap[x];
          if (seq != lastSeq[y]) {
            //Serial.printf("enc %d : seq %d\n", x, seq);
            if (stateMachineL[stateLPos[y]] == seq) {
              stateLPos[y]++;
            } else {
              stateLPos[y] = 0;
            }
           if (stateMachineR[stateRPos[y]] == seq) {
            stateRPos[y]++;
            } else {
              stateRPos[y] = 0;
            }
            bool up = false;
            bool done = false;
            if (stateLPos[y] == 4) {
              up = false;
              done = true;
            }
            if (stateRPos[y] == 4) {
              if (done) {
                //Should never happen
                stateRPos[y] = 0;
                stateLPos[y] = 0;
                done = false;
              } else {
                up = true;
                done = true;
              }
            }
            if (done) {
              stateRPos[y] = 0;
              stateLPos[y] = 0;
              uint32_t tdif = inttime - lastInt[y];
	      _count[y]++;
              if (inttime - _timeLast[y] >= _period)
              {
                _spd[y] = (uint32_t)_count[y] * (1000L/_period);
                _timeLast[y] = inttime;
                _count[y] = 0;
              }

              uint16_t stp = 1;
              lastInt[y] = inttime;
              if (tdif < 250) {
                stp = (250000/tdif)/1000;
              }
              if (up ^ encrevmap[y]) {
                 knobs[y]+= stp;
                 _dir[y] = DIR_CW;
              } else {
                knobs[y] -= stp;
                 _dir[y] = DIR_CCW;
	      }
              if (knobs[y] > maxvals[y]) knobs[y] = maxvals[y];
              if (knobs[y] < minvals[y]) knobs[y] = minvals[y];
              knobChanged[y] = true;
#ifdef DEBUG
              Serial.print ("Enc ");
              Serial.print (y);
              Serial.print(" : ");
              Serial.println(knobs[y]);
#endif
          }
          lastSeq[y] = seq;
         }
        }  
       //expander.readINTCAPAB();
       //expander2.readINTCAPAB();
  }

  bool MCPEncoders::isButtonChanged(int b)
  {
    if (b >= 0 && b < max_buttons) return buttonsChanged[b];
    else return false;
  }

  bool MCPEncoders::isKnobChanged(int k)
  {
    if (k >= 0 && k < max_encoders) return knobChanged[k];
    else return false;
  }

  bool MCPEncoders::getButton(int b)
  {
    if (b >= 0 && b < max_buttons) {
      buttonsChanged[b] = false;
      return buttons[b];
    }
    else return false;
  }
 uint32_t MCPEncoders::getButtonTime(int b)
  {
    if (b >= 0 && b < max_buttons) {
      return buttonChange[b] - lastButtonChange[b];
    }
    else return 0;

  }

  int MCPEncoders::getKnob(int k)
  {
    if (k >= 0 && k < max_encoders) {
      knobChanged[k] = false;
      return knobs[k];
    }
    else return -1;
  }

  void MCPEncoders::setMin(int k, int val)
  {
        if (k >= 0 && k < max_encoders) {
          minvals[k] = val;
        }
  }

  void MCPEncoders::setMax(int k, int val)
  {
        if (k >= 0 && k < max_encoders) {
          maxvals[k] = val;
        }
  }

  void MCPEncoders::setKnob(int k, int val)
  {
        if (k >= 0 && k < max_encoders) {
          knobs[k] = val;
        }
  }
  uint8_t MCPEncoders::getDir(int k)
  {
     uint8_t d = _dir[k];
     _dir[k] = DIR_NONE;
     return d; 
  }
  int8_t MCPEncoders::getDirMD_R(int k)
  {
      switch(_dir[k])
      {
       case DIR_CW:
        _dir[k]=DIR_NONE;
        return(4);
       case DIR_CCW:
        _dir[k]=DIR_NONE;
        return(-4);
      }
      return(0);
  }

  SingleMCPEncoder MCPEncoders::operator[] (int idx) 
  {
		return SingleMCPEncoder(this, idx);
  }
volatile unsigned long MCPEncoders::inttime;
volatile bool MCPEncoders::gotInt;

int MCPEncoders::isr_pin;

