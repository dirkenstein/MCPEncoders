#include <Wire.h> 
#include "Adafruit_MCP23017.h"
#pragma once

#define DEBUG 1


//  Direction values returned by read() method 
/**
 \def DIR_NONE
  read() return value - No complete step/movement
 */
#define DIR_NONE  0x00
/**
 \def DIR_CW
 read() return value - Clockwise step/movement
 */
#define DIR_CW    0x10
/**
 \def DIR_CCW
 read() return value - Counter-clockwise step/movement
 */
#define DIR_CCW   0x20  


class SingleMCPEncoder;

class MCPEncoders 
{
  public:
  MCPEncoders(int num_encoders, int num_buttons, int num_expanders=-1);
  void setencmap(const int * nencmap);
  void setencrmap(const int * nencrmap);
  void setbutnmap(const int * nbtnmap);
  void begin(int maxval,  const int expander_addrs[], int interrupt_pin=-1);
  static void ISRgateway();
  void handle();
  bool isButtonChanged(int b);
  bool isKnobChanged(int k);
  bool getButton(int b);
  uint32_t getButtonTime(int b);
  int getKnob(int k);
  void setMin(int k, int val);
  void setMax(int k, int val);
  void setKnob(int k, int val);
  uint8_t getDir(int k);
  int8_t getDirMD_R(int k);

  SingleMCPEncoder operator[] (int idx);
  inline void setPeriod(uint16_t t) { if ((t != 0) && (t <= 1000)) _period = t; };
  inline uint16_t getSpeed(int k)
  {
        if(_spd[k]==0)
          return(1);
        if(_spd[k]>10)
          return(10/1.5+0.5);
        
        return(float(_spd[k])/1.5+0.5);
   };



  private:
    Adafruit_MCP23017 * expanders;

    int max_encoders;
    int max_buttons;
    int max_expanders = 3;

    static int isr_pin;
    static volatile bool gotInt;
    static volatile unsigned long inttime;

    volatile bool * buttons;
    volatile bool * buttonsChanged;
    uint32_t * lastButtonChange;
    uint32_t * buttonChange;

    volatile int * knobs;

    int * minvals;
    int * maxvals;
    uint8_t * lastSeq;
  
    volatile bool * knobChanged;

    uint8_t * stateLPos;
    uint8_t * stateRPos;
    uint32_t * lastInt; 

    int * encmap;
    int * revencmap;
    int * encrevmap;

    int * btnmap;
    int * revbtnmap;
    // Velocity data
    uint16_t  _period;  // velocity calculation period
    uint16_t  * _count;   // running count of encoder clicks
    uint16_t  * _spd;     // last calculated speed (no sign) in clicks/second
    uint32_t  * _timeLast;  // last time read
    uint8_t * _dir; //Direction

     
    const uint8_t stateMachineR [4] = {2, 0, 1, 3};
    const uint8_t stateMachineL [4] = {1, 0, 2, 3};
    const uint16_t DEFAULT_PERIOD = 500;
    friend class SingleMCPEncoder; 
};


class SingleMCPEncoder
{
	public:
	SingleMCPEncoder(MCPEncoders * parent, int idx = 0) : _idx(idx), _parent(parent) { };
	int read() { return _parent->getDirMD_R(_idx);};
	int read_value() { return _parent->getKnob(_idx);};
	void write (int val) { _parent->setKnob(_idx, val);};
	void min(int val) { _parent->setMin(_idx, val);};
	void max(int val) { _parent->setMax(_idx, val);};
	int speed() { return _parent->getSpeed(_idx);};
	bool button() { return _parent->getButton(_idx);};
	bool isButtonChanged() { return _parent->isButtonChanged(_idx);};
	bool isKnobChanged(){ return _parent->isKnobChanged(_idx);};
	uint32_t getButtonTime() {return _parent->getButtonTime(_idx);};
	private:
		int _idx = 0;
		MCPEncoders * _parent;
};
