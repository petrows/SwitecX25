#ifndef SwitecX12_h
#define SwitecX12_h
#include <Arduino.h>

class SwitecX12 {
  public:
    typedef struct
    {
      unsigned short steps;
      unsigned short time; // in microseconds
    } AccelTable;
    unsigned char pinStep;
    unsigned char pinDir;
    unsigned int currentStep;      // step we are currently at
    unsigned int targetStep;       // target we are moving to
    unsigned int steps;            // total steps available
    unsigned long time0;           // time when we entered this state
    unsigned int microDelay;       // microsecs until next state
    AccelTable *accelTable;        // accel table can be modified.
    unsigned char accelTableSize;  // How many rows in the acceleration table
    unsigned int maxVel;           // fastest vel allowed
    unsigned int vel;              // steps travelled under acceleration
    int8_t dir;                    // direction -1,0,1
    int8_t dirPrevious;            // Last direction set to pinDir
    boolean stopped;               // true if stopped
    SwitecX12(unsigned int steps, unsigned char pinStep, unsigned char pinDir);

    //void stepUp();
    void step(int dir);
    void zero();
    void stepTo(int position);
    void advance();
    void update();
    void setCurrentPosition(unsigned int pos);
    void setPosition(unsigned int pos);
    template <typename T, size_t N>
    void setAccelTable(T (&table)[N])
    {
      accelTable = table;
      accelTableSize = N;
      maxVel = accelTable[accelTableSize - 1].steps;
    };
    short getAccel(int vel);
};

#endif
