#if ARDUINO >= 100
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

#define START_ADDRESS 0x02

const byte DIGIT_SWITCH_OFF[24] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const byte TIME_SEMICOLON[24] = { 0, 0, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

const byte DIGIT0[10][24] = {
  { 0, 0, 120, 0, 0, 72, 0, 0, 72, 0, 0, 72, 0, 0, 72, 0, 0, 72, 0, 0, 120, 0, 0, 0 },  //0
  { 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 0 },           //1
  { 0, 0, 120, 0, 0, 8, 0, 0, 8, 0, 0, 120, 0, 0, 64, 0, 0, 64, 0, 0, 120, 0, 0, 0 },   //2
  { 0, 0, 120, 0, 0, 8, 0, 0, 8, 0, 0, 120, 0, 0, 8, 0, 0, 8, 0, 0, 120, 0, 0, 0 },     //3
  { 0, 0, 72, 0, 0, 72, 0, 0, 72, 0, 0, 120, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 0 },      //4
  { 0, 0, 120, 0, 0, 64, 0, 0, 64, 0, 0, 120, 0, 0, 8, 0, 0, 8, 0, 0, 120, 0, 0, 0 },   //5
  { 0, 0, 120, 0, 0, 64, 0, 0, 64, 0, 0, 120, 0, 0, 72, 0, 0, 72, 0, 0, 120, 0, 0, 0 }, //6
  { 0, 0, 120, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 0 },         //7
  { 0, 0, 120, 0, 0, 72, 0, 0, 72, 0, 0, 120, 0, 0, 72, 0, 0, 72, 0, 0, 120, 0, 0, 0 }, //8
  { 0, 0, 120, 0, 0, 72, 0, 0, 72, 0, 0, 120, 0, 0, 8, 0, 0, 8, 0, 0, 120, 0, 0, 0 }    //9
};

const byte DIGIT1[10][24] = {
  { 0, 15, 0, 0, 9, 0, 0, 9, 0, 0, 9, 0, 0, 9, 0, 0, 9, 0, 0, 15, 0, 0, 0, 0 },  //0
  { 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },    //1
  { 0, 15, 0, 0, 1, 0, 0, 1, 0, 0, 15, 0, 0, 8, 0, 0, 8, 0, 0, 15, 0, 0, 0, 0 }, //2
  { 0, 15, 0, 0, 1, 0, 0, 1, 0, 0, 15, 0, 0, 1, 0, 0, 1, 0, 0, 15, 0, 0, 0, 0 }, //3
  { 0, 9, 0, 0, 9, 0, 0, 9, 0, 0, 15, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },   //4
  { 0, 15, 0, 0, 8, 0, 0, 8, 0, 0, 15, 0, 0, 1, 0, 0, 1, 0, 0, 15, 0, 0, 0, 0 }, //5
  { 0, 15, 0, 0, 8, 0, 0, 8, 0, 0, 15, 0, 0, 9, 0, 0, 9, 0, 0, 15, 0, 0, 0, 0 }, //6
  { 0, 15, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },   //7
  { 0, 15, 0, 0, 9, 0, 0, 9, 0, 0, 15, 0, 0, 9, 0, 0, 9, 0, 0, 15, 0, 0, 0, 0 }, //8
  { 0, 15, 0, 0, 9, 0, 0, 9, 0, 0, 15, 0, 0, 1, 0, 0, 1, 0, 0, 15, 0, 0, 0, 0 }  //9 
};

const byte DIGIT2[10][24] = {
  { 7, 128, 0, 4, 128, 0, 4, 128, 0, 4, 128, 0, 4, 128, 0, 4, 128, 0, 7, 128, 0, 0, 0, 0 },//0
  { 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 0, 0 },//1
  { 7, 128, 0, 0, 128, 0, 0, 128, 0, 7, 128, 0, 4, 0, 0, 4, 0, 0, 7, 128, 0, 0, 0, 0 },    //2
  { 7, 128, 0, 0, 128, 0, 0, 128, 0, 7, 128, 0, 0, 128, 0, 0, 128, 0, 7, 128, 0, 0, 0, 0 },//3
  { 4, 128, 0, 4, 128, 0, 4, 128, 0, 7, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 0, 0 },//4
  { 7, 128, 0, 4, 0, 0, 4, 0, 0, 7, 128, 0, 0, 128, 0, 0, 128, 0, 7, 128, 0, 0, 0, 0 },    //5
  {7, 128, 0, 4, 0, 0, 4, 0, 0, 7, 128, 0, 4, 128, 0, 4, 128, 0, 7, 128, 0, 0, 0, 0 },     //6
  { 7, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 128, 0, 0, 0, 0 },//7
  { 7, 128, 0, 4, 128, 0, 4, 128, 0, 7, 128, 0, 4, 128, 0, 4, 128, 0, 7, 128, 0, 0, 0, 0 },//8
  { 7, 128, 0, 4, 128, 0, 4, 128, 0, 7, 128, 0, 0, 128, 0, 0, 128, 0, 7, 128, 0, 0, 0, 0 } //9
};

const byte DIGIT3[10][24] = {
  { 240, 0, 0, 144, 0, 0, 144, 0, 0, 144, 0, 0, 144, 0, 0, 144, 0, 0, 240, 0, 0, 0, 0, 0}, //0
  { 16, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 0, 0, 0 },       //1
  { 240, 0, 0, 16, 0, 0, 16, 0, 0, 240, 0, 0, 128, 0, 0, 128, 0, 0, 240, 0, 0, 0, 0, 0 },  //2
  { 240, 0, 0, 16, 0, 0, 16, 0, 0, 240, 0, 0, 16, 0, 0, 16, 0, 0, 240, 0, 0, 0, 0, 0 },    //3
  { 144, 0, 0, 144, 0, 0, 144, 0, 0, 240, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 0, 0, 0 },   //4
  { 240, 0, 0, 128, 0, 0, 128, 0, 0, 240, 0, 0, 16, 0, 0, 16, 0, 0, 240, 0, 0, 0, 0, 0 },  //5
  { 240, 0, 0, 128, 0, 0, 128, 0, 0, 240, 0, 0, 144, 0, 0, 144, 0, 0, 240, 0, 0, 0, 0, 0 },//6
  { 240, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 16, 0, 0, 0, 0, 0 },      //7
  { 240, 0, 0, 144, 0, 0, 144, 0, 0, 240, 0, 0, 144, 0, 0, 144, 0, 0, 240, 0, 0, 0, 0, 0}, //8
  { 240, 0, 0, 144, 0, 0, 144, 0, 0, 240, 0, 0, 16, 0, 0, 16, 0, 0, 240, 0, 0, 0, 0, 0 }   //9
};  

class SunFounderEmo {
    private :
        byte ledData[24];
        
        void setData(byte data[24]);
        
       void sendByte(byte data);

        byte CS_PIN;

    public:
        /* 
         * Create a new controler 
         * Params :
         * csPin     pin for the chip select
         */
        SunFounderEmo(byte csPin);

        /* 
         * Set all Leds on the display off. 
         */
        void resetDisplay();
        
        /* 
         * Switch all Leds on the display off. 
         */
        void clearDisplay();

        /* 
         * Display a hexadecimal digit on a Dot Matrix Display
         * Params:
         * digit  the position of the digit on the display (0..4)
         * value  the value to be displayed. (0...9)
         */
        void setDigit(byte digit, byte value);

        /* 
         * Switch on/off semicolon on a Dot Matrix Display
         * Params:
         * isShown state
         */
        void showTimeTick(boolean isShown);

        /* 
         * Refreshes data on a Dot Matrix Display
         */
        void updateDisplay();
};
