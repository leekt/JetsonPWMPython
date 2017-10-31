#ifndef _JHPWMPCA9685_H
#define _JHPWMPCA9685_H

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

class PCA9685
{
public:
    unsigned char kI2CBus ;         // I2C bus of the PCA9685
    int kI2CFileDescriptor ;        // File Descriptor to the PCA9685
    int kI2CAddress ;               // Address of PCA9685; defaults to 0x40
    int error ;
    PCA9685(int address=0x40);
    ~PCA9685() ;
    bool openPCA9685() ;
    void closePCA9685();

    void reset() ;

    // Sets the frequency of the PWM signal
    // Frequency is ranged between 40 and 1000 Hertz
    void setPWMFrequency ( float frequency );

    // Channels 0-15
    // Channels are in sets of 4 bytes
    void setPWM ( int channel, int onValue, int offValue);

    void setAllPWM (int onValue, int offValue);

    // Read the given register
    int readByte(int readRegister);

    // Write the the given value to the given register
    int writeByte(int writeRegister, int writeValue);

    int getError() ;

};


// Register definitions from Table 7.3 NXP Semiconductors
// Product Data Sheet, Rev. 4 - 16 April 2015
#define PCA9685_MODE1            0x00
#define PCA9685_MODE2            0x01
#define PCA9685_SUBADR1          0x02
#define PCA9685_SUBADR2          0x03
#define PCA9685_SUBADR3          0x04
#define PCA9685_ALLCALLADR       0x05
// LED outbut and brightness
#define PCA9685_LED0_ON_L        0x06
#define PCA9685_LED0_ON_H        0x07
#define PCA9685_LED0_OFF_L       0x08
#define PCA9685_LED0_OFF_H       0x09

#define PCA9685_LED1_ON_L        0x0A
#define PCA9685_LED1_ON_H        0x0B
#define PCA9685_LED1_OFF_L       0x0C
#define PCA9685_LED1_OFF_H       0x0D

#define PCA9685_LED2_ON_L        0x0E
#define PCA9685_LED2_ON_H        0x0F
#define PCA9685_LED2_OFF_L       0x10
#define PCA9685_LED2_OFF_H       0x11

#define PCA9685_LED3_ON_L        0x12
#define PCA9685_LED3_ON_H        0x13
#define PCA9685_LED3_OFF_L       0x14
#define PCA9685_LED3_OFF_H       0x15

#define PCA9685_LED4_ON_L        0x16
#define PCA9685_LED4_ON_H        0x17
#define PCA9685_LED4_OFF_L       0x18
#define PCA9685_LED4_OFF_H       0x19

#define PCA9685_LED5_ON_L        0x1A
#define PCA9685_LED5_ON_H        0x1B
#define PCA9685_LED5_OFF_L       0x1C
#define PCA9685_LED5_OFF_H       0x1D

#define PCA9685_LED6_ON_L        0x1E
#define PCA9685_LED6_ON_H        0x1F
#define PCA9685_LED6_OFF_L       0x20
#define PCA9685_LED6_OFF_H       0x21

#define PCA9685_LED7_ON_L        0x22
#define PCA9685_LED7_ON_H        0x23
#define PCA9685_LED7_OFF_L       0x24
#define PCA9685_LED7_OFF_H       0x25

#define PCA9685_LED8_ON_L        0x26
#define PCA9685_LED8_ON_H        0x27
#define PCA9685_LED8_OFF_L       0x28
#define PCA9685_LED8_OFF_H       0x29

#define PCA9685_LED9_ON_L        0x2A
#define PCA9685_LED9_ON_H        0x2B
#define PCA9685_LED9_OFF_L       0x2C
#define PCA9685_LED9_OFF_H       0x2D

#define PCA9685_LED10_ON_L       0x2E
#define PCA9685_LED10_ON_H       0x2F
#define PCA9685_LED10_OFF_L      0x30
#define PCA9685_LED10_OFF_H      0x31

#define PCA9685_LED11_ON_L       0x32
#define PCA9685_LED11_ON_H       0x33
#define PCA9685_LED11_OFF_L      0x34
#define PCA9685_LED11_OFF_H      0x35

#define PCA9685_LED12_ON_L       0x36
#define PCA9685_LED12_ON_H       0x37
#define PCA9685_LED12_OFF_L      0x38
#define PCA9685_LED12_OFF_H      0x39

#define PCA9685_LED13_ON_L       0x3A
#define PCA9685_LED13_ON_H       0x3B
#define PCA9685_LED13_OFF_L      0x3C
#define PCA9685_LED13_OFF_H      0x3D

#define PCA9685_LED14_ON_L       0x3E
#define PCA9685_LED14_ON_H       0x3F
#define PCA9685_LED14_OFF_L      0x40
#define PCA9685_LED14_OFF_H      0x41

#define PCA9685_LED15_ON_L       0x42
#define PCA9685_LED15_ON_H       0x43
#define PCA9685_LED15_OFF_L      0x44
#define PCA9685_LED15_OFF_H      0x45

#define PCA9685_ALL_LED_ON_L     0xFA
#define PCA9685_ALL_LED_ON_H     0xFB
#define PCA9685_ALL_LED_OFF_L    0xFC
#define PCA9685_ALL_LED_OFF_H    0xFD
#define PCA9685_PRE_SCALE        0xFE

// Register Bits
#define PCA9685_ALLCALL          0x01
#define PCA9685_OUTDRV           0x04
#define PCA9685_RESTART          0x80
#define PCA9685_SLEEP            0x10
#define PCA9685_INVERT           0x10

#endif
PCA9685::PCA9685(int address) {
    kI2CBus = 1 ;           // Default I2C bus for Jetson TK1
    kI2CAddress = address ; // Defaults to 0x40 for PCA9685 ; jumper settable
    error = 0 ;
}

PCA9685::~PCA9685() {
    closePCA9685() ;
}

bool PCA9685::openPCA9685()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
       error = errno ;
       return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0) {
        // Could not open the device on the bus
        error = errno ;
        return false ;
    }
    return true ;
}

void PCA9685::closePCA9685()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }
}

void PCA9685::reset () {
    writeByte(PCA9685_MODE1, PCA9685_ALLCALL );
    writeByte(PCA9685_MODE2, PCA9685_OUTDRV) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
}

// Sets the frequency of the PWM signal
// Frequency is ranged between 40 and 1000 Hertz
void PCA9685::setPWMFrequency ( float frequency ) {
    printf("Setting PCA9685 PWM frequency to %f Hz\n",frequency) ;
    float rangedFrequency = fmin(fmax(frequency,40),1000) ;
    int prescale = (int)(25000000.0f / (4096 * rangedFrequency) - 0.5f) ;
    // For debugging
    // printf("PCA9685 Prescale: 0x%02X\n",prescale) ;
    int oldMode = readByte(PCA9685_MODE1) ;
     int newMode = ( oldMode & 0x7F ) | PCA9685_SLEEP ;
    writeByte(PCA9685_MODE1, newMode) ;
    writeByte(PCA9685_PRE_SCALE, prescale) ;
    writeByte(PCA9685_MODE1, oldMode) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
    writeByte(PCA9685_MODE1, oldMode | PCA9685_RESTART) ;
}

// Channels 0-15
// Channels are in sets of 4 bytes
void PCA9685::setPWM ( int channel, int onValue, int offValue) {
    writeByte(PCA9685_LED0_ON_L+4*channel, onValue & 0xFF) ;
    writeByte(PCA9685_LED0_ON_H+4*channel, onValue >> 8) ;
    writeByte(PCA9685_LED0_OFF_L+4*channel, offValue & 0xFF) ;
    writeByte(PCA9685_LED0_OFF_H+4*channel, offValue >> 8) ;
}

void PCA9685::setAllPWM (int onValue, int offValue) {
    writeByte(PCA9685_ALL_LED_ON_L, onValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_ON_H, onValue >> 8) ;
    writeByte(PCA9685_ALL_LED_OFF_L, offValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_OFF_H, offValue >> 8) ;
}


// Read the given register
int PCA9685::readByte(int readRegister)
{
    int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, readRegister);
    if (toReturn < 0) {
        printf("PCA9685 Read Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    // For debugging
    // printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", kI2CAddress, toReturn, readRegister);
    return toReturn ;
}

// Write the the given value to the given register
int PCA9685::writeByte(int writeRegister, int writeValue)
{   // For debugging:
    // printf("Wrote: 0x%02X to register 0x%02X \n",writeValue, writeRegister) ;
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    if (toReturn < 0) {
        printf("PCA9685 Write Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}


class PWMPython: public PCA9685{
private:
    int servoMin;
    int servoMax;
    int map ( int x, int out_min, int out_max) {
        int in_min = 0;
        int in_max = 180;
        int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
        return toReturn ;
    }
    
public:
    PWMPython(int min=120,int max=720,int address=0):PCA9685(address){
        servoMin = min;
        servoMax = max;
    }
    void setPWM(int channel,int degree){
        PCA9685::setPWM(channel,0,map(degree,servoMin,servoMax));
    }
    void setAllPwm(int degree){
        PCA9685::setAllPWM(0,map(degree,servoMin,servoMax));
    }
}
