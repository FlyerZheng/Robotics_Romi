#ifndef _Line_follow_h
#define _Line_follow_h

const int NUM_CALIBRATIONS = 20;

class LineSensor
{
    public:
        LineSensor(int pin);
        void calibrate();
        int  readRaw();
        float  readCalibrated();
    
    private:

        float calibration_offset=0;
        int pin;
    
};

LineSensor::LineSensor(int line_pin)
{

    pin = line_pin;
    pinMode(pin, INPUT);

}


int LineSensor::readRaw()
{
    return analogRead(pin);
}


void LineSensor::calibrate()
{

    analogWrite(BUZZER_PIN, 255);

    for (int i=0;i<NUM_CALIBRATIONS;i++)
    {
        calibration_offset += analogRead(pin);
        delay(10);
    }

    calibration_offset =  calibration_offset / NUM_CALIBRATIONS;

    analogWrite(BUZZER_PIN, 0);

}

float LineSensor::readCalibrated()
{
    return (analogRead(pin) - calibration_offset);
}

#endif
