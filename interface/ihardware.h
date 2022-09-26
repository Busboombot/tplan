
class IHardware {

    public:

    IHardware() = default;
    virtual ~IHardware()= default;

    virtual void writePin(int pin, int value) = 0;
    virtual int readPint(int pin) = 0;

    virtual unsigned long millis() = 0;
    virtual unsigned long micros() = 0;

    virtual void delayMillis(unsigned long) = 0;
    virtual void delayMicros(unsigned long) = 0;

};