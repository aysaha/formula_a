#include <Servo.h>

#define FREQUENCY 100

#define STEER_PIN 5
#define DRIVE_PIN 6

#define SERVO_MIN 1000
#define SERVO_MID 1500
#define SERVO_MAX 2000

#define BAUD_RATE 115200
#define BUFFER_SIZE 4

enum state_t {UNKNOWN, RUNNING, ERROR};

state_t state;
uint32_t time;

uint16_t position;
uint16_t speed;

Servo steer;
Servo drive;

void setup()
{
    // initialize steering servo
    steer.attach(STEER_PIN, SERVO_MIN, SERVO_MAX);
    steer.writeMicroseconds(SERVO_MID);
    position = SERVO_MID;

    // initialize driving servo
    drive.attach(DRIVE_PIN, SERVO_MIN, SERVO_MAX);
    drive.writeMicroseconds(SERVO_MID);
    speed = SERVO_MID;

    // initialize serial connection
    Serial.begin(BAUD_RATE);
    while(!Serial);

    // initialize onboard LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // initialize state machine
    state = RUNNING;
}

void loop()
{
    // start time
    time = micros();

    // run state machine
    switch(state)
    {
        case RUNNING:
            state = running();
            break;

        case ERROR:
            state = error();
            break;

        default:
            state = ERROR;
    }

    // write output signals
    steer.writeMicroseconds(saturate(position));
    drive.writeMicroseconds(saturate(speed));

    // synchronize loop
    while(micros() - time < 1000000 / FREQUENCY);
}

state_t running()
{
    static uint8_t clock = 0;
    uint8_t buffer[BUFFER_SIZE];
    size_t data;

    if(Serial.available())
    {
        data = Serial.readBytes(buffer, BUFFER_SIZE);

        if(data < BUFFER_SIZE)
        {
            return(ERROR);
        }
        else
        {
            position = (buffer[0] << 8) | buffer[1];
            speed = (buffer[2] << 8) | buffer[3];

            if(position > SERVO_MAX || position < SERVO_MIN || speed > SERVO_MAX || speed < SERVO_MIN)
            {
                return(ERROR);
            }
        }

        clock = 0;
    }
    else if(clock < FREQUENCY)
    {
        clock += 1;
    }
    else
    {
        position = SERVO_MID;
        speed = SERVO_MID;
    }

    return(RUNNING);
}

state_t error()
{
    static uint8_t clock = 0;

    if(clock < FREQUENCY / 2)
    {
        clock += 1;
    }
    else
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        clock = 0;
    }

    position = SERVO_MID;
    speed = SERVO_MID;

    return(ERROR);
}

uint16_t saturate(uint16_t value)
{
    if(value > SERVO_MAX)
    {
        return(SERVO_MAX);
    }
    else if(value < SERVO_MIN)
    {
        return(SERVO_MIN);
    }
    else
    {
        return(value);
    }
}
