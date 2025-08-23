/* Nano TinyController
   - Motor control via L298N (ENA=D11, IN1=D4, IN2=D5)
   - Steering SG90 servo on D9
   - Encoders on D2 (A) and D3 (B)
   - Ultrasonics on D8/D12 (L) and D10/D13 (R)
   - IR sensors on A0, A1
   - Serial protocol: ASCII lines over Serial @115200
       Incoming:  DRV <speed> <F|R|S>    (speed 0-255)
                  STR <angle>            (0-180)
                  PING
                  STOP
       Outgoing telemetry (every 500 ms):
                  ENC <ticks>
                  RNG <left_cm> <right_cm>
                  IR <l> <r>
*/
#include <Servo.h>

const int ENA = 11;
const int IN1 = 4;
const int IN2 = 5;

const int SERVO_PIN = 9;

const int ENC_A_PIN = 2;
const int ENC_B_PIN = 3;

const int IR_LEFT = A0;
const int IR_RIGHT = A1;

const int US_L_TRIG = 8;
const int US_L_ECHO = 12;
const int US_R_TRIG = 10;
const int US_R_ECHO = 13;

volatile long encoderCount = 0;
volatile int lastEncoded = 0;

Servo steering;

String rxBuffer = "";
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 500; // ms

// ---- Encoder update (quadrature)
void updateEncoder()
{
    int MSB = digitalRead(ENC_A_PIN);
    int LSB = digitalRead(ENC_B_PIN);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;
    // decode table
    if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000)
        encoderCount++;
    if (sum == 0b0010 || sum == 0b0100 || sum == 0b1101 || sum == 0b1011)
        encoderCount--;
    lastEncoded = encoded;
}

// ---- Ultrasonic single measurement (cm)
long readUltrasonicCm(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30 ms
    if (duration == 0)
        return -1; // timeout
    long cm = duration / 29 / 2;
    return cm;
}

// ---- Motor control
void driveMotor(int speed, char dir)
{
    speed = constrain(speed, 0, 255);
    if (dir == 'F' || dir == 'f')
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, speed);
    }
    else if (dir == 'R' || dir == 'r')
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, speed);
    }
    else
    { // stop/brake
        analogWrite(ENA, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
}

// ---- Serial command processing
void processCommand(String s)
{
    s.trim();
    if (s.length() == 0)
        return;
    if (s.startsWith("DRV "))
    {
        int i1 = s.indexOf(' ');
        int i2 = s.indexOf(' ', i1 + 1);
        if (i2 == -1)
        {
            // invalid
            return;
        }
        String sp = s.substring(i1 + 1, i2);
        String dir = s.substring(i2 + 1);
        int speed = sp.toInt();
        char d = dir.length() ? dir.charAt(0) : 'S';
        driveMotor(speed, d);
        Serial.print("OK DRV ");
        Serial.print(speed);
        Serial.print(" ");
        Serial.println(d);
    }
    else if (s.startsWith("STR "))
    {
        int angle = s.substring(4).toInt();
        angle = constrain(angle, 0, 180);
        steering.write(angle);
        Serial.print("OK STR ");
        Serial.println(angle);
    }
    else if (s == "PING")
    {
        Serial.println("PONG");
    }
    else if (s == "STOP")
    {
        driveMotor(0, 'S');
        Serial.println("OK STOP");
    }
    else
    {
        // unknown
        Serial.print("ERR UNKNOWN: ");
        Serial.println(s);
    }
}

void sendTelemetry()
{
    // ENC
    Serial.print("ENC ");
    Serial.println(encoderCount);

    // Ultrasonics
    long l = readUltrasonicCm(US_L_TRIG, US_L_ECHO);
    long r = readUltrasonicCm(US_R_TRIG, US_R_ECHO);
    Serial.print("RNG ");
    Serial.print(l);
    Serial.print(" ");
    Serial.println(r);

    // IR
    int irl = digitalRead(IR_LEFT);
    int irr = digitalRead(IR_RIGHT);
    Serial.print("IR ");
    Serial.print(irl);
    Serial.print(" ");
    Serial.println(irr);
}

void setup()
{
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);

    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);

    pinMode(US_L_TRIG, OUTPUT);
    pinMode(US_L_ECHO, INPUT);
    pinMode(US_R_TRIG, OUTPUT);
    pinMode(US_R_ECHO, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), updateEncoder, CHANGE);

    steering.attach(SERVO_PIN);
    steering.write(90); // center

    Serial.begin(115200);
    // safe stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
}

void loop()
{
    // Serial receive (non-blocking)
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n')
        {
            processCommand(rxBuffer);
            rxBuffer = "";
        }
        else if (c >= 13)
        { // ignore CR alone
            rxBuffer += c;
        }
    }

    // periodic telemetry
    unsigned long now = millis();
    if (now - lastTelemetry >= TELEMETRY_INTERVAL)
    {
        lastTelemetry = now;
        sendTelemetry();
    }
    // do not block the loop
}
