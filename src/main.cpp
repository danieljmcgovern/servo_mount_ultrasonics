#include <Arduino.h>
#include <NewPing.h>       //https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <Servo.h>         //https://www.arduino.cc/reference/en/libraries/servo/
#include <RunningMedian.h> //https://playground.arduino.cc/Main/RunningMedian/
#include <math.h>
#include <QuickStats.h>   

//PINS
#define ULTRASONIC_PING_FRONT 4
#define ULTRASONIC_ECHO_FRONT 5
#define ULTRASONIC_PING_REAR 6
#define ULTRASONIC_ECHO_REAR 7
#define SERVO_FRONT 9 //pin for front servo
#define SERVO_REAR 10 //pin for rear servo

#define MAX_DISTANCE 500    // Maximum distance (cm) to ping.
#define ULTRASONIC_DELAY 30 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SERVO_DELAY 200     //allow time for the servo to move to new position after calling "servo".write()
#define SWEEP_SEGMENTS 40
#define SERVO_START 40
#define SERVO_END 140
#define MEASUREMENT_COUNT 3 //number of measurements to take at each servo position
#define SEGMENT_SIZE (SERVO_END - SERVO_START) / SWEEP_SEGMENTS

Servo servo_front;
Servo servo_rear;
NewPing ultrasonic_front(ULTRASONIC_PING_FRONT, ULTRASONIC_ECHO_FRONT, MAX_DISTANCE);
NewPing ultrasonic_rear(ULTRASONIC_PING_REAR, ULTRASONIC_ECHO_REAR, MAX_DISTANCE);

void setup()
{
  Serial.begin(115200);
  pinMode(SERVO_FRONT, OUTPUT);
  pinMode(SERVO_REAR, OUTPUT);
  servo_front.attach(SERVO_FRONT);
  servo_rear.attach(SERVO_REAR);
}

struct RecordedMeasurement
{
  int servoPos;
  float distance;
};

RecordedMeasurement measure_front[SWEEP_SEGMENTS];
RecordedMeasurement measure_rear[SWEEP_SEGMENTS];
RecordedMeasurement lowest_front;
RecordedMeasurement lowest_rear;
RunningMedian subMeasurement_front(MEASUREMENT_COUNT);
RunningMedian subMeasurement_rear(MEASUREMENT_COUNT);
RunningMedian runningMeasurement_front(SWEEP_SEGMENTS);
RunningMedian runningMeasurement_rear(SWEEP_SEGMENTS);

float front_perp_dist;
float rear_perp_dist;

void sweepForMedian()
{

  for (int i = 0; i < SWEEP_SEGMENTS; i++)
  {
    measure_front[i].servoPos = (SERVO_START + (i * SEGMENT_SIZE));
    measure_rear[i].servoPos = (SERVO_START + (i * SEGMENT_SIZE));
    servo_front.write(measure_rear[i].servoPos);
    servo_rear.write(measure_rear[i].servoPos);
    delay(SERVO_DELAY);
    subMeasurement_front.clear();
    subMeasurement_rear.clear();
    for (int j = 0; j < MEASUREMENT_COUNT; j++)
    {
      subMeasurement_front.add(ultrasonic_front.ping_cm());
      delay(ULTRASONIC_DELAY);
      subMeasurement_rear.add(ultrasonic_rear.ping_cm());
      delay(ULTRASONIC_DELAY);
    }
    measure_front[i].distance = subMeasurement_front.getMedian();
    measure_rear[i].distance = subMeasurement_rear.getMedian();
    runningMeasurement_front.add(measure_front[i].distance);
    runningMeasurement_rear.add(measure_rear[i].distance);

    if (i == 0 || measure_front[i].distance < lowest_front.distance)
    {
      if(measure_front[i].distance == 0.0) break;
      lowest_front = measure_front[i];
    }
    if (i == 0 || measure_rear[i].distance < lowest_rear.distance)
    {
      if(measure_rear[i].distance == 0.0) break;
      lowest_rear = measure_rear[i];
    }

    //Serial.print("servoPos: ");
    Serial.print(measure_front[i].servoPos);
    //Serial.print("\t\tfront: ");
    Serial.print(",");
    Serial.print(measure_front[i].distance);
    //Serial.print("\t\trear: ");
    Serial.print(",");
    Serial.println(measure_rear[i].distance);
  }
  Serial.print("Lowest front: ");
  Serial.print(lowest_front.servoPos);
  Serial.print(",");
  Serial.println(lowest_front.distance);
  Serial.print("Lowest rear: ");
  Serial.print(lowest_rear.servoPos);
  Serial.print(",");
  Serial.println(lowest_rear.distance);

  // front_perp_dist = runningMeasurement_front.getMedian();
  // rear_perp_dist = runningMeasurement_rear.getMedian();
  


  Serial.print("running median front: ");
  Serial.println(front_perp_dist);
  Serial.print("running median rear: ");
  Serial.println(rear_perp_dist);
}

float front_mode;
float rear_mode;

void mode()
{
  //float *front = new float(SWEEP_SEGMENTS);
  float front[SWEEP_SEGMENTS];
  for (int i = 0; i < SWEEP_SEGMENTS; i++)
  {
    front[i] = measure_front[i].distance;
  }

  //float *rear = new float(SWEEP_SEGMENTS);
  float rear[SWEEP_SEGMENTS];
  for (int i = 0; i < SWEEP_SEGMENTS; i++)
  {
    rear[i] = measure_rear[i].distance;
  }

  QuickStats stats;

  front_mode = stats.mode(front, SWEEP_SEGMENTS, 0.1);
  rear_mode = stats.mode(rear, SWEEP_SEGMENTS, 0.1);

  Serial.print("front mode: ");
  Serial.println(front_mode);
  Serial.print("rear mode: ");
  Serial.println(rear_mode);
}

#define D 36.5

double computeAngle()
{
  double theta = asin(abs(front_perp_dist - rear_perp_dist) / D);
  return theta * 180 / PI;
}

void loop()
{
  sweepForMedian();
  Serial.println(computeAngle());  
  mode();
  delay(60000);
}

/*

*/