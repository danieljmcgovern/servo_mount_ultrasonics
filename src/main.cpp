#include <Arduino.h>
#include <NewPing.h>       //https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <Servo.h>         //https://www.arduino.cc/reference/en/libraries/servo/
#include <math.h>
#include <QuickStats.h>
#include "DHT.h"            //https://github.com/adafruit/DHT-sensor-library
#include "Adafruit_Sensor.h"

//PINS
#define DHTPIN 3  //pin for DHT11 temperature&humidity sensor
#define ULTRASONIC_PING_FRONT 4
#define ULTRASONIC_ECHO_FRONT 5
#define ULTRASONIC_PING_REAR 6
#define ULTRASONIC_ECHO_REAR 7
#define SERVO_FRONT 9 //pin for front servo
#define SERVO_REAR 10 //pin for rear servo

#define MAX_DISTANCE 500                                        // Maximum distance (cm) to ping.
#define ULTRASONIC_DELAY 30                                     // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define SERVO_DELAY 200                                         //allow time for the servo to move to new position after calling "servo.write()""
#define SWEEP_SEGMENTS 40                                       //number of intervals at which servo will pause to collect a data point
#define SERVO_START 40                                          //in degrees, the beginning of the sweep
#define SERVO_END 140                                           //the end of the sweep
#define MEASUREMENT_COUNT 3                                     //number of measurements to take at each servo position
#define SEGMENT_SIZE (SERVO_END - SERVO_START) / SWEEP_SEGMENTS //each servo write will move the servo this much

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

struct RecordedMeasurement
{
  int servoPos;
  float distance;
};

Servo servo_front;
Servo servo_rear;

NewPing ultrasonic_front(ULTRASONIC_PING_FRONT, ULTRASONIC_ECHO_FRONT, MAX_DISTANCE);
NewPing ultrasonic_rear(ULTRASONIC_PING_REAR, ULTRASONIC_ECHO_REAR, MAX_DISTANCE);

RecordedMeasurement measure_front[SWEEP_SEGMENTS];
RecordedMeasurement measure_rear[SWEEP_SEGMENTS];

float front_perp_dist; //to be used by computeAngle as the effective shortest distance to the wall. Using mode() to set this value.
float rear_perp_dist;

void setup()
{
  Serial.begin(115200);
  pinMode(SERVO_FRONT, OUTPUT);
  pinMode(SERVO_REAR, OUTPUT);
  servo_front.attach(SERVO_FRONT);
  servo_rear.attach(SERVO_REAR);
  dht.begin();
}

float speedOfSound(float temp)
{
  return 331.0*sqrt(1.0+(temp/273.0)); //speed of sound (meters per sec) in air as fxn of temperature (celsius)
}

void measurementSweep()
{
  float t = dht.readTemperature();  //temperature in celsius from DHT11 sesnor
  float v = speedOfSound(t);  //speed of sound in air at that temp (in m/s)
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    delay(5000);
    return;
  }
  Serial.print(F("Temperature: "));
  Serial.print(t);
  Serial.println(F(" °C"));

  for (int i = 0; i < SWEEP_SEGMENTS; i++)
  {
    //set servo position
    measure_front[i].servoPos = (SERVO_START + (i * SEGMENT_SIZE));
    measure_rear[i].servoPos = (SERVO_START + (i * SEGMENT_SIZE));
    //write the servo to each of its positions
    servo_front.write(measure_rear[i].servoPos);
    servo_rear.write(measure_rear[i].servoPos);
    delay(SERVO_DELAY);           //time needed for servo to get to its position        
    
       

    //the distance value associated with that servo position
    measure_front[i].distance = ultrasonic_front.ping_median()*v*pow(10,-4)*0.5; //10^-6 microsec to sec * 10^2 meters to cm = 10^-4
    delay(ULTRASONIC_DELAY); //how long of a delay???                                 
    measure_rear[i].distance = ultrasonic_rear.ping_median()*v*pow(10,-4)*0.5;
    delay(ULTRASONIC_DELAY);

    //PRINT: print all the data in CSV form
    //Serial.print("servoPos: ");
    Serial.print(measure_front[i].servoPos);
    //Serial.print("\t\tfront: ");
    Serial.print(",");
    Serial.print(measure_front[i].distance);
    //Serial.print("\t\trear: ");
    Serial.print(",");
    Serial.println(measure_rear[i].distance);
  }
}

void lowest()
{
  RecordedMeasurement lowest_front;
  RecordedMeasurement lowest_rear;
  for (int i = 0; i < SWEEP_SEGMENTS; i++)
  {
    //LOWEST: find the lowest value recorded
    if (i == 0 || measure_front[i].distance < lowest_front.distance)
    {
      if (measure_front[i].distance == 0.0) //discard zero values, they must be some kind of aberration, min distance on sensors is around 3cm
        break;
      lowest_front = measure_front[i];
    }
    if (i == 0 || measure_rear[i].distance < lowest_rear.distance)
    {
      if (measure_rear[i].distance == 0.0)
        break;
      lowest_rear = measure_rear[i];
    }
  }
  //PRINT LOWEST: print the lowest distance recorded
  Serial.print("lowest front, servoPos: ");
  Serial.print(lowest_front.servoPos);
  Serial.print(", distance: ");
  Serial.println(lowest_front.distance);
  Serial.print("lowest rear, servoPos: ");
  Serial.print(lowest_rear.servoPos);
  Serial.print(", distance: ");
  Serial.println(lowest_rear.distance);
}

void mode()
{
  float front_mode;
  float rear_mode;
  //because an array must be passed to stats.mode, created one from the RecordedMeasurement struct... TODO is there a better way?
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

  front_mode = stats.mode(front, SWEEP_SEGMENTS, 0.5);    //third parameter is epsilon, TODO can it be used to merge two really close values?
  rear_mode = stats.mode(rear, SWEEP_SEGMENTS, 0.5);

  front_perp_dist = front_mode;
  rear_perp_dist = rear_mode;

  Serial.print("front mode: ");
  Serial.println(front_mode);
  Serial.print("rear mode: ");
  Serial.println(rear_mode);
}

void median()
{
  float front_median;
  float rear_median;

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
  //lots of redundancies with mode() TODO how to make it cleaner?
  QuickStats stats;

  front_median = stats.median(front, SWEEP_SEGMENTS);
  rear_median = stats.median(rear, SWEEP_SEGMENTS);

  Serial.print("front median: ");
  Serial.println(front_median);
  Serial.print("rear median: ");
  Serial.println(rear_median);
}

#define D 36.5 //distance between the two sensors

double computeAngle()
{
  double theta = asin(abs(front_perp_dist - rear_perp_dist) / D);
  return theta * 180 / PI;
}

void loop()
{
  measurementSweep();
  lowest();
  mode();
  median();
  Serial.print("Angle off parallel: ");
  Serial.println(computeAngle());
  delay(60000);  

}
