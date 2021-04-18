#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <MechaQMC5883.h>
#include <BlynkSimpleSerialBLE.h>
#include <RobotMotion.h>

char authenticationKey[] = "vWVJ8YZeHavp5iXS4U2eBT1rOrop_azE";

TinyGPS onboardGPS;
MechaQMC5883 compass;
RobotMotion robot(2, 22, 23, 3, 24, 25);

WidgetTerminal terminal(V3);

struct GeoLocation
{
     float latitude;
     float longitude;
};

// Onboard GPS location
GeoLocation onboardLocation;

// Mobile Phone GPS location
GeoLocation mobileLocation;

// distance message
String distanceMsg;

void setup()
{
     Serial.begin(9600);  // arduino serial
     Serial1.begin(9600); // onboard gps serial
     Serial2.begin(9600); // bluetooth serial

     compass.init();                                               // initialize compass
     compass.setMode(Mode_Continuous, ODR_200Hz, RNG_2G, OSR_256); // set compass mode

     Blynk.begin(Serial2, authenticationKey); // initialize Blynk

     distanceMsg = "0m";

     terminal.clear();

     robot.setSpeed(200);
     robot.Stop();
}

// Mobile GPS Stream
BLYNK_WRITE(V0)
{
     GpsParam mobileGPS(param);

     mobileLocation.latitude = mobileGPS.getLat();
     mobileLocation.longitude = mobileGPS.getLon();

     Serial.println(mobileLocation.latitude, 10);
}

// Killswitch
BLYNK_WRITE(V1)
{
}

// DISPLAY DISTANCE AND TERMINAL
BLYNK_READ(V2)
{
     terminal.clear();

     Blynk.virtualWrite(V2, distanceMsg);

     if (onboardLocation.latitude != 0 && onboardLocation.longitude != 0)
     {
          terminal.println("Onboard GPS:");
          terminal.println(onboardLocation.latitude, 10);
          terminal.println(onboardLocation.longitude, 10);
     }
     else
     {
          terminal.println("Initializing Onboard GPS...");
     }

     terminal.flush();
}

// bearing angle of A with respect to B
float bearing(GeoLocation A, GeoLocation B)
{
     float deltaL = abs(A.longitude - B.longitude);

     float cosA = cos(A.latitude * DEG_TO_RAD);
     float cosB = cos(B.latitude * DEG_TO_RAD);
     float sinA = sin(A.longitude * DEG_TO_RAD);
     float sinB = sin(B.longitude * DEG_TO_RAD);

     float x = cosB * sin(deltaL * DEG_TO_RAD);
     float y = cosA * sinB - sinA * cosB * cos(deltaL * DEG_TO_RAD);

     float bearing = atan2(x, y) * RAD_TO_DEG;
     bearing = fmod((bearing + 360), 360);

     return bearing;
}

// heading degrees of the robot from the north
float heading()
{
     float headingDegrees;
     int x, y, z;

     compass.read(&x, &y, &z);

     headingDegrees = compass.azimuth(&y, &x);

     return headingDegrees;
}

// get distance between two points using Harvensine's formula
float distance(GeoLocation A, GeoLocation B)
{
     const float R = 6371000;                // Earth's radius (km)
     float p1 = A.latitude * DEG_TO_RAD;
     float p2 = B.latitude * DEG_TO_RAD;
     float dp = (B.latitude - A.latitude) * DEG_TO_RAD;
     float dl = (A.longitude - B.longitude) * DEG_TO_RAD;

     float x = pow(sin(dp / 2), 2) * 
               sin(dp / 2) + cos(p1) * cos(p2) * 
               pow(sin(dl / 2), 2);

     float y = 2 * atan2(sqrt(x), sqrt(1 - x));

     return R * y;
}

// overload != operator
bool operator!=(GeoLocation location, int num)
{
     if (location.latitude != num && location.longitude != num)
          return true;

     return false;
}

void loop()
{
     Blynk.run();

     if (Serial1.available())
     {
          if (onboardGPS.encode(Serial1.read()))
          {
               // get robot latitude and longitude
               onboardGPS.f_get_position(&onboardLocation.latitude, &onboardLocation.longitude);
          }
     }

     if (onboardLocation != 0 && mobileLocation != 0)
     {
          int bearingAngle = (int)bearing(onboardLocation, mobileLocation);
          int headingAngle = (int)heading();

          float dist = distance(onboardLocation, mobileLocation);

          distanceMsg = String(dist) + "m";

          if(dist <= 1.5)
          {
               robot.Stop();
          }
          else if (headingAngle == bearingAngle)
          {
               robot.Forward();
          }
          else if (headingAngle < bearingAngle)
          {
               int RIGHT = headingAngle - bearingAngle;
               int LEFT = 360 - RIGHT;

               if(RIGHT < LEFT)
                    robot.Right();
               else
                    robot.Left();
          }
          else if (headingAngle > bearingAngle)
          {
               int LEFT = headingAngle - bearingAngle;
               int RIGHT = 360 - LEFT;

               if(LEFT < RIGHT)
                    robot.Left();
               else
                    robot.Right();
          }
     }
}
