#include <hal_conf_extra.h>
#include <STM32CAN.h>
#include <RPLidar.h>

// angle d'observation de 110° (55*2)
#define ANGLE_DOBSERVATION 55
#define DISTANCE_MAX 300
#define LED PA14
RPLidar lidar;
CAN_message_t Received_msg;
CAN_message_t Transmit_msg;

float distance = 0, angle = 0;

void setup()
{

  lidar.begin(Serial1);

  Can1.begin(CAN_BPS_1000K);

  delay(10);
  // Indication du démarrage du lidar et du can par un clignotement de la LED
  pinMode(LED, OUTPUT);
  digitalToggle(LED);
  delay(50);
  digitalToggle(LED);
  delay(50);
  digitalToggle(LED);
  delay(50);
  digitalToggle(LED);
  delay(50);
  digitalWrite(LED, HIGH);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 250); // démarrage du moteur du lidar
}

void loop()
{  
  // la fonction read renvoie 1 quand un message arrive dans le buffer du controlleur CAN
  while (Can1.read(Received_msg))
  {
    switch (Received_msg.id)
    {
    case 0x101: // Message de bon démarrage
      Transmit_msg.id = 0x201;
      Transmit_msg.data.bytes[0] = 0x20;
      Can1.write(Transmit_msg);
      break;
    case 0x102: // Message de fin de match
      analogWrite(RPLIDAR_MOTOR, 0);
      lidar.end();
      break;
    }
  }
  // Acquisition de 200 points allant de 315 à 45°, et de 135 à 225°
  // On dit que le 0 du lidar est à l'opposé du câble
  if (IS_OK(lidar.waitPoint()))
  {
    // lecture des valeurs du lidar et met dans la liste
    distance = lidar.getCurrentPoint().distance;
    angle = lidar.getCurrentPoint().angle;
    Serial.print("distance=");
    Serial.println(distance);
    Serial.print("angle=");
    Serial.println(angle);
    // si la valeur de la liste est dans l'angle d'observation
    if (((angle >= 0 && angle <= ANGLE_DOBSERVATION) || (angle >= 360 - ANGLE_DOBSERVATION && angle < 360)) || (angle >= 180 - ANGLE_DOBSERVATION && angle <= 180 + ANGLE_DOBSERVATION))
    {
      if (distance <= DISTANCE_MAX && distance != 0)
      { // si dans la zone de vigilance
        envoi_CAN();
      }
    }
  }
  else
  { // si bug du lidar

    analogWrite(RPLIDAR_MOTOR, 0); // stop the rplidar motor
    Serial.println("Waiting for lidar to start...");
    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100)))
    {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 250);
      delay(1000);
    }
  }
}
// envoi de trame CAN à la carte principale
void envoi_CAN()
{
  Transmit_msg.id = 0x202; // id de la carte principale
  Transmit_msg.data.bytes[0] = distance && 0xFF000000;
  Transmit_msg.data.bytes[1] = distance && 0x00FF0000;
  Transmit_msg.data.bytes[2] = distance && 0x0000FF00;
  Transmit_msg.data.bytes[3] = distance && 0x000000FF;

  Transmit_msg.data.bytes[4] = angle && 0xFF000000;
  Transmit_msg.data.bytes[5] = angle && 0x00FF0000;
  Transmit_msg.data.bytes[6] = angle && 0x0000FF00;
  Transmit_msg.data.bytes[7] = angle && 0x000000FF;
  Transmit_msg.dlc = 8;
  Can1.write(Transmit_msg);
}
