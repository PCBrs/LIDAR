#include <Arduino.h>
#include <RPLidar.h>
IntervalTimer myTimer;
#include <MsTimer2.h>  // Timer
#include <FlexCan.h>
#include <kinetis_flexcan.h>

//angle d'observation de 110° (55*2)
#define ANGLE_DOBSERVATION 55

#define ZONE_DE_VIGILANCE 450

#define LIMITE_X 3000
#define LIMITE_Y 2000

RPLidar lidar;

static CAN_message_t msg;
static CAN_filter_t Carte_Main;

float x = 0, y = 0, angle = 0; // DEPLACEMENT venant du CAN
boolean sens=true; // vient du CAN --> 1 : marche avant, 0 : marche arriere

int i = 0;
int derniere_detection=0, diff_timer=0;
boolean passe=true;
int distance, valeur[1][2];
//byte simul_CAN[5]={0xE8,0xDC,0x5A,0x5A,0x01};
int c1=0,c2=0,c3=0,c4,c5,c6;

float x_point, y_point;

void IntrerrupTimer(){
  derniere_detection++;
  if (derniere_detection>=100000)derniere_detection=100;
}

void setup() {
  Serial.begin(9600);
  lidar.begin(Serial2); //On est sur le TX2 et RX2

  Carte_Main.id=0x200;

  Can0.begin(1000000);
  Can0.setFilter(Carte_Main,1);


  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  pinMode(RPLIDAR_MOTOR, OUTPUT);

  analogWrite(RPLIDAR_MOTOR, 250); //démarrage du moteur du lidar

  myTimer.begin(IntrerrupTimer, 1000);
}


void loop() {
  // la fonction read renvoie 1 quand un message arrive dans le buffer du controlleur CAN
  while( Can0.read(msg)) {
    //la trame 0 fait office de filtre avec un '/' envoyé depuis la carte principale
    if(msg.buf[0]=='/'){
      c1=msg.buf[4];
      c2=msg.buf[4]; // empêche de modifier la trame 4 utilisée 3 fois pour x y angle
      c3=msg.buf[4]; // Trame[4]=[ayyy xxxx]

      x=(int)(msg.buf[1]+((c1&=(0x0F))<<8));
      y=(int)(msg.buf[2]+((c2&=(0x70))<<4));
      angle=(int)(msg.buf[3]+((c3&=(0x80))<<1));

      //Trame[5]=[---- -(-y)(-x)sens]
      c4=msg.buf[5];
      c5=msg.buf[5];
      c6=msg.buf[5];
      sens=(c4&=(0x01));
      if((c5&=(0x02))>>1) x*=-1;
      if((c6&=(0x04))>>2) y*=-1;
      //Serial.println(msg.buf[5]);
      //  Serial.println(msg.buf[1]);
      //  Serial.println(msg.buf[2]);
      //  Serial.println(msg.buf[4]);
      // Serial.print("xya");
      // Serial.print(x);
      // Serial.print("   ");
      // Serial.print(y);
      // Serial.print("sens");
      // Serial.println(sens);
    }

  }


  //Acquisition de 200 points allant de 315 à 45°, et de 135 à 225°
  //On dit que le 0 du lidar est à l'opposé du câble

    //si l'acquisition se fait et robot ne tourne pas
    if (IS_OK(lidar.waitPoint())){
      //lecture des valeurs du lidar et met dans la liste
      valeur[i][0]=lidar.getCurrentPoint().distance;
      valeur[i][1]=lidar.getCurrentPoint().angle;

      if (x>900 && x>2100 && y>1700){}//pas de détection si on EST dans la zone rocheuse
      else
      {
        //si la valeur de la liste est dans l'angle d'observation
        //et si l'angle mesuré est dans le même sens que la marche (sens false --> avant ; true --> arrière)
        if((sens==false && ((valeur[i][1] >= 0 && valeur[i][1] <= ANGLE_DOBSERVATION) || (valeur[i][1] >= 360 - ANGLE_DOBSERVATION && valeur[i][1] < 360))) || (sens==true  && (valeur[i][1] >= 180 - ANGLE_DOBSERVATION && valeur[i][1] <= 180 + ANGLE_DOBSERVATION))){
          if (valeur[i][0]<=ZONE_DE_VIGILANCE && valeur[i][0]!=0){ //si dans la zone de vigilance
            // BESOIN DE CONVERTIR UN DES DEUX ANGLES POUR AVOIR DANS LE MEME SENS
            // CAN : sens trigo ; LIDAR : sens horaire
            //angle=360-angle; //conversion angle rotation robot sens trigo en angle sens horaire
            //Serial.println("2e if");
            //calcul des coordoonées du point vu par le lidar
            x_point=-valeur[i][0]*sin((angle+valeur[i][1])*PI/180-PI/2)+x;
            y_point=valeur[i][0]*cos((angle+valeur[i][1])*PI/180-PI/2)+y;

            if (x_point>900 && x_point>2100 && y_point>1700){}//pas de détection si on VOIT dans la zone rocheuse
            else{
              //si le point trouvé est sur le terrain et si cette detection se fait
              if (x_point<=LIMITE_X-100 && x_point>=0+100 && y_point<=LIMITE_Y-100 && y_point>=0+100 && derniere_detection>50){
                derniere_detection=0; //Si on retourne dans cette condition dans la même seconde
                Serial.println("DETECTION");
                Serial.println(x_point);
                Serial.println(y_point);
                envoi_CAN();
              }
            }
            
          }
        }
      }


  }else {//si bug du lidar

    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();

       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 250);
       delay(1000);
    }
  }
}

//envoi de trame CAN à la carte principale
void envoi_CAN(){
  msg.id=0x100; //id de la carte principale
  msg.buf[0]='L';//dit à la carte principale qu'il s'agit un message de la carte Lidar
  msg.buf[1]='D';//dit à la carte principale qu'il y a DETECTION
  msg.len=2;
  Can0.write(msg);
}
