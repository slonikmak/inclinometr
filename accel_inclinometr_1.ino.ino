// библиотека для работы I²C
#include <Wire.h>
// библиотека для работы с модулями IMU
#include <TroykaIMU.h>
#include <SD.h>
#include <TFT.h> // Arduino LCD library
#include <SPI.h>

// pin definition for the Uno
#define cs 10
#define dc 9
#define rst 8
#define sd_cs 4

//Расположение данных на дисплее
#define pitch_y 10
#define data_x 10
#define roll_y 70

#define BETA 0.2f

TFT TFTscreen = TFT(cs, dc, rst);

// char array to print to the screen
char phiPrintout[5];
char thetaPrintout[5];
char depthPrintout[3];
char setPrintout[2];

const int chipSelect = 4;
Sd2Card card;
SdVolume volume;
SdFile root;
PImage logo;

// создаём объект для работы с акселерометром
Accelerometer accel;

float theta = 0.0f, phi = 0.0f, rho = 0.0f, theta1;
float thetaraw, thetaoffset, thetaoffset_rel, thetaoffset_180, phiraw, buff_phi = 0, buff_theta = 0;
float accx, accy, accz, buff_accx, buff_accy, buff_accz;
int depth = 0, set = 0, depth_buff = 0, set_buff = 1;

bool state = 0;

int koef = 0;

void setup()
{
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  // открываем последовательный порт
  Serial.begin(9600);
  // выводим сообщение о начале инициализации
  ////Serial.println("Accelerometer init...");
  // инициализация акселерометра
  accel.begin();
  // устанавливаем чувствительность акселерометра
  // 2g — по умолчанию, 4g, 8g
  accel.setRange(RANGE_2G);

  init_display();

  init_sd();
  // выводим сообщение об удачной инициализации
  ////Serial.println("Initialization completed");

  print_data_header();
}

void loop()
{
  //Проверяем кнопки
  if(digitalRead(2)){
    digitalWrite(3, LOW);
    if (!state){
      state = 1;
      set += 1;
    }
  } else {
    digitalWrite(3, HIGH);
    state = 0;
  }
  // вывод направления и величины ускорения в м/с² по оси X
  accel.readGXYZ(&buff_accx, &buff_accy, &buff_accz);
  
  //фильтр низких частот
  accx = buff_accx * BETA + (accx * (1.0 - BETA));
  accy = buff_accy * BETA + (accy * (1.0 - BETA));
  accz = buff_accz * BETA + (accz * (1.0 - BETA));

  //if (accx>0 && accy>0) koef = 0;
  //if (accx>0 && accy<0) koef = -180;

  theta = atan2(accx, sqrt(accy * accy + accz * accz)) * 57.2957795 + koef;
  phi = atan2(accy, sqrt(accx * accx + accz * accz)) * 57.2957795;

  Serial.print(theta);
  Serial.print(',');
  theta = round(theta*10.0f);
  Serial.print(theta);
  Serial.print(',');
  theta = theta/10.0f;
  Serial.println(theta);
  phi = round(phi*10.0f)/10.0f;
  
  //фильтр низких частот
  //theta = thetaraw * BETA + (theta * (1.0 - BETA));
  //phi = phiraw * BETA + (phi * (1.0 - BETA));

  calc_depht();

  print_data();

  // Serial.print(buff_accx);
  // Serial.print(',');
  // Serial.println(accx);

  delay(100);
}

void calc_depht()
{
  depth = analogRead(A2);
}

void init_display()
{
  TFTscreen.begin();
  TFTscreen.setRotation(90);
}

void print_data_header()
{
  String str = String(0);
  str.toCharArray(phiPrintout, 4);
  // clear the screen with a black background
  TFTscreen.background(255, 255, 255);
  TFTscreen.stroke(0, 0, 0);
  TFTscreen.setTextSize(2);
  TFTscreen.text("X\n ", 2, 5);
  TFTscreen.text("Y\n ", 2, 60);
  TFTscreen.setTextSize(1);
  TFTscreen.text("Depth     Set \n", 10, 120);
  // ste the font size very large for the loop
  TFTscreen.setTextSize(5);
}

void print_data()
{
  TFTscreen.textSize(4);
  if (buff_phi != abs(phi))
  {
    TFTscreen.stroke(255, 255, 255);
    TFTscreen.text(phiPrintout, 20, pitch_y);

    String sensorVal = String(abs(phi));

    sensorVal.toCharArray(phiPrintout, 5);

    TFTscreen.stroke(0, 0, 0);
    TFTscreen.text(phiPrintout, 20, pitch_y);
    buff_phi = phi;
  }

  if (buff_theta != abs(theta))
  {
    TFTscreen.stroke(255, 255, 255);
    TFTscreen.text(thetaPrintout, 20, roll_y);

    String sensorVal = String(abs(theta));

    sensorVal.toCharArray(thetaPrintout, 5);

    TFTscreen.stroke(0, 0, 0);
    TFTscreen.text(thetaPrintout, 20, roll_y);
    buff_theta = theta;
  }

  if (depth != depth_buff)
  {
    TFTscreen.stroke(255, 255, 255);
    TFTscreen.textSize(2);
    TFTscreen.text(depthPrintout, 20, 130);

    String sensorVal = String((int)depth);

    sensorVal.toCharArray(depthPrintout, 3);

    TFTscreen.stroke(0, 0, 0);
    TFTscreen.text(depthPrintout, 20, 130);
    depth_buff = depth;
  }

  if (set != set_buff)
  {
    TFTscreen.stroke(255, 255, 255);
    TFTscreen.textSize(2);
    TFTscreen.text(setPrintout, 80, 130);

    String sensorVal = String((int)set);

    sensorVal.toCharArray(setPrintout, 2);

    TFTscreen.stroke(0, 0, 0);
    TFTscreen.text(setPrintout, 80, 130);
    set_buff = set;
  }
}
void init_sd()
{
  if (!SD.begin(sd_cs))
  {
    //Serial.println(F("failed!"));
    return;
  }
  //Serial.println(F("OK!"));

  // now that the SD card can be access, try to load the
  // image file.
  //logo = TFTscreen.loadImage("logo.bmp");
  if (logo.isValid() == false)
  {
    return;
  }

  //Serial.println(F("drawing image"));
  if (!logo.isValid())
  {
    //Serial.println(F("error while loading arduino.bmp"));
  }

  TFTscreen.background(255, 255, 255);
  //TFTscreen.image(logo, 0, 20);

  delay(2000);
}
