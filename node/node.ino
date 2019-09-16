#include <SPI.h>
#include <Wire.h>
#include "Adafruit_MCP4725.h"
#include <SoftwareSerial.h>
#include "Adafruit_BME280.h"

#define voltsIn A0
#define DATIN 12  // pin 12 : SPI data input
#define SCLOCK 13 // pin 13 : SPI clock pin
#define CHIPSEL 10 //pin 10 : Slave Chip Select 1
#define ADC_VREF 3.3
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define VCC 3.3
#define DEFAULT_SV 4.3
#define DEFAULT_HV 5
#define MUX_0 8
#define MUX_1 9
#define BT_VCC 4
#define BT_GND 3
#define READ 64
#define SET_SV 63
#define SET_HV 62

/*
   Data packet format
   [cmd data0 data1 data2 .... data10]

   00:21:13:02:B2:DE NODE1
   98:D3:21:FC:75:79 NODE2
*/

int hval = 0, sval = 0, mux = 0;
uint8_t cnt = 0;
float dacout1, dacout2, gain = 3.2;
float adc, temp, pres, rh;
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;
Adafruit_BME280 bme;
SoftwareSerial BTSerial(5, 6); // RX | TX

float mcp3553_single_read()
{
  float val;
  uint8_t data[4];
  data[2] = 0xFF;
  while (data[2] == 0xFF)
  {
    data[0] = SPI.transfer(0xff);
    data[1] = SPI.transfer(0xaa);
    data[2] = SPI.transfer(0xff);
    data[3] = SPI.transfer(0xaa);
    val = 3.3 * (data[0] * 0x10000 + data[1] * 0x100 + data[2]) / 0x200000;
    //Serial.print(data[0], HEX);
    //Serial.print("\t");
    //Serial.print(data[1], HEX);
    //Serial.print("\t");
    //Serial.println(data[2], HEX);

  }
  if (val > 3 and mux != 3)
  {
    mux++;
    set_mux();
  }
  else if (val < 1 and mux != 0)
  {
    mux--;
    set_mux();
  }
  return (val);
}

float mcp3553_avg()
{
  float sum = 0;
  int i = 0;
  for (i = 0; i < 100; i++)
  {
    sum = sum + mcp3553_single_read();
  }
  sum = sum / 100.0;
  return (sum);
}


void set_mux()
{
  switch (mux) {
    case 0:
      digitalWrite(MUX_0, LOW);
      digitalWrite(MUX_1, LOW);
    case 1:
      digitalWrite(MUX_0, HIGH);
      digitalWrite(MUX_1, LOW);
    case 2:
      digitalWrite(MUX_0, LOW);
      digitalWrite(MUX_1, HIGH);
    case 3:
      digitalWrite(MUX_0, HIGH);
      digitalWrite(MUX_1, HIGH);
  }
}

void mcp3553_setup()
{
  pinMode(DATIN, INPUT);
  pinMode(SCLOCK, OUTPUT);
  pinMode(CHIPSEL, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  digitalWrite(CHIPSEL, HIGH);
}

void set_sv(float sv)
{
  dacout2 = (sv / gain);
  int x = (dacout2 * 4096) / 3.3;
  if (x > sval)
  {
    for (int i = sval; i <= x; i++)
    {
      dac2.setVoltage(i, false);
      delay(2);
    }
  }
  else if (x < sval)
  {
    for (int i = sval; i >= x; i--)
    {
      dac2.setVoltage(i, false);
      delay(2);
    }
  }
  sval = x;
  //Serial.println(sval);
}

void set_hv(float hv)
{
  dacout1 = (hv / gain);
  int x = (dacout1 * 4096) / 3.3;
  if (x > hval)
  {
    for (int i = hval; i <= x; i++)
    {
      dac1.setVoltage(i, false);
      delay(2);
    }
  }
  else if (x < hval)
  {
    for (int i = hval; i >= x; i--)
    {
      dac1.setVoltage(i, false);
      delay(2);
    }
  }
  hval = x;
  //Serial.println(hval);
}

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  pinMode(MUX_0, OUTPUT);
  pinMode(MUX_1, OUTPUT);
  pinMode(BT_VCC, OUTPUT);
  pinMode(BT_GND, OUTPUT);
  digitalWrite(MUX_0, LOW);
  digitalWrite(MUX_1, LOW);
  digitalWrite(BT_VCC, HIGH);
  digitalWrite(BT_GND, LOW);
  
    mcp3553_setup();
    dac1.begin(0x60);
    dac2.begin(0x61);
    bme.begin();
    set_sv(DEFAULT_SV);
    set_hv(DEFAULT_HV);
  
}

void loop()
{
  uint8_t cmd = 0, index = 0, data_packet[2];
  bool data_flag = 0;
  while (BTSerial.available() > 0)
  {
    data_packet[index] = BTSerial.read();
    index++;
    data_flag = 1;
  }
  if (data_flag)
  {
    switch (data_packet[0])
    {
      case READ:
        temp = bme.readTemperature();
        pres = bme.readPressure() / 100.0F;
        rh = bme.readHumidity();
        adc = mcp3553_avg();
        
        BTSerial.print(adc);
        BTSerial.print(" ");
        BTSerial.print(temp);
        BTSerial.print(" ");
        BTSerial.println(rh);
        //Serial.println("data");
        break;
      case SET_SV:
        set_sv(data_packet[1]/10.0);
        BTSerial.println("sv");
        break;
      case SET_HV:
        set_hv(data_packet[1]/10.0);
        BTSerial.println("hv");
        break;
    }
  }
  delay(10);
}
