#include <Wire.h>
//#include <math.h> 
/////////////////////// Ver libreria Adafuit_sensors
#define SENSORS_GAUSS_TO_MICROTESLA   100 /**< Gauss to micro-Tesla multiplier */
#define SENSORS_GRAVITY_EARTH         9.80665F /**< Earth's gravity in m/s^2 */
#define SENSORS_DPS_TO_RADS           0.017453293F /**< Degrees/s to rad/s multiplier*/                            \
#define SENSORS_RADS_TO_DPS           57.29577793F /**< Rad/s to degrees/s  multiplier */
///////////////////////////////////////////////////////////////////////////////
#define Factor_Conversion_LSM303D_Mag 0.00008F    /// Sensibilidad para +/- 2 gauss 
#define Factor_Conversion_LSM303D_Acc 0.000061F   /// Sensibilidad para +/- 2 g
#define Factor_Conversion_L3GD20H_Gyr 0.00875F    /// Sensibilidad para +/- 245 dps
//////////////Datos mediante PLX_DAQ
#define Alpha   -2.08285  //   
#define Beta    -1.3371 // 

/////////////////////////////////////LSM303D///////////////////////////////////
#define   LSM303D_SA0_HIGH_ADDRESS   0b0011101 // Pin SAO conectado a Vcc
#define   LSM303D_SA0_LOW_ADDRESS    0b0011110 // Pin SAO conectado a GND
#define   LSM303D_TEMP_OUT_L         0x05 
#define   LSM303D_TEMP_OUT_H         0x06 
#define   LSM303D_STATUS_M           0x07  
#define   LSM303D_OUT_X_L_M          0x08
#define   LSM303D_OUT_X_H_M          0x09
#define   LSM303D_OUT_Y_L_M          0x0A
#define   LSM303D_OUT_Y_H_M          0x0B
#define   LSM303D_OUT_Z_L_M          0x0C
#define   LSM303D_OUT_Z_H_M          0x0D
#define   LSM303D_WHO_AM_I           0x0F 
#define   LSM303D_INT_CTRL_M         0x12 
#define   LSM303D_INT_SRC_M          0x13  
#define   LSM303D_INT_THS_L_M        0x14  
#define   LSM303D_INT_THS_H_M        0x15  
#define   LSM303D_OFFSET_X_L_M       0x16  
#define   LSM303D_OFFSET_X_H_M       0x17 
#define   LSM303D_OFFSET_Y_L_M       0x18 
#define   LSM303D_OFFSET_Y_H_M       0x19 
#define   LSM303D_OFFSET_Z_L_M       0x1A 
#define   LSM303D_OFFSET_Z_H_M       0x1B 
#define   LSM303D_REFERENCE_X        0x1C 
#define   LSM303D_REFERENCE_Y        0x1D 
#define   LSM303D_REFERENCE_Z        0x1E 
#define   LSM303D_CTRL0              0x1F 
#define   LSM303D_CTRL1              0x20 // 0x57 = 0b01010111 AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
#define   LSM303D_CTRL2              0x21 // 0x00 = 0b00000000 AFS = 0 (+/- 2 g full scale)
#define   LSM303D_CTRL3              0x22  
#define   LSM303D_CTRL4              0x23  
#define   LSM303D_CTRL5              0x24 // 0x64 = 0b01100100 M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR) 
#define   LSM303D_CTRL6              0x25 // 0x00 = 0b00100000 MFS = 00 (+/- 2 gauss full scale) 
#define   LSM303D_CTRL7              0x26 // 0x00 = 0b00000000 MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode) 
#define   LSM303D_STATUS_A           0x27  
#define   LSM303D_OUT_X_L_A          0x28
#define   LSM303D_OUT_X_H_A          0x29
#define   LSM303D_OUT_Y_L_A          0x2A
#define   LSM303D_OUT_Y_H_A          0x2B
#define   LSM303D_OUT_Z_L_A          0x2C
#define   LSM303D_OUT_Z_H_A          0x2D
#define   LSM303D_FIFO_CTRL          0x2E  
#define   LSM303D_FIFO_SRC           0x2F  
#define   LSM303D_IG_CFG1            0x30  
#define   LSM303D_IG_SRC1            0x31  
#define   LSM303D_IG_THS1            0x32  
#define   LSM303D_IG_DUR1            0x33 
#define   LSM303D_IG_CFG2            0x34  
#define   LSM303D_IG_SRC2            0x35  
#define   LSM303D_IG_THS2            0x36  
#define   LSM303D_IG_DUR2            0x37  
#define   LSM303D_CLICK_CFG          0x38  
#define   LSM303D_CLICK_SRC          0x39  
#define   LSM303D_CLICK_THS          0x3A  
#define   LSM303D_TIME_LIMIT         0x3B  
#define   LSM303D_TIME_LATENCY       0x3C  
#define   LSM303D_TIME_WINDOW        0x3D  
#define   LSM303D_Act_THS            0x3E  
#define   LSM303D_Act_DUR            0x3F 
///////////////////////////////////////////////////////////////////////////
///////////////// L3GD20H///////////////////////////////////////////////
#define L3GD20H_SA0_HIGH_ADDRESS      0b1101011 // Direccion del esclavo si pin SDO/SAO se encuentra en Vcc (Diagrama esta asi del ZUMO_V1.2)// Es la direccion que se manda
#define L3GD20H_SA0_LOW_ADDRESS       0b1101010 // Direccion del esclavo si pin SDO/SAO se encuentra en GND
#define L3GD20H_WHO_ID               0xD7      // 0b11010111 Esto es el valor del registro WHO_AM_I solo es de tipo lectura,no es relevante
///////////////////////////// Direccion de los Subregistros (SUB)
#define L3GD20H_WHO_AM_I        0x0F
#define L3GD20H_CTRL1           0x20    // 0x6F = 0b01101111 para DR = 01 (200 Hz ODR=0); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (Todos los ejes habilitados) 
#define L3GD20H_CTRL2           0x21
#define L3GD20H_CTRL3           0x22
#define L3GD20H_CTRL4           0x23    // 0x00 = 0b00000000 FS = 00 (+/- 245 Escala del dps)  con sensibilidad de 8.45 mdps/LSB y Zero-rate de +/-25 dps  
#define L3GD20H_CTRL5           0x24
#define L3GD20H_REFERENCE       0x25
#define L3GD20H_OUT_TEMP        0x26
#define L3GD20H_STATUS          0x27
#define L3GD20H_OUT_X_L         0x28
#define L3GD20H_OUT_X_H         0x29
#define L3GD20H_OUT_Y_L         0x2A
#define L3GD20H_OUT_Y_H         0x2B
#define L3GD20H_OUT_Z_L         0x2C
#define L3GD20H_OUT_Z_H         0x2D
#define L3GD20H_FIFO_CTRL       0x2E    // 0x00 Para que el bufer trabaje en modo Bypass 
#define L3GD20H_FIFO_SRC        0x2F 
#define L3GD20H_IG_CFG          0x30
#define L3GD20H_IG_SRC          0x31
#define L3GD20H_IG_THS_XH       0x32
#define L3GD20H_IG_THS_XL       0x33
#define L3GD20H_IG_THS_YH       0x34
#define L3GD20H_IG_THS_YL       0x35
#define L3GD20H_IG_THS_ZH       0x36
#define L3GD20H_IG_THS_ZL       0x37
#define L3GD20H_IG_DURATION     0x38
#define L3GD20H_LOW_ODR         0x39    // Este Registro poner a 0b00000000 o 0x00 para deshabilitar low speed ODR
//////////////////////////////////////////////
////////////////////// Pines del Robot
#define LED     13
#define BUTTON  12
#define M2PWM   10  // MOTOR IZQUIERDO
#define M1PWM   9  // MOTOR IZQUIERDO
#define M2DIR   8  
#define M1DIR   7
#define BUZZER  3
#define BATTERY A1
#define Velocidad_Media 100




//////////////
int16_t Ax,Ay,Az=0;
int16_t Mx,My,Mz=0;
int16_t Gx,Gy,Gz=0;             // Se almacenara los MSB (byte de OUT_X_H) y LSB (byte de OUT_X_L) en una sola variable desplazando el MSB 8 posiciones y sumando bit a bit con LSB 

float C_Ax,C_Ay,C_Az=0;
float C_Mx,C_My,C_Mz=0;
float C_Gx,C_Gy,C_Gz=0;


float pitch,roll,yaw,pitch_last,roll_last=0;
long tiempo_previo=0;
float dt=0;

bool Anti_Rebote=0;
float SetPoint=0; 

///////PID
float dt_PID=0;
long  dt_Last_PID=0;
int16_t  Controlador_PID=0;
float Kp=3.75F;
float Ki=0.0009F;
float Kd=0.00008F;
float Proporcional=0.0F;
float Integral=0.0F;
float Derivativo=0.0F;

float Error_PID=0;

float Error_Last_PID=0;




void Iniciar_I2C()    // Iniciar dispositivos I2C conectados al bus de datos
{
  Escribir_En_Registro(LSM303D_CTRL1,0x57,"LSM303D");
  Escribir_En_Registro(LSM303D_CTRL2,0x00,"LSM303D");///////////// ast prueba 0x02
  Escribir_En_Registro(LSM303D_CTRL5,0x64,"LSM303D");
  Escribir_En_Registro(LSM303D_CTRL6,0x00,"LSM303D");
  Escribir_En_Registro(LSM303D_CTRL7,0x00,"LSM303D");

  Escribir_En_Registro(L3GD20H_LOW_ODR,0x00,"L3GD20H");
  Escribir_En_Registro(L3GD20H_CTRL1,  0x6F,"L3GD20H");
  Escribir_En_Registro(L3GD20H_CTRL4,  0x00,"L3GD20H");
  
}
void Escribir_En_Registro(byte Reg,byte Val,char Dispositivo[10]) // Escribir en registros Enviar datos correctamente
{
  if(Dispositivo=="LSM303D")
  {
    Wire.beginTransmission(LSM303D_SA0_HIGH_ADDRESS); // Direccion del esclado de 7 bits
    Wire.write(Reg);
    Wire.write(Val);
    Wire.endTransmission();
    //Serial.println("Comunicacion exitosa....");
  }
  else if(Dispositivo=="L3GD20H")
  {
    Wire.beginTransmission(L3GD20H_SA0_HIGH_ADDRESS); // Direccion del esclavo de 7bits
    Wire.write(Reg);
    Wire.write(Val);
    Wire.endTransmission();
    //Serial.println("Comunicacion exitosa....");
  }
  else{}//Serial.println("ERror de comunicacion....");}
}

byte Leer_Un_Registro(byte Reg,char Dispositivo[10]) // Escribir en registros Enviar datos correctamente
{
  byte Valor;
  if(Dispositivo=="LSM303D")
  {
    Wire.beginTransmission(LSM303D_SA0_HIGH_ADDRESS); // Direccion del esclado de 7 bits
    Wire.write(Reg);             // Direccion del SUB
    Wire.endTransmission();
    Wire.requestFrom(LSM303D_SA0_HIGH_ADDRESS,(byte)1);
    while(Wire.available()>1){}
    Valor=Wire.read();
    Wire.endTransmission();    
  }
  else if(Dispositivo=="L3GD20H")
  {
    Wire.beginTransmission(L3GD20H_SA0_HIGH_ADDRESS); // Direccion del esclavo de 7bits
    Wire.write(Reg);
    Wire.endTransmission();
    Wire.requestFrom(L3GD20H_SA0_HIGH_ADDRESS,(byte)1);
    while(Wire.available()>1){}
    Valor=Wire.read();
    Wire.endTransmission();
  }
  else{Serial.println("Error de comunicacion....");}
  return Valor ;
}
void Datos_RAW(void)
{
  Wire.beginTransmission(L3GD20H_SA0_HIGH_ADDRESS);
  Wire.write(L3GD20H_OUT_X_L | (1 << 7)); // 0b1 + Direccion del registro (SUB) continua la lectura de datos de FIFO,0b0 + Direccion del registro (SUB) no continua la lectura de datos, 
  Wire.endTransmission();
  Wire.requestFrom(L3GD20H_SA0_HIGH_ADDRESS, (byte)6);  // Solicitamos solo los 6 primeros bytes del registro FIFO y estaran almacenados en wire.read()
  while(Wire.available()>6){} // Ver lo del tiempo para hacer un break
  uint8_t xlg = Wire.read();  // Tener en cuenta el tipo de variable ya que si este es de 16 bit(int) tomara esa misma cantidad de bits del bufer
  uint8_t xhg = Wire.read();  // Al obtener 8 bits del buffer este se desplaza 8 posiciones por tanto los que datos nuevos seran del nuevo registro
  uint8_t ylg = Wire.read();  // Tener cuidado con el orden que se reciben leer data sheet ya que algunas posiciones talvez cambien
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();
  Wire.endTransmission();

  Wire.beginTransmission(LSM303D_SA0_HIGH_ADDRESS);
  Wire.write(LSM303D_OUT_X_L_M|(1<<7));
  Wire.endTransmission();
  Wire.requestFrom(LSM303D_SA0_HIGH_ADDRESS,(byte)6);
  while (Wire.available() < 6) {}   /// Incluir un tiempo por si no pasa nada salirse del while
  uint8_t M_x_l=Wire.read();
  uint8_t M_x_h=Wire.read();
  uint8_t M_y_l=Wire.read();
  uint8_t M_y_h=Wire.read();
  uint8_t M_z_l=Wire.read();
  uint8_t M_z_h=Wire.read();
  Wire.endTransmission();
  
  Wire.beginTransmission(LSM303D_SA0_HIGH_ADDRESS);
  Wire.write(LSM303D_OUT_X_L_A|(1<<7));
  Wire.endTransmission();
  Wire.requestFrom(LSM303D_SA0_HIGH_ADDRESS,(byte)6);
  while (Wire.available() < 6) {}
  uint8_t A_x_l=Wire.read();
  uint8_t A_x_h=Wire.read();
  uint8_t A_y_l=Wire.read();
  uint8_t A_y_h=Wire.read();
  uint8_t A_z_l=Wire.read();
  uint8_t A_z_h=Wire.read();
  Wire.endTransmission();
  
  Ax=(int16_t)((A_x_h<<8)|A_x_l);
  Ay=(int16_t)((A_y_h<<8)|A_y_l);
  Az=(int16_t)((A_z_h<<8)|A_z_l);
  
  Gx = (int16_t)(xhg << 8 | xlg); // Combinamos los MSB=OUT_X_H con LSB=OUT_X_L
  Gy = (int16_t)(yhg << 8 | ylg); // Se desplaza 8 posiciones a la derecha el MSB y se hace un or con LSB par ano alterar los bits que ya estan ahy
  Gz = (int16_t)(zhg << 8 | zlg);
  
  Mx=(int16_t)((M_x_h<<8)|M_x_l);
  My=(int16_t)((M_y_h<<8)|M_y_l);
  Mz=(int16_t)((M_z_h<<8)|M_z_l);

  C_Ax=Ax*Factor_Conversion_LSM303D_Acc*SENSORS_GRAVITY_EARTH;
  C_Ay=Ay*Factor_Conversion_LSM303D_Acc*SENSORS_GRAVITY_EARTH;
  C_Az=Az*Factor_Conversion_LSM303D_Acc*SENSORS_GRAVITY_EARTH;

  C_Mx=Mx*Factor_Conversion_LSM303D_Mag-Alpha;//*SENSORS_GAUSS_TO_MICROTESLA;
  C_My=My*Factor_Conversion_LSM303D_Mag-Beta;//*SENSORS_GAUSS_TO_MICROTESLA;
  C_Mz=Mz*Factor_Conversion_LSM303D_Mag;//*SENSORS_GAUSS_TO_MICROTESLA;

  C_Gx=Gx*Factor_Conversion_L3GD20H_Gyr;
  C_Gy=Gy*Factor_Conversion_L3GD20H_Gyr;
  C_Gz=Gz*Factor_Conversion_L3GD20H_Gyr;

  //Imprimir_Datos(C_Ax,C_Ay,C_Az,C_Mx,C_My,C_Mz,C_Gx,C_Gy,C_Gz);
}
void Imprimir_Datos(float Ax,float Ay,float Az,float Gx,float Gy,float Gz,float Mx,float My,float Mz)
{
  Serial.print("  Ax: ");
  Serial.print(Ax);
  Serial.print("  Ay: ");
  Serial.print(Ay);
  Serial.print("  Az: ");
  Serial.print(Az);
  Serial.print("  Gx: ");
  Serial.print(Gx);
  Serial.print("  Gy: ");
  Serial.print(Gy);
  Serial.print("  Gz: ");
  Serial.print(Gz);
  Serial.print("  Mx: ");  
  Serial.print(Mx);
  Serial.print("  My: ");
  Serial.print(My);
  Serial.print("  Mz: ");
  Serial.println(Mz);
  
}
void Orientacion(void)
{
  Datos_RAW();
  float Angulo_Aceleracion_Y;
  float Angulo_Aceleracion_X;
  dt = (millis() - tiempo_previo) / 1000.0;
  tiempo_previo = millis();
  //Calcular los ángulos con acelerometro
  Angulo_Aceleracion_X=(float)atan(C_Ay/sqrt(pow(C_Ax,2)+pow(C_Az,2)))*(180.0/PI);
  Angulo_Aceleracion_Y=(float)atan(-1*(C_Ax/sqrt(pow(C_Ay,2)+pow(C_Az,2))))*(180.0/PI);
  
  roll=0.98*(roll+C_Gx*dt)+0.02*Angulo_Aceleracion_X;
  pitch=0.98*(pitch+C_Gy*dt)+0.02*Angulo_Aceleracion_Y;
  yaw=(float)atan2(C_My,C_Mx)*(180.0/PI);
  if(yaw <0)
  {
    yaw+=360;
  }
  /*
  Serial.print("  roll: ");
  Serial.print(roll);
  Serial.print("  pitch: ");
  Serial.print(pitch);
  Serial.print("  yaw: ");
  Serial.println(yaw);
  */

}
void Iniciar_PLX_DAQ(void)
{
  ///////////////////////Parallax
  Serial.println("CLEARDATA"); //limpia los datos previos 
  Serial.println("LABEL,Time,Datos1,Datos2,Datos3"); 
  //siempre se escribe LABEL, puesto que excel reconoce
  // los siguientes textos como las nombres de las columnas 
  // (La columna tiempo puede dejarse así)
  Serial.println("RESETTIMER"); // pone el temporizador en 0
  ///////////////////////
}
int x=0;
void Imprimir_PLX_DAQ(int *Muestras)
{
  Datos_RAW();
  char P_Mx[32],P_My[32],P_Mz[32];
  // en este espacio van las operaciones del dato1 y el dato 2
  // (ej. Lectura de entrada analógica, entre otras)
  dtostrf(C_Mx, 10 , 4 , P_Mx); // se convierte a carácter
  dtostrf(C_My, 10 , 4 , P_My);
  dtostrf(C_Mz, 10 , 4 , P_Mz);
  Serial.print("DATA,TIME,"); 
  //escribe el tiempo en la columna A y el tiempo en segundos 
  // desde la primera medida en la columna B
  Serial.print(P_Mx);      Serial.print(",");
  Serial.print(P_My);      Serial.print(",");
  Serial.println(P_Mz);
  // No olvide adicionar println al ultimo comando para pasar de fila  
  if(*Muestras<x)
  {
    while(1){} 
  }
}
void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  Iniciar_I2C(); 
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  TCCR1B = TCCR1B & B11111000 | B00000001;   // frecuencia a 33kHZ
  pinMode(BUTTON,INPUT_PULLUP);
  pinMode(LED,OUTPUT);
  pinMode(M2DIR,OUTPUT);
  pinMode(M1DIR,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  Motores(0,0,0,0);
 
  while(1)
  {
    Orientacion();
    if(digitalRead(BUTTON)==0)
    {
      SetPoint=yaw;
      Serial.print("SetPoint: ");
      Serial.println(SetPoint);
      break;
    }
  }
  delay(3000);
  //Iniciar_PLX_DAQ();
}
void Motores(bool I,unsigned char I_PWM,unsigned char D_PWM,bool D)
{
  digitalWrite(M1DIR,D);
  analogWrite(M1PWM,D_PWM);
  digitalWrite(M2DIR,I);
  analogWrite(M2PWM,I_PWM);
}
void loop()
{
//Imprimir_PLX_DAQ(500);  
  Motores(0,0,0,0);    
  Serial.println("Pulsar Boton .......");
  if(digitalRead(12)==0)
  {
    delay(2000);
    while(1)
    {
      Orientacion();
      dt_PID=(millis()-dt_Last_PID)/1000.0;
      dt_Last_PID=millis();
      Error_PID=SetPoint-yaw;
      Proporcional=Error_PID;
      Integral=Error_PID*dt_PID+Integral;
      if(Integral>Velocidad_Media)
      {
        Integral=Velocidad_Media;
      }
      else if(Integral<(-1*Velocidad_Media))
      {
        Integral=-1*Velocidad_Media;
      }
      Derivativo=(Error_PID-Error_Last_PID)/dt_PID;
      Error_Last_PID=Error_PID;
      Controlador_PID=(int16_t)(Proporcional*Kp+Integral*Ki+Derivativo*Kd);

      if(Controlador_PID>=Velocidad_Media)
      {
        Controlador_PID=Velocidad_Media;
      }
      else if(Controlador_PID<(-1*Velocidad_Media))
      {
        Controlador_PID=-1*Velocidad_Media;
      }
      if(Controlador_PID<0)
      {
        Motores(1,15+abs(Controlador_PID),15+abs(Controlador_PID),0);
      }
      else if(Controlador_PID>0)
      {
        Motores(0,15+abs(Controlador_PID),15+abs(Controlador_PID),1);
      }
      Serial.print("Error: ");Serial.print(Error_PID);
      Serial.print(" P: ");Serial.print(Proporcional);Serial.print(" I: ");Serial.print(Integral);Serial.print(" D: ");Serial.print(Derivativo);
      Serial.print(" PID: ");Serial.print(Controlador_PID);Serial.print(" dt: ");Serial.println(dt_PID);
      

      if(digitalRead(BUTTON)==0)
      {
        Serial.println("Saliendo......");
        delay(1000);
        break;
      }
    }  
  }

}
