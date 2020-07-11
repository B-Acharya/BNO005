
#include <Wire.h>
/*operation modes of BNO055 values that can be written in OPR_MODE*/
#define BNO055_OPERATION_MODE_CONFIG 0x00
#define BNO055_OPERATION_MODE_ACCONLY 0x01
#define BNO055_OPERATION_MODE_MAGONLY 0x02
#define BNO055_OPERATION_MODE_GYRONLY 0x03
#define BNO055_OPERATION_MODE_ACCMAG 0x04
#define BNO055_OPERATION_MODE_ACCGYRO 0x05
#define BNO055_OPERATION_MODE_MAGGYRO 0x06
#define OPERATION_MODE_ANY_MOTION 0x07
#define BNO055_OPERATION_MODE_IMUPLUS 0x08
#define BNO055_OPERATION_MODE_COMPASS 0x09
#define BNO055_OPERATION_MODE_M4G 0x0A
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF 0x0B
#define BNO055_OPERATION_MODE_NDOF 0x0C

/*Queternion regsister access*/
#define BNO055_QUATERNION_DATA_W_LSB_ADDR   (0X20)
#define BNO055_QUATERNION_DATA_W_MSB_ADDR   (0X21)
#define BNO055_QUATERNION_DATA_X_LSB_ADDR   (0X22)
#define BNO055_QUATERNION_DATA_X_MSB_ADDR   (0X23)
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR   (0X24)
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR   (0X25)
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR   (0X26)
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR   (0X27)

/*Array indec for reading data  */
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_LSB   (0)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_W_MSB   (1)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_LSB   (2)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_X_MSB   (3)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_LSB   (4)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Y_MSB   (5)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_LSB   (6)
#define BNO055_SENSOR_DATA_QUATERNION_WXYZ_Z_MSB   (7)


/* Accel data register*/
#define BNO055_ACCEL_DATA_X_LSB_ADDR        (0X08)
#define BNO055_ACCEL_DATA_X_MSB_ADDR        (0X09)
#define BNO055_ACCEL_DATA_Y_LSB_ADDR        (0X0A)
#define BNO055_ACCEL_DATA_Y_MSB_ADDR        (0X0B)
#define BNO055_ACCEL_DATA_Z_LSB_ADDR        (0X0C)
#define BNO055_ACCEL_DATA_Z_MSB_ADDR        (0X0D)

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR          (0X0E)
#define BNO055_MAG_DATA_X_MSB_ADDR          (0X0F)
#define BNO055_MAG_DATA_Y_LSB_ADDR          (0X10)
#define BNO055_MAG_DATA_Y_MSB_ADDR          (0X11)
#define BNO055_MAG_DATA_Z_LSB_ADDR          (0X12)
#define BNO055_MAG_DATA_Z_MSB_ADDR          (0X13)

/*Gyro data registers*/
#define BNO055_GYRO_DATA_X_LSB_ADDR         (0X14)
#define BNO055_GYRO_DATA_X_MSB_ADDR         (0X15)
#define BNO055_GYRO_DATA_Y_LSB_ADDR         (0X16)
#define BNO055_GYRO_DATA_Y_MSB_ADDR         (0X17)
#define BNO055_GYRO_DATA_Z_LSB_ADDR         (0X18)
#define BNO055_GYRO_DATA_Z_MSB_ADDR         (0X19)

#define BNO055_SYS_TRIGGER_ADDR 0X3F
#define BNO055_CHIP_ID_ADDR 0X00
#define BNO055_PAGE_ID_ADDR 0X07
#define BNO055_ID (0xA0)

#define OPR_MODE 0x3D // Opereation mode register 

/*i2c Slave address*/
#define BNO055_I2C_ADDR 0x29 //Primary address when COM3 is HIGH

struct quaternion{
  double w;
  double x;
  double y;
  double z;
};

void setup() {
  Serial.begin(9600);   
  Serial.println("setup begin");
  Wire.begin();
/* Make sure we have the right device */
  uint8_t id = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_CHIP_ID_ADDR);
  if (id != BNO055_ID) {
    delay(1000); // hold on for boot
    id = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
      return false; // still not? ok bail
    }
  }

  
// put your setup code here, to run once:
// BNO055_SET_OPERATION_MODE(BNO055_I2C_ADDR, BNO055_OPERATION_MODE_NDOF); // Setting the BNO055 to NDOF mode (Fusion)
  BNO055_SET_OPERATION_MODE(BNO055_I2C_ADDR, BNO055_OPERATION_MODE_CONFIG);
  BNO055_WRITE_BYTE(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 0x20);
  delay(30);
    while (BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
      delay(10);
      Serial.println("Reading device address");
    }
  delay(50);
  BNO055_SET_OPERATION_MODE(BNO055_I2C_ADDR, BNO055_OPERATION_MODE_IMUPLUS); // setting the BNo055 to AMG mode 
  delay(10);
  BNO055_WRITE_BYTE(BNO055_I2C_ADDR, BNO055_PAGE_ID_ADDR, 0);
  Serial.println("setup end");
}
void loop() {
  Serial.println("Loop begins"); 
  // put your main code here, to run repeatedly:
//  uint16_t data[8] ;
//  uint16_t acc[6];
//  uint8_t quternion[4] ;
//  BNO055_GET_QUATERNION(&data[0]);
//  MSB_LSB(&quternion[0], &data[0]);
//  BNO055_GET_ACC(&acc[0]);
  struct quaternion q;
  q = BNO055_GET_QUATERNION();
  delay(500);
  Serial.println(q.w);
  Serial.println(q.x);
  Serial.println(q.y);
  Serial.println(q.z);
}


void i2c_init()
{
  Wire.begin();   
}

void i2c_write8(uint8_t data)
{
    Wire.write(data);
}

uint8_t i2c_read8(uint8_t address)
{
    Wire.requestFrom(address, 1);
    return Wire.read();
}

void BNO055_WRITE_BYTE(uint8_t i2c_slave_address, uint16_t register_address, uint8_t data) 
{
    Wire.beginTransmission(i2c_slave_address);
    i2c_write8(register_address);
    i2c_write8(data);
    Wire.endTransmission();
}

uint8_t BNO055_READ_BYTE(uint8_t i2c_slave_address, uint16_t register_address) 
{
    Wire.beginTransmission(i2c_slave_address);
    i2c_write8(register_address);
    Wire.endTransmission();
    return i2c_read8(i2c_slave_address);
}

void BNO055_SET_OPERATION_MODE(uint8_t address, uint8_t op_mode)
{
   BNO055_WRITE_BYTE(address, OPR_MODE, op_mode);
}

//void BNO055_GET_QUATERNION(uint16_t *data){
//  uint16_t QUA_BUFFER[8]; 
//  if(!data)
//    return;
//  Serial.println("Reading registers");
//  QUA_BUFFER[0] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_W_LSB_ADDR );
//  QUA_BUFFER[1] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_W_MSB_ADDR );
//  QUA_BUFFER[2] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_X_LSB_ADDR );
//  QUA_BUFFER[3] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_X_MSB_ADDR );
//  QUA_BUFFER[4] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_Y_LSB_ADDR );
//  QUA_BUFFER[5] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_Y_MSB_ADDR );
//  QUA_BUFFER[6] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_Z_LSB_ADDR );
//  QUA_BUFFER[7] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_Z_MSB_ADDR );
//  Serial.println("Reading Registers Complete");
//
//  int j = 0;
//  for(int i=0; i<6 ;i+=2){
//    data[j] = ((QUA_BUFFER[i+1]<<8)|(QUA_BUFFER[i]&0xFF));
//    j++;
//  }
//}
void readlen(uint8_t address, byte *buffer_1, uint8_t len){
  Wire.beginTransmission(BNO055_I2C_ADDR);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_I2C_ADDR, len );
  for (uint8_t i=0; i<len ; i++){
    buffer_1[i] = Wire.read();
    Serial.println(buffer_1[i]);
  }
}


struct quaternion BNO055_GET_QUATERNION(){
  uint8_t buffer_qua[8];
  memset(buffer_qua, 0, 8);
  int16_t x=0, y=0, z=0, w=0;
  readlen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer_qua, 8);
  Serial.print(buffer_qua[0]);
  w = (((uint16_t)buffer_qua[1]) << 8) | ((uint16_t)buffer_qua[0]);
  x = (((uint16_t)buffer_qua[3]) << 8) | ((uint16_t)buffer_qua[2]);
  y = (((uint16_t)buffer_qua[5]) << 8) | ((uint16_t)buffer_qua[4]);
  z = (((uint16_t)buffer_qua[7]) << 8) | ((uint16_t)buffer_qua[6]);
//  const double scale = (1.0/(1<<14));
  const double scale = 1;
  struct quaternion q ;
  q = { scale * w,  scale*x,  scale*y,  scale*z};
  return q;
  }
void BNO055_GET_ACC(uint16_t *data){
  if(!data){
    return;
  }
  uint8_t ACC_BUFFER[6]; 
  ACC_BUFFER[0] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_X_LSB_ADDR );
  ACC_BUFFER[1] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_X_MSB_ADDR );
  ACC_BUFFER[2] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_Y_LSB_ADDR );
  ACC_BUFFER[3] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_Y_MSB_ADDR );
  ACC_BUFFER[4] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_Z_LSB_ADDR );
  ACC_BUFFER[5] = BNO055_READ_BYTE(BNO055_I2C_ADDR, BNO055_ACCEL_DATA_Z_MSB_ADDR );

  int j = 0;
  for(int i=0; i<6 ;i+=2){
    data[j] = ((ACC_BUFFER[i+1]<<8)|(ACC_BUFFER[i]&0xFF));
    j++;
  }
}
void MSB_LSB(uint8_t *p, uint8_t *q)
{
  int j = 0;
  for(int i=0; i<8 ;i+=2)
  {
    p[j] = ((q[i]<<8)|q[i+1]&0xFF);
    j++;
  }
}
