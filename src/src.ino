#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <WC_ICM42688.h>
#include <event_mgr.h>
#include <float.h>
#include <math.hpp>
#include "SPI.h"
#include <Preferences.h>

bool  PRINT_ACCEL_CORRECT = 0;
float IMU_CALI_TH = 0.05f;
unsigned long print_timer = 0;
Preferences pref;
TaskHandle_t *imu_handle;
TaskHandle_t *communicate_handle;
void ImuTask(void *pvParameter);
void CommunicateTask(void *pvParameter);

struct accel_t {
  float x;
  float y;
  float z;
};

struct gyro_t {
  float x;
  float y;
  float z;
};

struct euler_angles_t {
  float roll;
  float pitch;
  float yaw;
};

struct to_master_t {
  accel_t accel;
  gyro_t gyro;
  euler_angles_t angle;
  float temperature = 30;
  int8_t direction;
};

class Imu {
 private:
  matrix::Vector3f accel_final_v;
  struct accel_t accel_read;
  struct gyro_t gyro;
  struct euler_angles_t euler;
  bool calibrated_;
  float temperature_{NAN};
  int8_t gravity_direction{0};
  Madgwick algorithm;

  matrix::Vector3f scale_{1.0f, 1.0f, 1.0f};
  matrix::Vector3f offset_{0.0f, 0.0f, 0.0f};
  int16_t last_accel_sample[3]{};

 public:
  float accel_cali[6][3];
  bool  cali_flag[6];
  bool  cali_;
  uint8_t cali_step = 0;
  uint8_t cali_status = 0;
  struct to_master_t imu_to_master;
  void InitICM42688(void);
  void UpdateICM42688(void);
  void ProcessAccel();
  void UpdateEulerAngles(void);
  void ReadAccelAvg();
  void DoAccelCalibrationQuick();
  void DoAccelCalibrationFull();
  struct accel_t *get_accel_read_point() { return &accel_read; };
  void ParametersSave();
  int32_t sum(const int16_t samples[], uint8_t len);

  bool set_scale(const matrix::Vector3f scale);
  bool set_offset(const matrix::Vector3f offset);
  float get_roll() { return algorithm.getRoll(); };
  float get_pitch() { return algorithm.getPitch(); };
  float get_yaw() { return algorithm.getYaw(); };
  matrix::Vector3f get_accel() {return accel_final_v;};
  int8_t get_gravity_direction() { return gravity_direction; };
};

WC_ICM42688_SPI ICM42688(6);
Imu imu;
// static publisher_t imu_pub;
// SENSOR: ATTITUDE
#define IMU_MSG 1
#define IMU_MSG_LEN sizeof(struct sensor_to_com_t)

#define SENSOR_TASK_PERIOD 1
#define COMMUNICATE_TASK_PERIOD 50
#define LED_PIN 8
#define IMU_PIN 12
#define IMU_SPI_SCK 7
#define IMU_SPI_MISO 10
#define IMU_SPI_MOSI 3
#define IMU_SPI_SS 6

void Imu::InitICM42688() {
  pinMode(IMU_PIN, OUTPUT);
  digitalWrite(IMU_PIN, LOW);
  delay(1000);  // Wait for IMU bootup

  SPI.begin(IMU_SPI_SCK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_SS);

  int status;
  while ((status = ICM42688.begin()) != 0) {
    if (status == -1) {
      Serial.println("bus data access error");
    } else
      Serial.println("Chip versions do not match");
    delay(1000);
  }
  Serial.println("ICM42688 begin success!!!");

  // 设置传感器的输出数据率（ODR）和满量程范围（FSR）
  ICM42688.setODRAndFSR(GYRO, ODR_1KHZ, FSR_3);
  ICM42688.setODRAndFSR(ACCEL, ODR_1KHZ, FSR_3);
  // 启动温度测量
  ICM42688.startTempMeasure();
  // 启动陀螺仪测量:使用低噪声模式进行测量
  ICM42688.startGyroMeasure(LN_MODE);
  // 启动加速度计测量:使用低噪声模式进行测量
  ICM42688.startAccelMeasure(LN_MODE);

  delay(1000);  // Wait for sensor to stabilize
}

void Imu::UpdateICM42688() {
  double ICM42688Temp;
  if ((ICM42688Temp = ICM42688.getTemperature()) != 0) {
    temperature_ = ICM42688Temp;
  }
  if ((ICM42688Temp = ICM42688.getAccelDataX()) != 0) {
    accel_read.x = ICM42688Temp;
  }
  if ((ICM42688Temp = ICM42688.getAccelDataY()) != 0) {
    accel_read.y = ICM42688Temp;
  }
  if ((ICM42688Temp = ICM42688.getAccelDataZ()) != 0) {
    accel_read.z = ICM42688Temp;
  }
  if ((ICM42688Temp = ICM42688.getGyroDataX()) != 0) {
    gyro.x = ICM42688Temp;
  }
  if ((ICM42688Temp = ICM42688.getGyroDataY()) != 0) {
    gyro.y = ICM42688Temp;
  }
  if ((ICM42688Temp = ICM42688.getGyroDataZ()) != 0) {
    gyro.z = ICM42688Temp;
  }

  // if(accel_read.x >  8194){gravity_direction = 0;return;}
  // if(accel_read.y >  8194){gravity_direction = 1;return;}
  // if(accel_read.z >  8194){gravity_direction = 2;return;}
  // if(accel_read.x < -8194){gravity_direction = 3;return;}
  // if(accel_read.y < -8194){gravity_direction = 4;return;}
  // if(accel_read.z < -8194){gravity_direction = 5;return;}
}

void Imu::ProcessAccel() {
  matrix::Vector3f accel_read_v{accel_read.x/1670.6011,accel_read.y/1670.6011,accel_read.z/1670.6011};
  // matrix::Vector3f accel_read_v{accel_read.x,accel_read.y,accel_read.z};
  matrix::Vector3f accel_correct_v{(accel_read_v - offset_).emult(scale_)};
  accel_final_v = accel_correct_v;

  if(PRINT_ACCEL_CORRECT)
  {
    if(millis() - print_timer > 1000){
    // 将 Vector3f 转换为字符串
    String vectorStr = "(";
    vectorStr += String(accel_read_v(0), 4); 
    vectorStr += ", ";
    vectorStr += String(accel_read_v(1), 4); 
    vectorStr += ", ";
    vectorStr += String(accel_read_v(2), 4);
    vectorStr += ") ";
    vectorStr += "(";
    vectorStr += String(accel_correct_v(0), 4); 
    vectorStr += ", ";
    vectorStr += String(accel_correct_v(1), 4); 
    vectorStr += ", ";
    vectorStr += String(accel_correct_v(2), 4);
    vectorStr += ")";
    Serial.println(vectorStr);
    print_timer = millis();
    }
  }
}

void Imu::UpdateEulerAngles() {
  algorithm.updateIMU(gyro.x, gyro.y, gyro.z, accel_final_v(0), accel_final_v(1),
                      accel_final_v(2));
}

#include "math.hpp"
enum detect_orientation_return {
  ORIENTATION_TOP_DOWN,
  ORIENTATION_BOTTOM_DOWN,
  ORIENTATION_LEFT_DOWN,
  ORIENTATION_RIGHT_DOWN,
  ORIENTATION_BACK_DOWN,
  ORIENTATION_FRONT_DOWN,
  ORIENTATION_ERROR
};

bool Imu::set_offset(const matrix::Vector3f offset) {
  // if (matrix::Vector3f(offset_ - offset).longerThan(0.01f)) {
  //   if (offset.isAllFinite()) {
    offset_ = offset;
    // 将 Vector3f 转换为字符串
    String vectorStr = "offset_: (";
    vectorStr += String(offset_(0), 4); 
    vectorStr += ", ";
    vectorStr += String(offset_(1), 4); 
    vectorStr += ", ";
    vectorStr += String(offset_(2), 4);
    vectorStr += ")";
    Serial.println(vectorStr);
  //     // _calibration_count++;
  return true;
  //   }
  // }
  // return false;
}

bool Imu::set_scale(const matrix::Vector3f scale) {
  // if (matrix::Vector3f(scale_ - scale).longerThan(0.01f)) {
  //   if (scale.isAllFinite() && (scale(0) > 0.f) && (scale(1) > 0.f) &&
  //       (scale(2) > 0.f)) {
    scale_ = scale;
        // 将 Matrix3f 转换为字符串
    String vectorStr = "scale_: (";
    vectorStr += String(scale_(0), 4); 
    vectorStr += ", ";
    vectorStr += String(scale_(1), 4); 
    vectorStr += ", ";
    vectorStr += String(scale_(2), 4);
    vectorStr += ")";
    Serial.println(vectorStr);
      // _calibration_count++;
  return true;
  //   }
  // }

  // return false;
}

// static constexpr float CONSTANTS_ONE_G = 9.80665f;
static constexpr float CONSTANTS_ONE_G = 16383;
static constexpr unsigned detect_orientation_side_count = 6;
float accel_raw_ref[detect_orientation_side_count][3]{};

void Imu::ReadAccelAvg() {

  // check cali_step is right[1-6]
  if(cali_step < 1  || cali_step > 6){
    Serial.printf("[ReadAccelAvg]ERROR:cali_step = %d\n",cali_step);
    return;
  }

  // check if cali already
  if(cali_flag[cali_step - 1] == 1){return;}

  //check if the IMU is placed along the required diection
    Serial.printf("[ReadAccelAvg]ERROR:gravity_direction[%d] != cali_step[%d]\n",gravity_direction,cali_step);
  if(gravity_direction != cali_step - 1){
    return;
  }

  int num = 750;
  int counts = 0;
  matrix::Vector3f raw;
  matrix::Vector3f offset{0, 0, 0};
  matrix::Vector3f sum;
  matrix::Vector3f start;

  while (counts < num) {

    imu.UpdateICM42688();
    matrix::Vector3f accel_raw_v{accel_read.x/1670.6011,accel_read.y/1670.6011,accel_read.z/1670.6011};

    if(counts == 0){
      start = accel_raw_v;
      return;
    }

    raw = accel_raw_v;
    if ((raw - start).norm() > IMU_CALI_TH) {
      Serial.printf("[ReadAccelAvg] > norm:%f\n",(raw - start).norm());
      counts = 1;
      start = raw;
      sum.setZero();
      sum += raw;
      return;
    }
    sum += raw;
    counts++;
  }

  const matrix::Vector3f avg{sum / counts};
  avg.copyTo(accel_cali[cali_step - 1]);
  Serial.print("[AccelResult]");
  for (int j = 0; j < 3; ++j) {
    Serial.print(String(accel_cali[cali_step - 1][j],4));
    Serial.print(",");
  }
  Serial.println("\n"); // 换行
  // 复位，等待管理员调换姿势按下按键
  Serial.printf("[ReadAccelAvg]cali_step:%d complete\n",cali_step);
  counts = 0;
  sum.setZero();
  cali_flag[cali_step - 1] = 1;
}

void Imu::DoAccelCalibrationFull() {
  /*=== 计算出 offset ===*/
  // X offset: average X from accel_top_down + accel_bottom_down
  matrix::Vector3f offset_v;
  const matrix::Vector3f accel_top_down{accel_cali[ORIENTATION_TOP_DOWN]};
  const matrix::Vector3f accel_bottom_down{accel_cali[ORIENTATION_BOTTOM_DOWN]};
  offset_v(0) = (accel_top_down(0) + accel_bottom_down(0)) * 0.5f;

  // Y offset: average Y from accel_left + accel_right
  const matrix::Vector3f accel_left_down{accel_cali[ORIENTATION_LEFT_DOWN]};
  const matrix::Vector3f accel_right_down{accel_cali[ORIENTATION_RIGHT_DOWN]};
  offset_v(1) = (accel_left_down(1) + accel_right_down(1)) * 0.5f;

  // Z offset: average Z from accel_back_down + accel_front_down
  const matrix::Vector3f accel_back_down{accel_cali[ORIENTATION_BACK_DOWN]};
  const matrix::Vector3f accel_front_down{accel_cali[ORIENTATION_FRONT_DOWN]};
  offset_v(2) = (accel_back_down(2) + accel_front_down(2)) * 0.5f;

  /*=== 计算出 accel_T ===*/
  matrix::Matrix3f mat_A;
  mat_A.row(0) = accel_top_down - offset_v;
  mat_A.row(1) = accel_left_down - offset_v;
  mat_A.row(2) = accel_back_down - offset_v;

  // calculate inverse matrix for A: simplify matrices mult because b has only
  // one non-zero element == g at index i
  const matrix::Matrix3f accel_T = mat_A.I() * CONSTANTS_ONE_G;

  /*=== 设置 offset 和 scale*/
  set_offset(offset_v);
  set_scale(accel_T.diag());

  while (!pref.begin("imu_cali", false)) {
    Serial.println("[DoAccelCalibrationFull]imu_cali Put Fail");
  }
  pref.putFloat("offset0", offset_(0));
  pref.putFloat("offset1", offset_(1));
  pref.putFloat("offset2", offset_(2));
  pref.putFloat("scale0", scale_(0));
  pref.putFloat("scale1", scale_(1));
  pref.putFloat("scale2", scale_(2));
  pref.end();

  // 将 Matrix3f 转换为字符串
  String matrixStr = "";
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      matrixStr += String(accel_T(i, j), 4); 
      matrixStr += "\t";
    }
    matrixStr += "\n";
  }
  Serial.println("[accel_T]");
  Serial.println(matrixStr);

  // 将 Vector3f 转换为字符串
  String vectorStr = "offset_v: (";
  vectorStr += String(offset_v(0), 4); 
  vectorStr += ", ";
  vectorStr += String(offset_v(1), 4); 
  vectorStr += ", ";
  vectorStr += String(offset_v(2), 4);
  vectorStr += ")";
  Serial.println(vectorStr);
}
void Imu::ParametersSave() {}

// matrix::Vector3f accels_raw;
// void Imu::DoAccelCalibrationQuick() {
//   const matrix::Vector3f accel_offset{accels_raw - offset_};
//   if (counts > 0) {
//     const matrix::Vector3f diff{accel_offset - (accel_sum / counts)};
//     if (diff.norm() < 1.f) {
//       accel_sum += matrix::Vector3f{accels_raw - offset_};
//       counts++;
//     }
//   } else {
//     accel_sum = accels_raw;
//     counts = 1;
//   }

//   if ((counts > 0)) {
//     const matrix::Vector3f accel_avg = accel_sum / counts;
//     const matrix::Quatf q;
//     const matrix::Vector3f accel_ref =
//         q.rotateVectorInverse(matrix::Vector3f{0.f, -1000, 0});
//     const float angle =
//         matrix::AxisAnglef(matrix::Quatf(accel_avg, accel_ref)).angle();

//     if (angle <= radians(10.f)) {
//       offset_ = accel_avg - accel_ref;
//       calibrated_ = true;
//     }

//     if (!calibrated_) {
//       // otherwise simply normalize to gravity and remove offset
//       matrix::Vector3f accel{accel_avg};
//       accel.normalize();
//       accel = accel * CONSTANTS_ONE_G;

//       offset_ = accel_avg - accel;
//       calibrated_ = true;
//     }

//     ParametersSave();
//   }
// }

int32_t Imu::sum(const int16_t samples[], uint8_t len) {
  int32_t sum = 0;
  for (int n = 0; n < len; n++) {
    sum += samples[n];
  }
  return sum;
}
void SendTOMaster(to_master_t *to_master) {

  String dataString;
  dataString = "";
  dataString += "<";
  dataString += String(to_master->angle.roll, 4);
  dataString += ",";
  dataString += String(to_master->angle.pitch, 4);
  dataString += ",";
  dataString += String(to_master->angle.yaw, 4);
  dataString += ",";
  dataString += String(to_master->angle.roll, 4);
  dataString += ",";
  dataString += String(to_master->angle.pitch, 4);
  dataString += ",";
  dataString += String(to_master->angle.yaw, 4);
  dataString += ",";
  dataString += String(to_master->accel.x, 4);
  dataString += ",";
  dataString += String(to_master->accel.y, 4);
  dataString += ",";
  dataString += String(to_master->accel.z, 4);
  dataString += ",";
  dataString += String(to_master->temperature, 4);
  dataString += ",";
  dataString += String(to_master->direction, 4);
  dataString += ">";
  Serial1.print(dataString);
}

void PrintSerialMonitor(to_master_t *to_master) {
  // matrix::Vector3f accel_to_print = imu.get_accel();
  // String dataString;
  // dataString = "[accel]";
  // dataString += "<";
  // dataString += String(accel_to_print(0), 4);
  // dataString += ",";
  // dataString += String(accel_to_print(1), 4);
  // dataString += ",";
  // dataString += String(accel_to_print(2), 4);
  // dataString += ",";
  // dataString += String(ICM42688._accelRange, 3);
  // dataString += ">\n";
  // Serial.print(dataString);
  // // String dataString;
  // dataString = "[euler]";
  // dataString += "<";
  // dataString += String(to_master->angle.roll, 4);
  // dataString += ",";
  // dataString += String(to_master->angle.pitch, 4);
  // dataString += ",";
  // dataString += String(to_master->angle.yaw, 4);
  // dataString += ">\n";
  // Serial.print(dataString);
}

char rx_buffer[128];
const int rx_max_num = 128;
bool rx_data_flag = 0;
void readSerialData(unsigned char rc) {
  static boolean rx_progress = false;//是否正在接收数据
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';

  // 已经开始接收数据，那么它会判断当前接收到的字符是否为结束标记
  if (rx_progress == true) {
    if (rc != endMarker) {
      rx_buffer[ndx] = rc;
      ndx++;
      if (ndx >= rx_max_num) {
        ndx = rx_max_num - 1;
      }
    } else {
      rx_buffer[ndx] = '\0';  // terminate the string
      rx_progress = false;
      ndx = 0;
      rx_data_flag = true;
    }
  } else if (rc == startMarker) {
    rx_progress = true;
  }
}

void ParseSerialFromMaster() {  // split the data into its parts
  if (rx_data_flag) {
    char* strtokIndx;  // this is used by strtok() as an index
    strtokIndx = strtok(rx_buffer, ",");  // get the first part - the string
    if (strtokIndx != NULL) imu.cali_step = atof(strtokIndx);

    strtokIndx = strtok(rx_buffer, ",");  // get the first part - the string
    if (strtokIndx != NULL) imu.cali_status = atof(strtokIndx);

    rx_data_flag = false;

    // Serial.printf("rx_data: %d, %d\n",imu.cali_step,imu.cali_status);
  }
}


void ImuTask(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (!pref.begin("imu_cali", false)) {
    Serial.println("[ImuTask]imu_cali get Fail");
  }
  matrix::Vector3f offset_temp;
  matrix::Vector3f scale_temp;
  offset_temp(0) = pref.getFloat("offset0", 0.0);
  offset_temp(1) = pref.getFloat("offset1", 0.0);
  offset_temp(2) = pref.getFloat("offset2", 0.0);
  scale_temp(0)  = pref.getFloat("scale0", 1.0);
  scale_temp(1)  = pref.getFloat("scale1", 1.0);
  scale_temp(2)  = pref.getFloat("scale2", 1.0);
  pref.end();

  imu.set_offset(offset_temp);
  imu.set_scale(scale_temp);
  imu.InitICM42688();
  imu.UpdateICM42688();

  // EventPostInit(&imu_pub, IMU_MSG, IMU_MSG_LEN);

  for (;;) {
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, SENSOR_TASK_PERIOD);
    if (!xWasDelayed && millis() > 10000) {
      // Serial.println("[Warning] ImuTask Time Out.");
    }
    
    if(imu.cali_step == 0){
    if(imu.cali_ == 1){
      imu.cali_ = 0;
      for(int i = 0;i < 6;i++){
        imu.cali_flag[i] = 0;
        for (int j = 0; j < 3; ++j) {
            imu.accel_cali[i][j] = 0.0;
        }
      }
    }

    imu.UpdateICM42688();

    // HACK ====================
    imu.ProcessAccel();
    // =========================

    imu.UpdateEulerAngles();

    // HACK =====================
    imu.imu_to_master.angle.roll  = imu.get_roll();
    imu.imu_to_master.angle.pitch = imu.get_pitch();
    imu.imu_to_master.angle.yaw   = imu.get_yaw();
    imu.imu_to_master.accel.x = imu.get_accel()(0);
    imu.imu_to_master.accel.y = imu.get_accel()(1);
    imu.imu_to_master.accel.z = imu.get_accel()(2);
    imu.imu_to_master.direction   = imu.get_gravity_direction();
    imu.imu_to_master.temperature = 30;
    // ==========================
    //  EventMsgPost(&imu_pub, imu.get_accel_read_point(), IMU_MSG_LEN);
    //  imu.Print();
    }
    else if(imu.cali_step == 7){
      if(imu.cali_ == 0){
      imu.DoAccelCalibrationFull();
      imu.cali_ = 1;
      }
    }
    else{
      imu.ReadAccelAvg();
    }
  }
}

void CommunicateTask(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  pinMode(LED_PIN, OUTPUT);
  long led_timer = millis();
  bool led_state = 0;
  Serial1.begin(921600, SERIAL_8N1, 0, 1);

  // subscriber_t nolistSubs;
  // EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
  // EventSubscribe(&nolistSubs, IMU_MSG, IMU_MSG_LEN, 0, NULL);

  to_master_t *to_master = &imu.imu_to_master;

  for (;;) {
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, COMMUNICATE_TASK_PERIOD);
    if (!xWasDelayed && millis() > 10000) {
      Serial.println("[Warning] CommunicateTask Time Out.");
    }

    // HACK Guru Meditation Error: Core  0 panic'ed (Load access fault).
    // Exception was unhandled. EventMsgGetLast(&nolistSubs, IMU_MSG,
    // &to_master.angle, NULL);

    SendTOMaster(to_master);
    int StartTime = millis();

    while (Serial1.available() && millis() - StartTime < 1000 && !rx_data_flag) {
      readSerialData(Serial1.read());
    }
    ParseSerialFromMaster();

    if (millis() - led_timer > 1000) {
    PrintSerialMonitor(to_master);
    led_timer = millis();
    led_state = !led_state;
  }
  digitalWrite(LED_PIN, led_state);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Serial system begin");
  xTaskCreatePinnedToCore(ImuTask, "Core_1_IMU", 16384, NULL, 4, imu_handle, 1);
  xTaskCreatePinnedToCore(CommunicateTask, "Core_1_COM", 8192, NULL, 3,communicate_handle, 1);
}
void loop() {}
