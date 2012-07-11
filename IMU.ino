
void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  #if defined(NUNCHUCK)
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    ACC_getADC();
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    f.NUNCHUKDATA = 1;
    while(f.NUNCHUKDATA) ACC_getADC(); // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis])/4;
      gyroADCprevious[axis] = gyroADC[axis];
    }
  #else
    #if ACC
      ACC_getADC();
      getEstimatedAttitude();
    #endif
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    if ((micros()-timeInterleave)>650) {
       annex650_overrun_count++;
    } else {
       while((micros()-timeInterleave)<650) ; //empirical, interleaving delay between 2 consecutive reads
    }
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      if (!ACC) accADC[axis]=0;
    }
  #endif
  #if defined(GYRO_SMOOTHING)
    static int16_t gyroSmooth[3] = {0,0,0};
    for (axis = 0; axis < 3; axis++) {
      gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis]-1) )+gyroData[axis]+1 ) / conf.Smoothing[axis]);
      gyroSmooth[axis] = gyroData[axis];
    }
  #elif defined(TRI)
    static int16_t gyroYawSmooth = 0;
    gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW])/3;
    gyroYawSmooth = gyroData[YAW];
  #endif
  
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 10
#endif

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
#ifndef MG_LPF_FACTOR
//#define MG_LPF_FACTOR 4
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 400.0f
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#ifndef GYR_CMPFM_FACTOR
  #define GYR_CMPFM_FACTOR 200.0f
#endif

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
  #define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
  // +-2000/sec deg scale
  //#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
  #define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif 
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

int16_t _atan2(float y, float x){
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10); 
  return z;
}

#define GYRO_SCALE (4.0f / 16.384f * PI / 180.0f);

void getEstimatedAttitude() {
  uint8_t axis;
#if defined(MG_LPF_FACTOR)
  static int16_t mgSmooth[3]; 
#endif
#if defined(ACC_LPF_FACTOR)
  static float accLPF[3];
#endif
  
  static float q1 = 1, q2 = 0, q3 = 0, q4 = 0;
  static float Kp = 2.0f;
  static float Ki = 0.001f;
  static float eInt1, eInt2, eInt3;
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  static uint16_t previousT;
  uint16_t currentT = micros();

  float dt = (currentT - previousT) / 1000000.0f;
  previousT = currentT;
  
  for (axis = 0; axis < 3; axis++) {
    #if defined(ACC_LPF_FACTOR)
      accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
      accSmooth[axis] = accLPF[axis];
      #define ACC_VALUE accSmooth[axis]
    #else  
      accSmooth[axis] = accADC[axis];
      #define ACC_VALUE accADC[axis]
    #endif    
  }

  float gx = -gyroADC[1] * GYRO_SCALE;
  float gy = gyroADC[0] * GYRO_SCALE;
  float gz = -gyroADC[2] * GYRO_SCALE;

  float ax = -accADC[0];
  float ay = -accADC[1];
  float az = accADC[2];
    
  float mx = magADC[0];
  float my = magADC[1];
  float mz = -magADC[2];
    
  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1 / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1 / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2 * mx * (0.5f - q3q3 - q4q4) + 2 * my * (q2q3 - q1q4) + 2 * mz * (q2q4 + q1q3);
  hy = 2 * mx * (q2q3 + q1q4) + 2 * my * (0.5f - q2q2 - q4q4) + 2 * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2 * mx * (q2q4 - q1q3) + 2 * my * (q3q4 + q1q2) + 2 * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2 * (q2q4 - q1q3);
  vy = 2 * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2 * bx * (0.5f - q3q3 - q4q4) + 2 * bz * (q2q4 - q1q3);
  wy = 2 * bx * (q2q3 - q1q4) + 2 * bz * (q1q2 + q3q4);
  wz = 2 * bx * (q1q3 + q2q4) + 2 * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
      eInt1 += ex;      // accumulate integral error
      eInt2 += ey;
      eInt3 += ez;
  }
  else
  {
      eInt1 = 0.0f;     // prevent integral wind up
      eInt2 = 0.0f;
      eInt3 = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt1;
  gy = gy + Kp * ey + Ki * eInt2;
  gz = gz + Kp * ez + Ki * eInt3;

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * dt);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * dt);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * dt);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * dt);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
  q4 = q4 * norm;
  
  debug[0] = q1 * 1000;
  debug[1] = q2 * 1000;
  debug[2] = q3 * 1000;
  debug[3] = q4 * 1000;
  
  vx = 2 * (q2q4 - q1q3);
  vy = 2 * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;

  angle[ROLL] = -_atan2(vx, sqrt(vy*vy + vz*vz));
  angle[PITCH] = -_atan2(vy, sqrt(vx*vx + vz*vz));
  heading = _atan2(2*q2*q3 - 2*q1*q4, 2*q1*q1 + 2*q2*q2 - 1) / 10 - 90 + MAG_DECLINIATION;
}


// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
}

void getEstimatedAttitude2(){
  uint8_t axis;
  int32_t accMag = 0;
  static t_fp_vector EstG;
#if MAG
  static t_fp_vector EstM;
#endif
#if defined(MG_LPF_FACTOR)
  static int16_t mgSmooth[3]; 
#endif
#if defined(ACC_LPF_FACTOR)
  static float accLPF[3];
#endif
  static uint16_t previousT;
  uint16_t currentT = micros();
  float scale, deltaGyroAngle[3];

  scale = (currentT - previousT) * GYRO_SCALE;
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * scale;
    #if defined(ACC_LPF_FACTOR)
      accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
      accSmooth[axis] = accLPF[axis];
      #define ACC_VALUE accSmooth[axis]
    #else  
      accSmooth[axis] = accADC[axis];
      #define ACC_VALUE accADC[axis]
    #endif
//    accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);
    accMag += (int32_t)ACC_VALUE*ACC_VALUE ;
    #if MAG
      #if defined(MG_LPF_FACTOR)
        mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
        #define MAG_VALUE mgSmooth[axis]
      #else  
        #define MAG_VALUE magADC[axis]
      #endif
    #endif
  }
  accMag = accMag*100/((int32_t)acc_1G*acc_1G);

  rotateV(&EstG.V,deltaGyroAngle);
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);
  #endif 

  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 )
    for (axis = 0; axis < 3; axis++) {
      int16_t acc = ACC_VALUE;
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
    }
  #if MAG
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
  #endif
  
  // Attitude of the estimated vector
  //angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
  //angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;

  angle[ROLL]  =  _atan2(EstG.V.X , sqrt(EstG.V.Y*EstG.V.Y + EstG.V.Z*EstG.V.Z)) ;
  angle[PITCH] =  _atan2(EstG.V.Y , sqrt(EstG.V.X*EstG.V.X + EstG.V.Z*EstG.V.Z)) ;

  #if MAG
    // Attitude of the cross product vector GxM
    heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );
    heading += MAG_DECLINIATION * 10; //add declination
    heading = heading /10;
    if ( heading > 180)      heading = heading - 360;
    else if (heading < -180) heading = heading + 360;
  #endif
}

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000  // 4 sec initialization delay
#define BARO_TAB_SIZE   40

void getEstimatedAltitude(){
  uint8_t index;
  static uint32_t deadLine = INIT_DELAY;

  static int16_t BaroHistTab[BARO_TAB_SIZE];
  static int8_t BaroHistIdx;
  static int32_t BaroHigh,BaroLow;
  int32_t temp32;
  int16_t last;

  if (abs(currentTime - deadLine) < UPDATE_INTERVAL) return;
  deadLine = currentTime; 

  //**** Alt. Set Point stabilization PID ****
  //calculate speed for D calculation
  last = BaroHistTab[BaroHistIdx];
  BaroHistTab[BaroHistIdx] = BaroAlt/10;
  BaroHigh += BaroHistTab[BaroHistIdx];
  index = (BaroHistIdx + (BARO_TAB_SIZE/2))%BARO_TAB_SIZE;
  BaroHigh -= BaroHistTab[index];
  BaroLow  += BaroHistTab[index];
  BaroLow  -= last;

  BaroHistIdx++;
  if (BaroHistIdx == BARO_TAB_SIZE) BaroHistIdx = 0;

  BaroPID = 0;
  //D
  temp32 = conf.D8[PIDALT]*(BaroHigh - BaroLow) / 40;
  BaroPID-=temp32;

  EstAlt = BaroHigh*10/(BARO_TAB_SIZE/2);
  
  temp32 = AltHold - EstAlt;
  if (abs(temp32) < 10 && abs(BaroPID) < 10) BaroPID = 0;  //remove small D parametr to reduce noise near zero position
  
  //P
  BaroPID += conf.P8[PIDALT]*constrain(temp32,(-2)*conf.P8[PIDALT],2*conf.P8[PIDALT])/100;   
  BaroPID = constrain(BaroPID,-150,+150); //sum of P and D should be in range 150

  //I
  errorAltitudeI += temp32*conf.I8[PIDALT]/50;
  errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
  temp32 = errorAltitudeI / 500; //I in range +/-60
  BaroPID+=temp32;
}
