
#if defined IMU_EXPERIMENTAL

#define GYRO_SCALE (4.0f / 16.384f * PI / 180.0f);

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

#endif // IMU_EXPERIMENTAL
