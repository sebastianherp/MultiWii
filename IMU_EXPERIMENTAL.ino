
#if defined IMU_EXPERIMENTAL

#define KpACCMAG 5.0f
#define KiACCMAG 0.001f
#define GYRO_SCALE (4.0f / 16.384f * PI / 180.0f);
#define NORM_AS_FUNCTION

float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

void normalize3(float &x, float &y, float &z) {
  float norm = sqrt(x * x + y * y + z * z);
  if (norm == 0.0f) return; // handle NaN
  norm = 1 / norm;        // use reciprocal for division
  x *= norm;
  y *= norm;
  z *= norm;  
}

void normalize4(float &v1, float &v2, float &v3, float &v4) {
  float norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3 + v4 * v4);
  if (norm == 0.0f) return; // handle NaN
  norm = 1 / norm;        // use reciprocal for division
  v1 *= norm;
  v2 *= norm;
  v3 *= norm;  
  v4 *= norm;  
}


void getEstimatedAttitudeExperimental() {
  uint8_t axis;
#if defined(MG_LPF_FACTOR)
  static int16_t mgSmooth[3]; 
#endif
#if defined(ACC_LPF_FACTOR)
  static float accLPF[3];
#endif
  
  static float q1 = 1, q2 = 0, q3 = 0, q4 = 0;
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

  float ax = -accSmooth[0];
  float ay = -accSmooth[1];
  float az = accSmooth[2];
    
  float mx = magADC[0];
  float my = magADC[1];
  float mz = -magADC[2];
    
  // Normalise accelerometer measurement
#if defined NORM_AS_FUNCTION
  float zero = 0;
  normalize4(ax, ay, az, zero);
#else
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1 / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;
#endif

  // Normalise magnetometer measurement
#if defined NORM_AS_FUNCTION
  normalize4(mx, my, mz, zero);
#else
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1 / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;
#endif

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
  
  // Calculate angles from estimation
  angle[ROLL] = -_atan2(vx, sqrt(vy*vy + vz*vz));
  angle[PITCH] = -_atan2(vy, sqrt(vx*vx + vz*vz));
  heading = _atan2(2*(q2q3 - q1q4), 2*(q1q1 + q2q2) - 1) / 10 - 90 + MAG_DECLINIATION;
  if ( heading > 180)      heading = heading - 360;
  else if (heading < -180) heading = heading + 360;
    
  //debug[0] = -_atan2(vx, sqrt(vy*vy + vz*vz));
  //debug[1] = -_atan2(vy, sqrt(vx*vx + vz*vz));
  //debug[2] = angle[ROLL];
  //debug[3] = angle[PITCH];

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (KiACCMAG > 0.0f)
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
  gx = gx + KpACCMAG * ex + KiACCMAG * eInt1;
  gy = gy + KpACCMAG * ey + KiACCMAG * eInt2;
  gz = gz + KpACCMAG * ez + KiACCMAG * eInt3;

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (dt * 0.5f); // (dt * 0.5f) uses less space than just dt ... wtf
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (dt * 0.5f);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (dt * 0.5f);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (dt * 0.5f);

  // Normalise quaternion
#if defined NORM_AS_FUNCTION
  normalize4(q1, q2, q3, q4);
#else   
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
  q4 = q4 * norm;
#endif
  
}



#endif // IMU_EXPERIMENTAL
