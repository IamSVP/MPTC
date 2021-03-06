#ifndef __TCAR_FAST_DCT_H__
#define __TCAR_FAST_DCT_H__

#define _USE_MATH_DEFINES
#include <cmath>
#define M_PI 3.14159265358979323846
namespace MPTC {

  // Constants:
  static const float s1 = sinf(1.f * static_cast<float>(M_PI) / 16.f);
  static const float c1 = cosf(1.f * static_cast<float>(M_PI) / 16.f);
  static const float s3 = sinf(3.f * static_cast<float>(M_PI) / 16.f);
  static const float c3 = cosf(3.f * static_cast<float>(M_PI) / 16.f);
  static const float s6 = sinf(3.f * static_cast<float>(M_PI) / 8.f);
  static const float c6 = cosf(3.f * static_cast<float>(M_PI) / 8.f);

  static inline void fdct(float in[8], float out[8]) {
    // After stage 1:
    const float s1_0 = in[0] + in[7];
    const float s1_1 = in[1] + in[6];
    const float s1_2 = in[2] + in[5];
    const float s1_3 = in[3] + in[4];
    const float s1_4 = in[3] - in[4];
    const float s1_5 = in[2] - in[5];
    const float s1_6 = in[1] - in[6];
    const float s1_7 = in[0] - in[7];

    // After stage 2:
    const float s2_0 = s1_0 + s1_3;
    const float s2_1 = s1_1 + s1_2;
    const float s2_2 = s1_1 - s1_2;
    const float s2_3 = s1_0 - s1_3;

    const float z1 = c3 * (s1_7 + s1_4);
    const float s2_4 = ( s3-c3) * s1_7 + z1;
    const float s2_7 = (-s3-c3) * s1_4 + z1;

    const float z2 = c1 * (s1_6 + s1_5);
    const float s2_5 = ( s1-c1) * s1_6 + z2;
    const float s2_6 = (-s1-c1) * s1_5 + z2;

    // After stage 3:
    const float s3_0 =  s2_0 + s2_1;
    const float s3_1 = -s2_1 + s2_0;

    const float z3 = c6 * (s2_3 + s2_2);
    const float s3_2 = ( s6-c6) * s2_3 + z3;
    const float s3_3 = (-s6-c6) * s2_2 + z3;

    const float s3_4 =  s2_4 + s2_6;
    const float s3_5 = -s2_5 + s2_7;
    const float s3_6 = -s2_6 + s2_4;
    const float s3_7 =  s2_7 + s2_5;

    // After stage 4:
    const float s4_4 = -s3_4 + s3_7;
    const float s4_5 =  s3_5;
    const float s4_6 =  s3_6;
    const float s4_7 =  s3_7 + s3_4;

    // Shuffle and scaling:
    out[0] = s3_0;
    out[4] = s3_1;
    out[2] = s3_2;
    out[6] = s3_3;
    out[7] = s4_4;
    out[3] = s4_5;  // Alternative: s3_5 / 2
    out[5] = s4_6;
    out[1] = s4_7;
  }

  static inline void idct(float in[8], float out[8]) {
    const float s3_0 = out[0];
    const float s3_1 = out[4];
    const float s3_2 = out[2];
    const float s3_3 = out[6];
    const float s4_4 = out[7];
    const float s4_5 = out[3];
    const float s4_6 = out[5];
    const float s4_7 = out[1];

    const float s3_4 = s4_7 - s4_4;
    const float s3_5 = s4_5 * 2.0f;
    const float s3_6 = s4_6 * 2.0f;
    const float s3_7 = s4_7 + s4_4;

    const float s2_0 = s3_0 + s3_1;
    const float s2_1 = s3_0 - s3_1;
    const float s2_2 = (c6 * s3_2 - s6 * s3_3) * 2.f;
    const float s2_3 = (s6 * s3_2 + c6 * s3_3) * 2.f;
    const float s2_4 = s3_4 + s3_6;
    const float s2_5 = s3_7 - s3_5;
    const float s2_6 = s3_4 - s3_6;
    const float s2_7 = s3_7 + s3_5;

    const float s1_0 = s2_0 + s2_3;
    const float s1_1 = s2_1 + s2_2;
    const float s1_2 = s2_1 - s2_2;
    const float s1_3 = s2_0 - s2_3;
    const float s1_4 = c3 * s2_4 - s3 * s2_7;
    const float s1_5 = c1 * s2_5 - s1 * s2_6;
    const float s1_6 = s1 * s2_5 + c1 * s2_6;
    const float s1_7 = s3 * s2_4 + c3 * s2_7;

    in[0] = s1_0 + s1_7;
    in[1] = s1_1 + s1_6;
    in[2] = s1_2 + s1_5;
    in[3] = s1_3 + s1_4;
    in[4] = s1_3 - s1_4;
    in[5] = s1_2 - s1_5;
    in[6] = s1_1 - s1_6;
    in[7] = s1_0 - s1_7;
  }
}  // namespace GenTC

#endif  // #define __TCAR_FAST_DCT_H__
