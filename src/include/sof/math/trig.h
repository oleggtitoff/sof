/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 * Author: Seppo Ingalsuo <seppo.ingalsuo@linux.intel.com>
 *         Liam Girdwood <liam.r.girdwood@linux.intel.com>
 *         Keyon Jie <yang.jie@linux.intel.com>
 *         Shriram Shastry <malladi.sastry@linux.intel.com>
 */

#ifndef __SOF_MATH_TRIG_H__
#define __SOF_MATH_TRIG_H__

#include <stdint.h>

#ifndef UNIT_CORDIC_TEST
#define CONFIG_CORDIC_TRIGONOMETRY_FIXED
#endif


#define PI_DIV2_Q4_28 421657428
#define PI_Q4_28      843314857
#define PI_MUL2_Q4_28     1686629713
#define CORDIC_31B_TABLE_SIZE		31
#define CORDIC_15B_TABLE_SIZE		15
#define CORDIC_30B_ITABLE_SIZE		30
#define CORDIC_15B_ITABLE_SIZE		15

typedef enum {
	EN_32B_CORDIC_SINE,
	EN_32B_CORDIC_COSINE,
	EN_32B_CORDIC_CEXP,
	EN_16B_CORDIC_SINE,
	EN_16B_CORDIC_COSINE,
	EN_16B_CORDIC_CEXP,
	EN_32B_CORDIC_ASINE,
	EN_32B_CORDIC_ACOSINE,
} cordic_cfg;

static const int32_t cordic_ilookup[CORDIC_30B_ITABLE_SIZE] = {
	    497837829, 263043836, 133525158, 67021686, 33543515, 16775850,
	    8388437,   4194282,	  2097149,   1048575,  524287,	 262143,
	    131071,    65535,	  32767,     16383,    8191,	 4095,
	    2047,      1023,	  511,	     255,      127,	 63,
	    31,	       15,	  8,	     4,	       2,	 1};
void cordic_approx(int32_t th_rad_fxp, cordic_cfg type, int32_t *sign, int32_t *b_yn, int32_t *xn,
		   int32_t *th_cdc_fxp);
int32_t iscalarcordic(int32_t realvalue, int16_t numiters,
		      const int32_t LUT[CORDIC_30B_ITABLE_SIZE], cordic_cfg type);
/* Input is Q4.28, output is Q1.31 */
/**
 * Compute fixed point cordicsine with table lookup and interpolation
 * The cordic sine algorithm converges, when the angle is in the range
 * [-pi/2, pi/2).If an angle is outside of this range, then a multiple of
 * pi/2 is added or subtracted from the angle until it is within the range
 * [-pi/2,pi/2).Start with the angle in the range [-2*pi, 2*pi) and output
 * has range in [-1.0 to 1.0]
 * +------------------+-----------------+--------+--------+
 * | thRadFxp	      | cdcsinth	|thRadFxp|cdcsinth|
 * +----+-----+-------+----+----+-------+--------+--------+
 * |WLen| FLen|Signbit|WLen|FLen|Signbit| Qformat| Qformat|
 * +----+-----+-------+----+----+-------+--------+--------+
 * | 32 | 28  |  1    | 32 | 31 |   1	| 4.28	  | 1.31  |
 * +------------------+-----------------+--------+--------+
 */
static inline int32_t sin_fixed_32b(int32_t th_rad_fxp)
{
	int32_t th_cdc_fxp;
	int32_t sign;
	int32_t b_yn;
	int32_t xn;
	cordic_cfg type = EN_32B_CORDIC_SINE;
	cordic_approx(th_rad_fxp, type, &sign, &b_yn, &xn, &th_cdc_fxp);

	th_cdc_fxp = sign * b_yn;
	/*convert Q2.30 to Q1.31 format*/
	return sat_int32(Q_SHIFT_LEFT((int64_t)th_cdc_fxp, 30, 31));
}
/**
 * Compute fixed point cordicsine with table lookup and interpolation
 * The cordic cosine algorithm converges, when the angle is in the range
 * [-pi/2, pi/2).If an angle is outside of this range, then a multiple of
 * pi/2 is added or subtracted from the angle until it is within the range
 * [-pi/2,pi/2).Start with the angle in the range [-2*pi, 2*pi) and output
 * has range in [-1.0 to 1.0]
 * +------------------+-----------------+--------+--------+
 * | thRadFxp	      | cdccosth	|thRadFxp|cdccosth|
 * +----+-----+-------+----+----+-------+--------+--------+
 * |WLen| FLen|Signbit|WLen|FLen|Signbit| Qformat| Qformat|
 * +----+-----+-------+----+----+-------+--------+--------+
 * | 32 | 28  |  1    | 32 | 31 |   1	| 4.28	  | 1.31  |
 * +------------------+-----------------+--------+--------+
 */
static inline int32_t cos_fixed_32b(int32_t th_rad_fxp)
{
	int32_t th_cdc_fxp;
	int32_t sign;
	int32_t b_yn;
	int32_t xn;
	cordic_cfg type = EN_32B_CORDIC_COSINE;
	cordic_approx(th_rad_fxp, type, &sign, &b_yn, &xn, &th_cdc_fxp);

	th_cdc_fxp = sign * xn;
	/*convert Q2.30 to Q1.31 format*/
	return sat_int32(Q_SHIFT_LEFT((int64_t)th_cdc_fxp, 30, 31));
}
/* Input is Q4.28, output is Q1.15 */
/**
 * Compute fixed point cordic sine with table lookup and interpolation
 * The cordic sine algorithm converges, when the angle is in the range
 * [-pi/2, pi/2).If an angle is outside of this range, then a multiple of
 * pi/2 is added or subtracted from the angle until it is within the range
 * [-pi/2,pi/2).Start with the angle in the range [-2*pi, 2*pi) and output
 * has range in [-1.0 to 1.0]
 * +------------------+-----------------+--------+------------+
 * | thRadFxp	      | cdcsinth	|thRadFxp|    cdcsinth|
 * +----+-----+-------+----+----+-------+--------+------------+
 * |WLen| FLen|Signbit|WLen|FLen|Signbit| Qformat| Qformat    |
 * +----+-----+-------+----+----+-------+--------+------------+
 * | 32 | 28  |  1    | 32 | 15 |   1	| 4.28	 | 1.15       |
 * +------------------+-----------------+--------+------------+
 */
static inline int16_t sin_fixed_16b(int32_t th_rad_fxp)
{
	int32_t th_cdc_fxp;
	int32_t sign;
	int32_t b_yn;
	int32_t xn;
	cordic_cfg type = EN_16B_CORDIC_SINE;
	/* compute coeff from angles*/
	cordic_approx(th_rad_fxp, type, &sign, &b_yn, &xn, &th_cdc_fxp);

	th_cdc_fxp = sign * b_yn;
	/*convert Q1.31 to Q1.15 format*/
	return sat_int16(Q_SHIFT_RND((sat_int32(Q_SHIFT_LEFT((int64_t)th_cdc_fxp, 30, 31))),
	31, 15));
}
/**
 * Compute fixed point cordic cosine with table lookup and interpolation
 * The cordic cos algorithm converges, when the angle is in the range
 * [-pi/2, pi/2).If an angle is outside of this range, then a multiple of
 * pi/2 is added or subtracted from the angle until it is within the range
 * [-pi/2,pi/2).Start with the angle in the range [-2*pi, 2*pi) and output
 * has range in [-1.0 to 1.0]
 * +------------------+-----------------+--------+------------+
 * | thRadFxp	      | cdccosth	|thRadFxp|    cdccosth|
 * +----+-----+-------+----+----+-------+--------+------------+
 * |WLen| FLen|Signbit|WLen|FLen|Signbit| Qformat| Qformat    |
 * +----+-----+-------+----+----+-------+--------+------------+
 * | 32 | 28  |  1    | 32 | 15 |   1	| 4.28	 | 1.15       |
 * +------------------+-----------------+--------+------------+
 */
static inline int16_t cos_fixed_16b(int32_t th_rad_fxp)
{
	int32_t th_cdc_fxp;
	int32_t sign;
	int32_t b_yn;
	int32_t xn;
	cordic_cfg type = EN_16B_CORDIC_COSINE;
	/* compute coeff from angles*/
	cordic_approx(th_rad_fxp, type, &sign, &b_yn, &xn, &th_cdc_fxp);

	th_cdc_fxp = sign * xn;
	/*convert Q1.31 to Q1.15 format*/
	return sat_int16(Q_SHIFT_RND((sat_int32(Q_SHIFT_LEFT((int64_t)th_cdc_fxp, 30, 31))),
	31, 15));
}

/**
 * CORDIC-based approximation of inverse cosine
 * inverse cosine of cdc_acos_th based on a CORDIC approximation
 * Inverse cosine angle values in rad
 * Q2.30 cdc_acos_th , input value range [-1 to 1]
 * Q2.30 th_rad_fxp, output value range [3.14159265346825 to 0]
 * LUT size set type 31
 * Error (max = 0.000000026077032), THD+N  = -157.948952635422842
 */
static inline void acosine_fixed_32b(int32_t cdc_acos_th, int32_t th_rad_fxp)
{
	int32_t saturatedUnaryMinus;

	cordic_cfg type = EN_32B_CORDIC_ACOSINE;

	saturatedUnaryMinus = cdc_acos_th;
	if (saturatedUnaryMinus >= 0) {
		th_rad_fxp = iscalarcordic(saturatedUnaryMinus, CORDIC_31B_TABLE_SIZE,
					   cordic_ilookup, type);
	} else {
		if (saturatedUnaryMinus <= INT32_MIN)
			saturatedUnaryMinus = INT32_MAX;
		else
			saturatedUnaryMinus = -saturatedUnaryMinus;

	    th_rad_fxp =
		PI_MUL2_Q4_28 - iscalarcordic(saturatedUnaryMinus, CORDIC_31B_TABLE_SIZE,
		cordic_ilookup, type);
	}
}

/**
 * CORDIC-based approximation of inverse cosine
 * inverse cosine of cdc_acos_th based on a CORDIC approximation
 * Inverse cosine angle values in rad
 * Q2.30 cdc_acos_th , input value range [-1 to 1]
 * Q2.30 th_rad_fxp, output value range [3.14159265346825 to 0]
 * LUT size set type 15
 * Error (max = 0.000000026077032), THD+N  = -157.948952635422842
 */
static inline void acosine_fixed_16b(int32_t cdc_acos_th, int32_t th_rad_fxp)
{
	int32_t saturatedUnaryMinus;
	cordic_cfg type = EN_32B_CORDIC_ACOSINE;

	saturatedUnaryMinus = cdc_acos_th;
	if (saturatedUnaryMinus >= 0) {
		th_rad_fxp = iscalarcordic(saturatedUnaryMinus, CORDIC_15B_ITABLE_SIZE,
					   cordic_ilookup, type);
	} else {
		if (saturatedUnaryMinus <= INT32_MIN)
			saturatedUnaryMinus = INT32_MAX;
		else
			saturatedUnaryMinus = -saturatedUnaryMinus;

		th_rad_fxp =
		PI_MUL2_Q4_28 - iscalarcordic(saturatedUnaryMinus, CORDIC_15B_ITABLE_SIZE,
		cordic_ilookup, type);
	}
}

/**
 * CORDIC-based approximation of inverse sine
 * inverse sine of cdc_asin_th based on a CORDIC approximation.
 * Inverse sine angle values in rad
 * Q2.30 cdc_asin_th,input value range [-1 to 1]
 * Q2.30 th_rad_fxp, output value range [-1.5707963258028 to 1.5707963258028]
 * LUT size set type 31
 * Error (max = 0.000000027939677), THD+N  = -157.454534077921551
 */
static inline void asin_fixed_32b(int32_t cdc_asin_th, int32_t th_rad_fxp)
{
	int32_t saturatedUnaryMinus;
	cordic_cfg type = EN_32B_CORDIC_ASINE;

	saturatedUnaryMinus = cdc_asin_th;
	if (saturatedUnaryMinus >= 0) {
		th_rad_fxp = iscalarcordic(saturatedUnaryMinus, CORDIC_31B_TABLE_SIZE,
					   cordic_ilookup, type);
	} else {
		if (saturatedUnaryMinus <= INT32_MIN)
			saturatedUnaryMinus = INT32_MAX;
		else
			saturatedUnaryMinus = -saturatedUnaryMinus;

	th_rad_fxp = -iscalarcordic(saturatedUnaryMinus, CORDIC_31B_TABLE_SIZE,
				    cordic_ilookup, type);
	}
}

/**
 * CORDIC-based approximation of inverse sine
 * inverse sine of cdc_asin_th based on a CORDIC approximation.
 * Inverse sine angle values in rad
 * Q2.30 cdc_asin_th, value in between range of [-1 to 1]
 * Q2.30 th_rad_fxp, output value range [-1.5707963258028 to 1.5707963258028]
 * LUT size set type 15
 * Error (max = 0.000000027939677), THD+N  = -157.454534077921551
 */
static inline void asin_fixed_16b(int32_t cdc_asin_th, int32_t th_rad_fxp)
{
	int32_t saturatedUnaryMinus;
	cordic_cfg type = EN_32B_CORDIC_ASINE;

	saturatedUnaryMinus = cdc_asin_th;
	if (saturatedUnaryMinus >= 0) {
		th_rad_fxp = iscalarcordic(saturatedUnaryMinus, CORDIC_15B_ITABLE_SIZE,
					   cordic_ilookup, type);
	} else {
		if (saturatedUnaryMinus <= INT32_MIN)
			saturatedUnaryMinus = INT32_MAX;
		else
			saturatedUnaryMinus = -saturatedUnaryMinus;

	    th_rad_fxp = -iscalarcordic(saturatedUnaryMinus, CORDIC_15B_ITABLE_SIZE,
	    cordic_ilookup, type);
	}
}

#endif /* __SOF_MATH_TRIG_H__ */
