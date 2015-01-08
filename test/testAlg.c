/**
 * Testing suite for algorithms
 */

// Includes
#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <stdarg.h>
//#include <uart.h>

// Defines

// Global Variables
/**
 * Bias and scale constants
 * 0-2 are acc bias, applied to raw acc.
 * 3-5 are acc scale, applied to biased acc.
 * 6-8 are gyro bias, applied to raw gyro.
 * 9-12 are gyro scale, applied to biased gyro
 * 12-14 are mag bias, applied to raw mag.
 * 15-17 are mag scale, applied to biased mag.
 * @type int[18]
 */
int beta[18];

/**
 * Complementary Filter Weight constants.  0 is for the gyro, 1 is for acc+mag.
 * Must add to 1.
 * @type double[3]
 */
double CFW[3];

// Function Prototypes
double _vec_mag(double a[3]);
void _vec_norm(double* a);
double _dot(double a[3], double b[3]);
void apply_bias(int* vec, int* bias, int* scale, int size, double* ret);
void cross_product(double *a, double *b, double *ret);
void force_normal(double *a, double *b);
void make_coord_bases(double *x, double *y, double *z, int mode);
void integrate_gyro(double *gyro, double dt);
void gyro_to_DCM(double *gyro, double DCM[][3]);
void compose_rotations(double ret[][3], double rot1[][3], double rot2[][3]);
void comp_flt(double ret[][3], double input1[][3], double input2[][3], double *weights);
void const_init();
double getRoll(double cur_DCM[][3]);
double getPitch(double cur_DCM[][3]);
double getYaw(double cur_DCM[][3]);

//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
// Function Definitions
/**
 * Returns the yaw (z) axis Euler angle representation of the current
 * orientation
 *
 * @return Yaw Euler angle
 */
double getYaw(double cur_DCM[][3]){
	return atan2(cur_DCM[1][0], cur_DCM[0][0]);
}

/**
 * Returns the pitch (y) axis Euler angle representation of the current
 * orientation.
 *
 * @return Pitch Euler angle
 */
double getPitch(double cur_DCM[][3]){
	return asin(-cur_DCM[2][0]);
}

/**
 * Returns the roll (x) axis Euler angle representation of the current
 * orientation.
 *
 * @return roll Euler angle
 */
double getRoll(double cur_DCM[][3]){
	return atan2(cur_DCM[1][1], cur_DCM[2][1]);
}

void const_init(){
	// setup UART
	//uart_init();
	//stderr = stdout = stdin = &uart_str;

	// Initialize DCM weight
	CFW[0] = 0.4;
	CFW[1] = 0.6;


	// Initialize scale and bias constants
	beta[0] = 99;	// acc_x bias
	beta[1] = 70;	// acc_y bias
	beta[2] = 389;	// acc_z bias
	beta[3] = 16458;	// acc_x scale
	beta[4] = 16356;	// acc_y scale
	beta[5] = 16461;	// acc_z scale
	beta[9] = 7508;	// gyro_x scale
	beta[10] = 7508;	// gyro_y scale
	beta[11] = 7508;	// gyro_z scale
	beta[6] = 0;	// gyro_x bias
	beta[7] = 0;	// gyro_y bias
	beta[8] = 0;	// gyro_z bias
	beta[12] = 0;	// mag_x bias
	beta[13] = 0;	// mag_y bias
	beta[14] = 0;	// mag_z bias
	beta[15] = 160;	// mag_x scale
	beta[16] = 160;	// mag_y scale
	beta[17] = 160;	// mag_z scale
}

int main(int argc, char** argv){
	// INIT SCRIPT
	const_init();
	double DCM1[3][3];
	double gyro[3] = {3 * M_PI / 4, 0, 0};
	gyro_to_DCM(gyro, DCM1);
	printf("ROT_DCMG:\t%+3.4f\t%+3.4f\t%+3.4f\n", DCM1[0][0], DCM1[0][1], DCM1[0][2]);
	printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM1[1][0], DCM1[1][1], DCM1[1][2]);
	printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM1[2][0], DCM1[2][1], DCM1[2][2]);


	printf("%3.3f\t%3.3f\t%3.3f\n", getRoll(DCM1) * 180 / M_PI, getPitch(DCM1) * 180 / M_PI, getYaw(DCM1) * 180 / M_PI);


	// double DCM1[3][3];
	// double *scl_acc = DCM1[2];
	// double *scl_mag = DCM1[0];
	// // Get and scale acc
	// int raw_acc[3] = {-99, -70, -389};
	// apply_bias(raw_acc, &(beta[0]), &(beta[3]), 3, scl_acc);
	// raw_acc[2] = 10000;
	// apply_bias(raw_acc, &(beta[0]), &(beta[3]), 3, scl_acc);
	// _vec_norm(scl_acc);
	// assert(fabs(scl_acc[0] - 0) < 0.001);
	// assert(fabs(scl_acc[1] - 0) < 0.001);
	// assert(fabs(scl_acc[2] - 1) < 0.001);

	// // get and scale mag
	// int raw_mag[3] = {0, 0, 0};
	// raw_mag[0] = 1600;
	// apply_bias(raw_mag, &(beta[12]), &(beta[15]), 3, scl_mag);
	// _vec_norm(scl_mag);
	// assert(fabs(scl_acc[0] - 1) < 0.001);
	// assert(fabs(scl_acc[1] - 0) < 0.001);
	// assert(fabs(scl_acc[2] - 0) < 0.001);

	// // Force normal
	// force_normal(scl_mag, scl_acc);
	// // printf("ACC: %3.6f\t%3.6f\t%3.6f\n", scl_acc[0], scl_acc[1], scl_acc[2]);
	// // printf("MAG: %3.6f\t%3.6f\t%3.6f\n", scl_mag[0], scl_mag[1], scl_mag[2]);

	// // Create bases
	// make_coord_bases(DCM1[0], DCM1[1], DCM1[2], 1 << 1);
	// // printf("SYNTH_GYRO: %3.6f\t%3.6f\t%3.6f\n", DCM1[1][0], DCM1[1][1], DCM1[1][2]);
	// // printf("ROT_DCMG:\t%+3.4f\t%+3.4f\t%+3.4f\n", DCM1[0][0], DCM1[0][1], DCM1[0][2]);
	// // printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM1[1][0], DCM1[1][1], DCM1[1][2]);
	// // printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM1[2][0], DCM1[2][1], DCM1[2][2]);

	// // get and scale gyro
	// int raw_gyro[3] = {0, 0, 0};
	// double scl_gyro[3];
	// double DCM2[3][3];
	// apply_bias(raw_gyro, &(beta[6]), &(beta[9]), 3, scl_gyro);

	// // Integrate gyro
	// integrate_gyro(scl_gyro, 1);	// XXX 1 second integration!
	// // printf("GRYO: %3.9f\t%3.9f\t%3.9f\n", scl_gyro[0], scl_gyro[1], scl_gyro[2]);

	// // convert gyro to DCM
	// gyro_to_DCM(scl_gyro, (double (*)[3])DCM2);
	// // printf("ITER_DCMG:\t%+3.4f\t%+3.4f\t%+3.4f\n", DCM2[0][0], DCM2[0][1], DCM2[0][2]);
	// // printf("          \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM2[1][0], DCM2[1][1], DCM2[1][2]);
	// // printf("          \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM2[2][0], DCM2[2][1], DCM2[2][2]);

	// // add iterant DCM to previous DCM
	// double prev_DCM[3][3] = {{1, 0, 0},
	// 						{0, 1, 0},
	// 						{0, 0, 1}};
	// double DCM3[3][3] = {{0}};
	// compose_rotations(DCM3, prev_DCM, DCM2);
	// // printf("CUM_DCMG:\t%+3.4f\t%+3.4f\t%+3.4f\n", DCM3[0][0], DCM3[0][1], DCM3[0][2]);
	// // printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM3[1][0], DCM3[1][1], DCM3[1][2]);
	// // printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", DCM3[2][0], DCM3[2][1], DCM3[2][2]);

	// // Apply complementary filter
	// comp_flt(prev_DCM, DCM1, DCM3, CFW);
	// // printf("NEW_DCMG:\t%+3.4f\t%+3.4f\t%+3.4f\n", prev_DCM[0][0], prev_DCM[0][1], prev_DCM[0][2]);
	// // printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", prev_DCM[1][0], prev_DCM[1][1], prev_DCM[1][2]);
	// // printf("         \t%+3.4f\t%+3.4f\t%+3.4f\n", prev_DCM[2][0], prev_DCM[2][1], prev_DCM[2][2]);
}

/**
 * Complementary filter for two inputs.
 * 
 * @param ret		Pointer to the memory location for the returned DCM
 *            		representation
 * @param input1	First input DCM
 * @param input2	Second input DCM
 * @param weights	Array of DCM weights
 */
void comp_flt(double ret[][3], double input1[][3], double input2[][3], double *weights){
	for(int row = 0; row < 3; row++){
		for(int col = 0; col < 3; col++){
			ret[row][col] = input1[row][col] * weights[0] + input2[row][col] * weights[1];
		}
	}
}

/**
 * Composes the rotation rot2 onto rot1, i.e. apply rot1 first, then apply rot2.
 * ret must be initialized to zero prior to calling this function.
 *
 * @param ret	Pointer to the array to populate with the new rotation matrix
 * @param ret1	Pointer to the array representing matrix 1 in row col notation
 * @param ret2	Pointer to the array representing matrix 2 in row col notation
 */
void compose_rotations(double ret[][3], double rot1[][3], double rot2[][3]){
	for(int row = 0; row < 3; row++){
		for(int col = 0; col < 3; col++){
			for(int i = 0; i < 3; i++){
				ret[row][col] += rot1[row][i] * rot2[i][col];
			}
		}
	}
}

/**
 * Converts an array of the three primary Euler angles to a DCM matrix
 * representing the world orientation from the body perspective.
 *
 * @param gyro	Roll, Pitch, and Yaw (radians) representation as
 *             	[Roll, Pitch, Yaw]
 * @param DCM	Direct Cosine Matrix representing the rotation.  Must be a 3x3
 *            	matrix.  DCM representation is DCM[row][col].
 */
void gyro_to_DCM(double *gyro, double DCM[][3]){
	double roll, pitch, yaw;
	roll = gyro[0];
	pitch = gyro[1];
	yaw = gyro[2];

	DCM[0][0] = cos(pitch) * cos(yaw);
	DCM[0][1] = -cos(pitch) * sin(yaw);
	DCM[0][2] = sin(pitch);

	DCM[1][0] = cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw);
	DCM[1][1] = cos(roll) * cos(yaw) - sin(roll) * sin(pitch) * sin(yaw);
	DCM[1][2] = -sin(roll) * cos(pitch);

	DCM[2][0] = sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw);
	DCM[2][1] = sin(roll) * cos(yaw) + cos(roll) * sin(pitch) * sin(yaw);
	DCM[2][2] = cos(roll) * cos(pitch);
}

/**
 * Performs an integration over the gyroscope data with time interfal dt.
 * Assumes that there are three angles in the gyroscope data.
 * 
 * @param gyro Angular velocity data in angle per second
 * @param dt   time interval in seconds
 */
void integrate_gyro(double *gyro, double dt){
	for(int i = 0; i < 3; i++){
		gyro[i] *= dt;
	}
}
/**
 * Makes a coordinate bases using any two vectors.  Use mode to specify which
 * axis is unknown by placing raising bit 0 for Z, bit 1 for Y, bit 2 for X.
 * Assumes that the given vectors are already normalized.
 * 
 * @param x    x unit vector
 * @param y    y unit vector
 * @param z    z unit vector
 * @param mode bit field describing which coordinate bases to generate.  Bit 0
 *             should be raised for the z vector, bit 1 should be raised for the
 *             y vector, and bit 2 should be raised for the x vector.
 */
void make_coord_bases(double *x, double *y, double *z, int mode){
	double *vec1;
	double *vec2;
	double *newVec;
	switch(mode){
		case 1 << 0:
			vec1 = x;
			vec2 = y;
			newVec = z;
			break;
		case 1 << 1:
			vec1 = z;
			vec2 = x;
			newVec = y;
			break;
		case 1 << 2:
			vec1 = y;
			vec2 = z;
			newVec = x;
			break;
	}
	cross_product(vec1, vec2, newVec);
	_vec_norm(newVec);
}
/**
 * This function will force vec to be normal to ref by moving and adjusting vec.
 * Use the magnetometer for ref and accelerometer for vec.  Vec will also be
 * normalized following the operations.
 * @param ref Reference vector
 * @param vec Moving vector
 */
void force_normal(double *ref, double *vec){
	double norm[3];
	cross_product(ref, vec, norm);
	cross_product(norm, ref, vec);
	_vec_norm(vec);
	return;
}

/**
 * Returns the cross product of vector a and vector b in the array ret.
 * Requires all parameters have three elements
 * @param a   First vector
 * @param b   Second vector
 * @param ret Array in which to store the cross product
 */
void cross_product(double *a, double *b, double *ret){
	ret[0] = a[1] * b[2] - a[2] * b[1];
	ret[1] = a[2] * b[0] - a[0] * b[2];
	ret[2] = a[0] * b[1] - a[1] * b[0];
	return;
}

/**
 * Applies the bias and scaling characteristics to the given vector.  Result
 * will be placed in the ret array.  Result is equal to (vec + bias) / scale.
 * @param vec   n element vector to be scaled
 * @param bias  n element vector representing the bias to be applied
 * @param scale n element vector representing the scale to be applied
 * @param size	Integer representing n
 * @param ret   n element vector that will be filled with the scaled result
 */
void apply_bias(int* vec, int* bias, int* scale,int size, double* ret){
	for(int i = 0; i < size; i++){
		ret[i] = ((double)vec[i] + bias[i]) / scale[i];
	}
	return;
}

/**
 * Calculates the magnitude of the vector.
 * @param  a A vector, whose magnitude will be calculated
 * @return   Magnitude of vector a
 */
double _vec_mag(double *a){
	return sqrt(a[0] * a[0] + a[1]* a[1] + a[2] * a[2]);
}

/**
 * Returns the magnitude of the quarternion.
 * @param  a A quaternion, whose magnitude will be calculated
 * @return   Magnitude of quaternion a
 */
double _quat_mag(double *a){
	return sqrt(a[0] * a[0] + a[1]* a[1] + a[2] * a[2] + a[3] * a[3]);
}

/**
 * Normalizes the supplied vector.  The vector supplied will be modified to
 * have a magnitude of 1.
 * @param  a Vector to be normalized.
 */
void _vec_norm(double* a){
	double __mag = _vec_mag(a);
	if(fabs(__mag) < 0.001){
		a[0] = __mag;
		a[1] = __mag;
		a[2] = __mag;
		return;
	}
	a[0] /= __mag;
	a[1] /= __mag;
	a[2] /= __mag;
	return;
}

/**
 * Normalizes the supplied quarternion.  The quarternion supplied will be
 * modified to have a magnitude of 1.
 * @param a Quaternion to be normalized.
 */
void _quat_norm(double* a){
	double __mag = _vec_mag(a);
	a[0] /= __mag;
	a[1] /= __mag;
	a[2] /= __mag;
	a[3] /= __mag;
	return;
}

/**
 * Returns the dot product of the two vectors supplied.  The dot product is
 * equal to the sum of the products of eacn analogous element.  Thus, the dot
 * product of vectors u and v is u_x * v_x + u_y * v_y + u_z * v_z.
 * @param  a First vector
 * @param  b Second vector
 * @return   Dot product of vectors a and b
 */
double _dot(double *a, double *b){
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
