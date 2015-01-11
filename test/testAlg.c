/**
 * Testing suite for algorithms
 */

//////////////
// Includes //
//////////////
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <vmath.h>
#include <time.h>
#include <limits.h>

/////////////
// Defines //
/////////////

//////////////////////
// Global Variables //
//////////////////////

/////////////////////////
// Function Prototypes //
/////////////////////////
double DCM_to_Roll(double DCM[3][3]);
double DCM_to_Pitch(double DCM[3][3]);
double DCM_to_Yaw(double DCM[3][3]);
void compose_rotations(double ret[][3], double rot1[][3], double rot2[][3]);


//////////////////////////
// Function Definitions //
//////////////////////////
/**
 * Calculates and returns the yaw component of the given rotation matrix,
 * using a y-x'-z" rotation scheme.
 *
 * @return	calculated yaw component of the given rotation matrix
 */
double DCM_to_Yaw(double DCM[3][3]){
	return acos(DCM[1][1] / cos(-asin(DCM[1][2])));
}

/**
 * Calculates and returns the pitch component of the given rotation matrix,
 * using a y-x'-z" rotation scheme.
 *
 * @return	calculated pitch component of the given rotation matrix
 */
double DCM_to_Pitch(double DCM[3][3]){
	double x = -asin(DCM[1][2]);
	return asin(DCM[0][2] / cos(x));
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
 * Calculates and returns the roll component of the given rotation matrix,
 * using a y-x'-z" rotation scheme.
 *
 * @return	calculated pitch component of the given rotation matrix
 */
double DCM_to_Roll(double DCM[3][3]){
	return -asin(DCM[1][2]);
}

int main(int argc, char** argv){
	// Hardware configuration
	printf("Arduino Gimbal Controller Test Suite, compiled %s at %s\n", __DATE__, __TIME__);
	srand(0);
	for(long randCount = 0; randCount > 0; randCount++){
		double randroll = M_PI * (rand() / (double) RAND_MAX) - M_PI / 2;
		double randyaw = 2 * M_PI * (rand() / (double) RAND_MAX) - M_PI;
		double randpitch = M_PI * (rand() / (double) RAND_MAX) - M_PI / 2;

		double r_y[3][3] = {{0}};
		r_y[0][0] = cos(randpitch);
		r_y[0][2] = sin(randpitch);
		r_y[1][1] = 1;
		r_y[2][0] = -sin(randpitch);
		r_y[2][2] = cos(randpitch);

		double r_x[3][3] = {{0}};
		r_x[0][0] = 1;
		r_x[1][1] = cos(randroll);
		r_x[1][2] = -sin(randroll);
		r_x[2][1] = sin(randroll);
		r_x[2][2] = cos(randroll);

		double r_z[3][3] = {{0}};
		r_z[0][0] = cos(randyaw);
		r_z[0][1] = -sin(randyaw);
		r_z[1][0] = sin(randyaw);
		r_z[1][1] = cos(randyaw);
		r_z[2][2] = 1;

		double t_DCM[3][3] = {{0}};
		double r_DCM[3][3] = {{0}};
		compose_rotations(t_DCM, r_y, r_x);
		compose_rotations(r_DCM, t_DCM, r_z);

		double result = DCM_to_Yaw(r_DCM);
		if(fabs(result - randyaw) > 0.01){
			printf("Roll: %f\tPitch: %f\tYaw: %f\n", randroll, randpitch, randyaw);
			for(int i = 0; i < 3; i++){
				for(int j = 0; j < 3; j++){
					printf("%f\t", r_x[i][j]);
				}
				printf("\n");
			}
			printf("Calculated: %f\n", result);
			assert(0);
		}
	}
	
	return 0;

}
