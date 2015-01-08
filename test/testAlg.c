/**
 * Testing suite for algorithms
 */

//////////////
// Includes //
//////////////
#include <stdio.h>
#include <assert.h>

/////////////
// Defines //
/////////////
#define MAJVER		0
#define MINVER		2

//////////////////////
// Global Variables //
//////////////////////

/////////////////////////
// Function Prototypes //
/////////////////////////
void compose_rotations(double ret[][3], double rot1[][3], double rot2[][3]);
void invert_matrix(double ret[][3], double mat[][3]);


//////////////////////////
// Function Definitions //
//////////////////////////
/**
 * Inverts the given matrix and stores it in ret[][3].
 *
 * @param ret	Pointer to the array to populate with the new rotation matrix
 * @param mat	Pointer to the array containing the given matrix
 */
void invert_matrix(double ret[][3], double mat[][3]){
	double det =	mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) -
					mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
					mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
	ret[0][0] = (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) / det;
	ret[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) / det;
	ret[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / det;
	ret[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) / det;
	ret[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / det;
	ret[1][2] = (mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]) / det;
	ret[2][0] = (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]) / det;
	ret[2][1] = (mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1]) / det;
	ret[2][2] = (mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]) / det;
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

int main(int argc, char** argv){
	// Hardware configuration
	printf("Arduino Gimbal Controller v%d.%d, compiled %s at %s\n", MAJVER, MINVER, __DATE__, __TIME__);

	// Parse parse_DCM
	int goodRead = 1;
	do{
		double parse_DCM[3][3] = {{0}};

		int ret = 0;

		ret = scanf("%f %f %f", &parse_DCM[0][0], &parse_DCM[0][1], &parse_DCM[0][2]);
		if(ret != 3){
			goodRead = 0;
			continue;
		}
		ret = scanf("%f %f %f", &parse_DCM[1][0], &parse_DCM[1][1], &parse_DCM[1][2]);
		if(ret != 3){
			goodRead = 0;
			continue;
		}
		ret = scanf("%f %f %f", &parse_DCM[2][0], &parse_DCM[2][1], &parse_DCM[2][2]);
		if(ret != 3){
			goodRead = 0;
			continue;
		}
	} while(0);
	if(goodRead){
		// atomically pass parse_DCM to cur_DCM
	}

	printf("%3.3f\t%3.3f\t%3.3f\n", parse_DCM[0][0], parse_DCM[0][1], parse_DCM[0][2]);
	printf("%3.3f\t%3.3f\t%3.3f\n", parse_DCM[1][0], parse_DCM[1][1], parse_DCM[1][2]);
	printf("%3.3f\t%3.3f\t%3.3f\n", parse_DCM[2][0], parse_DCM[2][1], parse_DCM[2][2]);

	return 0;

}
