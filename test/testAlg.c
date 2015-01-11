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
#include <time.h>

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
					mat[1][0] * (mat[0][1] * mat[2][2] - mat[2][1] * mat[0][2]) +
					mat[2][0] * (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]);
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

	double DCM[3][3];
	double inv_DCM[3][3];
	double res[3][3] = {{0}};
	const double REF[3][3] =    {{1, 0, 0},
								{0, 1, 0},
								{0, 0, 1}};
	srand (time(NULL));
	for(int count = 0; count < 1000000; count++){
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				DCM[i][j] = (double)rand() / rand();
				res[i][j] = 0;
			}
		}
		invert_matrix(inv_DCM, DCM);
		compose_rotations(res, DCM, inv_DCM);
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3;j++){
				if(fabs(res[i][j] - REF[i][j]) > 0.01){
					printf("Failed on test %d\n", count);
					for(int a = 0; a < 3; a++){
						printf("%f\t%f\t%f\n", DCM[a][0], DCM[a][1], DCM[a][2]);
					}
					printf("\n");
					for(int a = 0; a < 3; a++){
						printf("%f\t%f\t%f\n", inv_DCM[a][0], inv_DCM[a][1], inv_DCM[a][2]);
					}
					printf("\n");
					for(int a = 0; a < 3; a++){
						printf("%f\t%f\t%f\n", res[a][0], res[a][1], res[a][2]);
					}
					printf("%d, %d: %f, %f => %f\n", i, j, res[i][j], REF[i][j], fabs(res[i][j] - REF[i][j]));
					assert(0);
				}
			}
		}
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				res[i][j] = 0;
			}
		}
		compose_rotations(res, inv_DCM, DCM);
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3;j++){
				if(fabs(res[i][j] - REF[i][j]) > 0.01){
					printf("Failed on test %d\n", count);
					for(int a = 0; a < 3; a++){
						printf("%f\t%f\t%f\n", DCM[a][0], DCM[a][1], DCM[a][2]);
					}
					printf("\n");
					for(int a = 0; a < 3; a++){
						printf("%f\t%f\t%f\n", inv_DCM[a][0], inv_DCM[a][1], inv_DCM[a][2]);
					}
					printf("\n");
					for(int a = 0; a < 3; a++){
						printf("%f\t%f\t%f\n", res[a][0], res[a][1], res[a][2]);
					}
					printf("%d, %d: %f, %f => %f\n", i, j, res[i][j], REF[i][j], fabs(res[i][j] - REF[i][j]));
					assert(0);
				}
			}
		}
	}
	return 0;

}
