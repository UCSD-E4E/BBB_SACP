/**
 * 3D Vector Math Library
 *
 * @author Nathan Hui
 */

/////////////
// Defines //
/////////////

//////////////
// Includes //
//////////////
#include <math.h>

/////////////////////////
// Function Prototypes //
/////////////////////////
void vec3_cross_product(double *a, double *b, double *ret);
double vec3_mag(double *a);
void vec3_norm(double* a);
double vec3_dot(double *a, double *b);

//////////////////////////
// Function Definitions //
//////////////////////////
/**
 * Returns the cross product of vector a and vector b in the array ret.
 * Requires all parameters have three elements
 * @param a   First vector
 * @param b   Second vector
 * @param ret Array in which to store the cross product
 */
void vec3_cross_product(double *a, double *b, double *ret){
	ret[0] = a[1] * b[2] - a[2] * b[1];
	ret[1] = a[2] * b[0] - a[0] * b[2];
	ret[2] = a[0] * b[1] - a[1] * b[0];
	return;
}

/**
 * Calculates the magnitude of the vector.
 * @param  a A vector, whose magnitude will be calculated
 * @return   Magnitude of vector a
 */
double vec3_mag(double *a){
	return sqrt(a[0] * a[0] + a[1]* a[1] + a[2] * a[2]);
}

/**
 * Normalizes the supplied vector.  The vector supplied will be modified to
 * have a magnitude of 1.
 * @param  a Vector to be normalized.
 */
void vec3_norm(double* a){
	double __mag = vec3_mag(a);
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
 * Returns the dot product of the two vectors supplied.  The dot product is
 * equal to the sum of the products of eacn analogous element.  Thus, the dot
 * product of vectors u and v is u_x * v_x + u_y * v_y + u_z * v_z.
 * @param  a First vector
 * @param  b Second vector
 * @return   Dot product of vectors a and b
 */
double vec3_dot(double *a, double *b){
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
