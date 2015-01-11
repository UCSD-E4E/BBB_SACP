/**
 * 3D Vector Math Library
 * 
 * @author Nathan Hui
 */

#ifndef _V3MATH_H_
#define _V3MATH_H_

double vec3_mag(double a[3]);
void vec3_norm(double* a);
double vec3_dot(double a[3], double b[3]);
void vec3_cross_product(double *a, double *b, double *ret);


#endif // _V3MATH_H_
