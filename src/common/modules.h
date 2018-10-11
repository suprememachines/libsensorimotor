/* modules.h
 * contains common mathematical functions and helpers */

#ifndef MODULES_H
#define MODULES_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <float.h>
#include <assert.h>
#include <vector>
#include <algorithm>
#include "log_messages.h"

typedef std::vector<double> VectorN;

/* tanh() on a vector_t */
template <typename Vector_t>
void vector_tanh(Vector_t& vec) { for (auto& v : vec) v = tanh(v); }

/* sigmoid function */
double sigmoid(double x);

/* squares the var */
inline double square(double x) { return x * x; }

/* clips values to Interval [-1,+1] */
double clip(double x);

/* clips values to Interval [-ul_limit,+ul_limit] */
double clip(double x, double ul_limit);

/* clips values to Interval [l_limit, u_limit] */
double clip(double x, double l_limit, double u_limit);

/* clip() on a vector_t */
template <typename Vector_t>
void vector_clip(Vector_t& vec) { for (auto& v : vec) v = clip(v); }

/* checks if vector is zero */
template <typename Vector_t>
bool is_vector_zero(Vector_t const& vec) {
    for (auto& v : vec)
        if (v != 0.0) return false;
    return true;
}

template <typename Vector_t>
void print_vector(Vector_t const& vec, const char* name = "", const char* format = "%+4.2f ") {
    printf("%s = [", name);
    for (auto const& el: vec)
        printf(format, el);
    printf("]\n");
}

/* min of 3 arguments */
double fmin3(double x, double y, double z);

/* max of 3 arguments */
double fmax3(double x, double y, double z);

/* computes the minimum of N arguments */
double fminN(double *x, unsigned int N);

/* computes the maximum of N arguments */
double fmaxN(double *x, unsigned int N);

/* computes median-of-three */
double median3(double a, double b, double c);

/* generates a double random value within interval [a,b] */
double random_value(double a, double b);

/* generates a integer random index within interval [0,N[ */
unsigned int random_index(unsigned int N);

/* generates a integer random value within interval [a,b] */
int random_int(int a, int b);

/* generates a pseudo-random double between 0.0 and 0.999... */
double random_value(void);

/* normally distributed random value */
double random_value_norm(const double m, const double s, const double min, const double max);

/* zero mean normal distributed random value with max. 3 sigma */
inline double rand_norm_zero_mean(double sigma) { return random_value_norm(0.0, sigma, -3*sigma, 3*sigma); }

/* random sign */
inline int rand_sign(void) { return (random_index(2) == 0) ? -1 : 1; }

/* returns a random vector of size N with values in [a,b]*/
std::vector<double> random_vector(std::size_t N, double a, double b);

/* multiplies matrix by vector */
void mult_mat_by_vect(double *result_vect, const double *mat, const double *vect, const unsigned int Zeilen, const unsigned int Spalten);
void mult_mat_by_vect(VectorN& result_vect, const VectorN& mat, const VectorN& vect);

/* computes the argument which minimizes the function */
int argmin(double *f, unsigned int N);

/* computes the argument which maximizes the function */
int argmax(double *f, unsigned int N);

/* computes the squared distance of two vectors */
double squared_distance(double *x , double *y, unsigned int length);

/* computes the squared distance of two vectors */
template <typename Vector_Type_A, typename Vector_Type_B>
double squared_distance(Vector_Type_A const& x , Vector_Type_B const& y) {
    assert(x.size() == y.size());
    double d = .0;
    for (std::size_t i = 0; i < x.size(); ++i)
        d += square(x[i] - y[i]);
    return d;
}

/* true is integer is even */
inline bool is_even(unsigned int number) { return (number%2 == 0); }

/* signum */
double sign(double x);

/* floating point modulo, better than fmod */
double modulo(double a, double b);

/* maps arbitrary angles from interval -inf..+inf to -pi..+pi */
double wrap(double angle);

/* maps arbitrary angles from interval -inf..+inf to -pi..+pi by Martin Marmulla */
double wrap2(double angle);

/* unwraps angles of -pi..+pi to -inf..+inf */
double unwrap(double new_angle, double last_angle);

/* checks that value is close to refval by max distance of tolerance */
inline bool close(double value, double refval, double tolerance) { return (fabs(value - refval) < tolerance); }

/* checks for vector that value is close to refval by max distance of tolerance */
template <typename Vector_t>
inline bool close(Vector_t vec, Vector_t refvec, double tolerance) {
    assert(vec.size() == refvec.size());
    for (std::size_t i = 0; i < vec.size();++i)
        if (fabs(vec[i] - refvec[i]) > tolerance)
            return false;
    return true;
}

/* asserts that value is close to refval by max distance of tolerance */
inline void assert_close(double value, double refval, double tolerance, const char* msg) {
    if (not close(value, refval, tolerance))
        err_msg(__FILE__, __LINE__, "%s: value %f not close to %f by tolerance of %f.\n", msg, value, refval, tolerance);
}

/* checks if variable is in the given range [lower, upper]*/
template <typename T>
inline bool in_range(T value, T lower, T upper) { return (lower <= value) and (value <= upper); }

inline void test_range(double value, double lower, double upper, const char* msg)
{
    if (not in_range(value, lower, upper))
        err_msg(__FILE__, __LINE__, "%s: value %f out of range [%f %f].\n", msg, value, lower, upper);
}
inline void test_range(std::size_t value, std::size_t lower, std::size_t upper, const char* msg)
{
    if (not in_range(value, lower, upper))
        err_msg(__FILE__, __LINE__, "%s: value %lu out of range [%lu %lu].\n", msg, value, lower, upper);
}

template <typename Vector_t>
inline void test_range(Vector_t const& values, double lower, double upper, const char* msg)
{
    for (std::size_t i = 0; i < values.size(); ++i)
        if (not in_range(values[i], lower, upper))
            err_msg(__FILE__, __LINE__, "%s: value %f from vector index %u out of range [%f %f].\n", msg, values[i], i, lower, upper);
}

#define assert_in_range(VALUE, LOWER, UPPER)  \
test_range(VALUE, LOWER, UPPER, #VALUE); \

std::string random_string(size_t length);

#endif /*MODULES_H*/
