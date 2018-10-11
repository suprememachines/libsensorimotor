/* modules.cpp */

#include "modules.h"

/* sigmoid function */
double
sigmoid(double x)
{
    return 1.0/(1 + exp(-x));
}

/* clips values to Interval [-1,+1] */
double clip(double x)
{
    if (x > 1.0) return 1.0;
    else if (x < -1.0) return -1.0;
    else return x;
}

/* clips values to Interval [-ul_limit,+ul_limit] */
double clip(double x, double ul_limit)
{
    if (x > ul_limit) return ul_limit;
    else if (x < -ul_limit) return -ul_limit;
    else return x;
}

/* clips values to Interval [l_limit, u_limit] */
double clip(double x, double l_limit, double u_limit)
{
    if (x > u_limit) return u_limit;
    else if (x < l_limit) return l_limit;
    else return x;
}

/* computes the minimum of three arguments */
inline double
fmin3(double x, double y, double z)
{
    return fmin(fmin(x, y), z);
}

/* computes the maximum of three arguments */
inline double
fmax3(double x, double y, double z)
{
    return fmax(fmax(x, y), z);
}

/* computes the minimum of N arguments */
double
fminN(double *x, unsigned int N)
{
    double minX = DBL_MAX;
    for (unsigned int n = 0; n < N; n++)
        if (x[n] < minX) minX = x[n];
    return minX;
}

/* computes the maximum of N arguments */
double
fmaxN(double *x, unsigned int N)
{
    double maxX = -DBL_MAX;
    for (unsigned int n = 0; n < N; n++)
        if (x[n] > maxX) maxX = x[n];
    return maxX;
}

/* computes median-of-three */
double
median3(double a, double b, double c)
{
    if (a > b)
        if (b > c)
            return b;
        else
            if (a > c)
                return c;
            else
                return a;
    else
        if (b < c)
            return b;
        else
            if (a > c)
                return a;
            else
                return c;
}

/* generates a random value within interval [a,b] */
double
random_value(double a, double b)
{
    /* switch interval borders if necessary */
    double temp;
    if (b < a) {
        temp = a;
        a = b;
        b = temp;
    }
    /* generate random value for the given interval */
    return (b - a) * ((double) rand()) / RAND_MAX + a;
}

unsigned int
random_index(unsigned int N)
{
    if (N > 0) return rand() % N;
    else return 0;
}

int random_int(int a, int b) {
    if (a < b)
        return (rand() % (b - a + 1)) + a;
    else if (b < a)
        return (rand() % (a - b + 1)) + b;
    else return a;
}

/* generates a pseudo-random double between 0.0 and 0.999... */
double
random_value(void) { return (double) rand() / (double(RAND_MAX) + 1.0); }

/* normally distributed random value */
double
random_value_norm(const double m, const double s, const double min, const double max)
{
    /* mean m, standard deviation s */
    double x1, x2, w, y1;
    static double y2;
    static int use_last = 0;

    /* use value from previous call */
    if (use_last) {
        y1 = y2;
        use_last = 0;
    }
    else {
        do {
            x1 = 2.0 * random_value() - 1.0;
            x2 = 2.0 * random_value() - 1.0;
            w = x1 * x1 + x2 * x2;
        } while (w >= 1.0);

        w = sqrt((-2.0 * log(w)) / w);
        y1 = x1 * w;
        y2 = x2 * w;
        use_last = 1;
    }

    double ret = ( m + y1 * s );
    if (ret < min) ret = min;
    if (ret > max) ret = max;
    return ret;
}

/* returns a random vector of size N with values in [a,b]*/
std::vector<double> random_vector(std::size_t N, double a, double b) {
    std::vector<double> rvec(N);
    for (std::size_t i = 0; i < N; ++i)
        rvec[i] = random_value(a, b);
    return rvec;
}

/* multiplies matrix by vector */
void
mult_mat_by_vect(double *result_vect, const double *mat, const double *vect, const unsigned int Zeilen, const unsigned int Spalten)
{
    for (unsigned int z = 0; z < Zeilen; z++)
    {
        result_vect[z] = 0;
        for (unsigned int s = 0; s < Spalten; s++)
            result_vect[z] += mat[z * Spalten + s] * vect[s];
    }
}

/* multiplies matrix by vector */
void
mult_mat_by_vect(VectorN& result_vect, const VectorN& mat, const VectorN& vect)
{
    for (std::size_t z = 0; z < result_vect.size(); ++z)
    {
        result_vect[z] = .0;
        for (std::size_t s = 0; s < vect.size(); ++s)
            result_vect[z] += mat[z * vect.size() + s] * vect[s];
    }
}

/* computes the argument which minimizes the function */
int
argmin(double *f, unsigned int N)
{
    double minf = DBL_MAX;
    int arg = -1;
    for (unsigned int n = 0; n < N; n++)
        if (f[n] < minf)
        {
            minf = f[n];
            arg = n;
        }
    return arg;
}

/* computes the argument which maximizes the function */
int
argmax(double *f, unsigned int N)
{
    double maxf = -FLT_MAX;
    int arg = 0;
    for (unsigned int n = 0; n < N; n++)
        if (f[n] > maxf)
        {
            maxf = f[n];
            arg = n;
        }
    return arg;
}

double
squared_distance(double *x , double *y, unsigned int length)
{
    double d = .0;
    for (unsigned int i = 0; i < length; i++)
        d += square(x[i] - y[i]);
    return d;
}

double
sign(double x)
{
    if (x > 0.0) return 1.0;
    else if (x < 0.0) return -1.0;
    else return 0.0;
}

double
modulo(double a, double b)
{
    int result = static_cast<int>( a / b );
    return a - static_cast<double>( result ) * b;
}

double
wrap(double angle)
{
    if (angle > M_PI)
        return modulo(angle+M_PI, 2*M_PI) - M_PI;
    else if (angle < -M_PI)
        return modulo(angle-M_PI, 2*M_PI) + M_PI;
    else
        return angle;
}

/* by Martin Marmulla */
double
wrap2(double angle)
{
    return angle - 2*M_PI * floor((angle + M_PI) / (2*M_PI));
}

double
unwrap(double new_angle, double last_angle)
{
    double diff = modulo(new_angle - last_angle, 2*M_PI);
    last_angle += diff;

    if (diff < -M_PI)
        last_angle += 2*M_PI;
    else if (diff > M_PI)
        last_angle -= 2*M_PI;

    return last_angle;
}

std::string random_string(size_t length)
{
    auto randchar = []() -> char
    {
        const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[rand() % max_index];
    };
    std::string str(length,0);
    std::generate_n(str.begin(), length, randchar);
    return str;
}

