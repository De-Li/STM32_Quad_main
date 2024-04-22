#pragma once
/*
  allow for builds with either single or double precision EKF
 */
#include <math.h>
#include <float.h>

/*
  capital F is used to denote the chosen float type (float or double)
 */
/*
 * @brief: Check whether a float is zero
 */
inline bool is_zero(const float x) {
    return fabsf(x) < FLT_EPSILON;
}

/*
 * @brief: Check whether a double is zero
 */
inline bool is_zero(const double x) {
	return fabsf(static_cast<float>(x)) < FLT_EPSILON;
}

#undef MIN
template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
{
    return one < two ? one : two;
}

#undef MAX
template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two)
{
    return one > two ? one : two;
}
