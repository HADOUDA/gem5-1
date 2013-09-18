/*
 * Copyright (c) 2013 Andreas Sandberg
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Sandberg
 */

#ifndef __SOFTFLOAT_HH__
#define __SOFTFLOAT_HH__

#include <cstdint>
#include <ostream>

#include "softfloat/softfloat.hh"
#include "softfloat/softfloatx80.hh"

/**
 * Wrapper around SoftFloat's floating point environment.
 */
class SoftFloatEnv
{
  public:
    SoftFloatEnv();

    /**
     * Get the global software floating point environment.
     *
     * @todo Store this in the TLS.
     */
    static SoftFloatEnv &get() {
        return global_env;
    }

    /**
     * SoftFloat's representation of the floating point environment.
     */
    SoftFloat::float_status_t status;

  private:
    static SoftFloatEnv global_env;
};

/**
 * Templatized SoftFloat wrapper that overloads common C++ operators.
 *
 * This class takes an implementation class, which provides a unified
 * interface to the underlying SoftFloat library, and the underlying
 * float representation as its template parameters. The purpose of
 * this class is to provide a unified interface for the different
 * software float types (32-bit, 64-bit, and 80-bit).
 *
 * @see SoftFloat80Impl
 * @see SoftFloat80
 */
template<class T, class R>
class GenericSoftFloat
{
  public:
    GenericSoftFloat<T, R>() {
        repr = T::from_int32(0);
    }

    GenericSoftFloat<T, R>(const GenericSoftFloat<T, R> &val) {
        repr = val.repr;
    }

    GenericSoftFloat<T, R>(const R &val) {
        repr = val;
    }

    GenericSoftFloat<T, R>(const double val) {
        repr = T::from_float64(*(const SoftFloat::float64 *)&val);
    }

    GenericSoftFloat<T, R>(const float val) {
        repr = T::from_float32(*(const SoftFloat::float32 *)&val);
    }

    GenericSoftFloat<T, R>(const int64_t val) {
        repr = T::from_int64(val);
    }

    GenericSoftFloat<T, R>(const int32_t val) {
        repr = T::from_int32(val);
    }

    static GenericSoftFloat<T, R> load(const void *mem) {
        return T::from_mem((const uint8_t *)mem);
    }

    void store(void *mem) const {
        T::to_mem((uint8_t *)mem, repr);
    }

    operator double() const {
	const SoftFloat::float64 val(T::to_float64(repr));
        return *(const double *)&val;
    }

    operator float() const {
	const SoftFloat::float32 val(T::to_float32(repr));
        return *(const float *)&val;
    }

    GenericSoftFloat<T, R> &operator+=(const GenericSoftFloat<T, R> &rhs) {
        repr = T::add(repr, rhs.repr);
	return *this;
    }

    GenericSoftFloat<T, R> &operator-=(const GenericSoftFloat<T, R> &rhs) {
        repr = T::sub(repr, rhs.repr);
	return *this;
    }

    GenericSoftFloat<T, R> &operator*=(const GenericSoftFloat<T, R> &rhs) {
        repr = T::mul(repr, rhs.repr);
	return *this;
    }

    GenericSoftFloat<T, R> &operator/=(const GenericSoftFloat<T, R> &rhs) {
        repr = T::div(repr, rhs.repr);
	return *this;
    }

    GenericSoftFloat<T, R> sqrt() const {
        return T::sqrt(repr);
    }

    bool isNan() const {
        return T::is_nan(repr);
    }

    bool isSignalingNan() const {
        return T::is_signaling_nan(repr);
    }

    SoftFloat::float_class_t floatClass() const {
        return T::float_class(repr);
    }

    static int compare(const GenericSoftFloat<T, R> &lhs,
                       const GenericSoftFloat<T, R> &rhs) {
        return T::compare(lhs.repr, rhs.repr);
    }

    static int compare_quiet(const GenericSoftFloat<T, R> &lhs,
                             const GenericSoftFloat<T, R> &rhs) {
        return T::compare_quiet(lhs.repr, rhs.repr);
    }

  private:
    R repr;
};

template<class T, class R>
inline GenericSoftFloat<T, R> operator+(const GenericSoftFloat<T, R> &lhs,
                                        const GenericSoftFloat<T, R> &rhs)
{
    return GenericSoftFloat<T, R>::add(lhs, rhs);
}

template<class T, class R>
inline GenericSoftFloat<T, R> operator-(const GenericSoftFloat<T, R> &lhs,
                                        const GenericSoftFloat<T, R> &rhs)
{
    return GenericSoftFloat<T, R>::sub(lhs, rhs);
}

template<class T, class R>
inline GenericSoftFloat<T, R> operator*(const GenericSoftFloat<T, R> &lhs,
                                        const GenericSoftFloat<T, R> &rhs)
{
    return GenericSoftFloat<T, R>::mul(lhs, rhs);
}

template<class T, class R>
inline GenericSoftFloat<T, R> operator/(const GenericSoftFloat<T, R> &lhs,
                                        const GenericSoftFloat<T, R> &rhs)
{
    return GenericSoftFloat<T, R>::div(lhs, rhs);
}

template<class T, class R>
inline bool operator<=(const GenericSoftFloat<T, R> &lhs,
                       const GenericSoftFloat<T, R> &rhs)
{
    const int relation(GenericSoftFloat<T, R>::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_less ||
        relation == SoftFloat::float_relation_equal;
}

template<class T, class R>
inline bool operator>=(const GenericSoftFloat<T, R> &lhs,
                       const GenericSoftFloat<T, R> &rhs)
{
    const int relation(GenericSoftFloat<T, R>::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_greater ||
        relation == SoftFloat::float_relation_equal;
}

template<class T, class R>
inline bool operator<(const GenericSoftFloat<T, R> &lhs,
                      const GenericSoftFloat<T, R> &rhs)
{
    const int relation(GenericSoftFloat<T, R>::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_less;
}

template<class T, class R>
inline bool operator>(const GenericSoftFloat<T, R> &lhs,
                      const GenericSoftFloat<T, R> &rhs)
{
    const int relation(GenericSoftFloat<T, R>::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_greater;
}

template<class T, class R>
inline bool operator==(const GenericSoftFloat<T, R> &lhs,
                       const GenericSoftFloat<T, R> &rhs)
{
    const int relation(GenericSoftFloat<T, R>::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_equal;
}

template<class T, class R>
inline bool operator!=(const GenericSoftFloat<T, R> &lhs,
                       const GenericSoftFloat<T, R> &rhs)
{
    const int relation(GenericSoftFloat<T, R>::compare_quiet(lhs, rhs));
    return relation != SoftFloat::float_relation_equal &&
        relation != SoftFloat::float_relation_unordered ;
}

template<class T, class R>
inline std::ostream &operator<<(std::ostream &os,
                                const GenericSoftFloat<T, R> &obj)
{
    os << (double)obj;
    return os;
}


/**
 * Wrapper for the SoftFloat::floatx80 type.
 *
 * @see GenericSoftFloat
 */
struct SoftFloat80Impl
{
    typedef SoftFloat::floatx80 Repr;

    static Repr from_floatx80(const SoftFloat::floatx80 f) {
        return f;
    }

    static Repr from_float64(const SoftFloat::float64 f) {
        return SoftFloat::float64_to_floatx80(f, status());
    }

    static Repr from_float32(const SoftFloat::float32 f) {
        return SoftFloat::float32_to_floatx80(f, status());
    }

    static Repr from_int64(const int64_t val) {
        return SoftFloat::int64_to_floatx80(val);
    }

    static Repr from_int32(const int32_t val) {
        return SoftFloat::int32_to_floatx80(val);
    }

    static Repr from_mem(const uint8_t *mem) {
        Repr repr;

        repr.fraction = *(uint64_t *)mem;
        repr.exp = *(uint16_t *)(mem + 8);
        
        return repr;
    }

    static SoftFloat::floatx80 to_floatx80(const Repr repr) {
	return repr;
    }

    static SoftFloat::float64 to_float64(const Repr repr) {
	return SoftFloat::floatx80_to_float64(repr, status());
    }

    static SoftFloat::float32 to_float32(const Repr repr) {
	return SoftFloat::floatx80_to_float32(repr, status());
    }

    static void to_mem(uint8_t *mem, const Repr repr) {
        *(uint64_t *)mem = repr.fraction;
        *(uint16_t *)(mem + 8) = repr.exp;
    }

    static Repr add(const Repr lhs, const Repr rhs) {
        return SoftFloat::floatx80_add(lhs, rhs, status());
    }

    static Repr sub(const Repr lhs, const Repr rhs) {
        return SoftFloat::floatx80_sub(lhs, rhs, status());
    }

    static Repr mul(const Repr lhs, const Repr rhs) {
        return SoftFloat::floatx80_mul(lhs, rhs, status());
    }

    static Repr div(const Repr lhs, const Repr rhs) {
        return SoftFloat::floatx80_div(lhs, rhs, status());
    }

    static Repr sqrt(const Repr val) {
        return SoftFloat::floatx80_sqrt(val, status());
    }

    static int compare(const Repr lhs, const Repr rhs) {
        return SoftFloat::floatx80_compare(lhs, rhs, status());
    }

    static int compare_quiet(const Repr lhs, const Repr rhs) {
        return SoftFloat::floatx80_compare_quiet(lhs, rhs, status());
    }

    static bool is_nan(const Repr repr) {
        return SoftFloat::floatx80_is_nan(repr);
    }

    static bool is_signaling_nan(const Repr repr) {
        return SoftFloat::floatx80_is_signaling_nan(repr);
    }

    static SoftFloat::float_class_t float_class(const Repr repr) {
        return SoftFloat::floatx80_class(repr);
    }

    static SoftFloat::float_status_t &status() {
        return SoftFloatEnv::get().status;
    }
};

/**
 * Wrapper around SoftFloat's floatx80 representation.
 *
 * This class wraps the floatx80 implementation in a
 * GenericSoftFloat. It overloads most common floating point
 * operators. By default, it uses the global SoftFloatEnv to represent
 * status flags and control rounding modes.
 */
typedef GenericSoftFloat<SoftFloat80Impl, SoftFloat80Impl::Repr> SoftFloat80;


#endif
