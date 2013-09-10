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
 * Wrapper around SoftFloat's floatx80 representation.
 *
 * This class wraps the floatx80 implementation in SoftFloat. It
 * overloads most common floating point operators. By default, it uses
 * the global SoftFloatEnv to represent status flags and control
 * rounding modes.
 */
class SoftFloat80
{
  public:
    SoftFloat80() {
        repr = SoftFloat::int32_to_floatx80(0);
    }

    SoftFloat80(const SoftFloat::floatx80 &val) {
        repr = val;
    }

    SoftFloat80(const SoftFloat80 &val) {
        repr = val.repr;
    }

    SoftFloat80(const double val) {
        repr = SoftFloat::float64_to_floatx80(*(const SoftFloat::float64 *)&val, status());
    }

    SoftFloat80(const float val) {
        repr = SoftFloat::float32_to_floatx80(*(const SoftFloat::float32 *)&val, status());
    }

    SoftFloat80(const int64_t val) {
        repr = SoftFloat::int64_to_floatx80(val);
    }

    SoftFloat80(const int32_t val) {
        repr = SoftFloat::int32_to_floatx80(val);
    }

    /**
     * Load an 80-bit floating point value from memory.
     *
     * @param mem Pointer to an 80-bit float.
     */
    static SoftFloat80 load_raw(const void *mem);
    /**
     * Store an 80-bit floating point value to memory.
     *
     * @param mem Pointer to target in memory.
     */
    void store_raw(void *mem) const;

    operator double() const {
	const SoftFloat::float64 val(SoftFloat::floatx80_to_float64(repr, status()));
        return *(const double *)&val;
    }

    operator float() const {
	const SoftFloat::float32 val(SoftFloat::floatx80_to_float32(repr, status()));
        return *(const double *)&val;
    }

    SoftFloat80 &operator+=(const SoftFloat80 &rhs) {
        repr = SoftFloat::floatx80_add(repr, rhs.repr, status());
	return *this;
    }

    SoftFloat80 &operator-=(const SoftFloat80 &rhs) {
        repr = SoftFloat::floatx80_sub(repr, rhs.repr, status());
	return *this;
    }

    SoftFloat80 &operator*=(const SoftFloat80 &rhs) {
        repr = SoftFloat::floatx80_mul(repr, rhs.repr, status());
	return *this;
    }

    SoftFloat80 &operator/=(const SoftFloat80 &rhs) {
        repr = SoftFloat::floatx80_div(repr, rhs.repr, status());
	return *this;
    }

    SoftFloat80 sqrt() const {
        return SoftFloat::floatx80_sqrt(repr, status());
    }

    bool isNan() const {
        return SoftFloat::floatx80_is_nan(repr);
    }

    bool isSignalingNan() const {
        return SoftFloat::floatx80_is_signaling_nan(repr);
    }

    SoftFloat::float_class_t floatClass() const {
        return SoftFloat::floatx80_class(repr);
    }

    static SoftFloat80 add(const SoftFloat80 &lhs, const SoftFloat80 &rhs) {
        return SoftFloat::floatx80_add(lhs.repr, rhs.repr, status());
    }

    static SoftFloat80 sub(const SoftFloat80 &lhs, const SoftFloat80 &rhs) {
        return SoftFloat::floatx80_sub(lhs.repr, rhs.repr, status());
    }

    static SoftFloat80 mul(const SoftFloat80 &lhs, const SoftFloat80 &rhs) {
        return SoftFloat::floatx80_mul(lhs.repr, rhs.repr, status());
    }

    static SoftFloat80 div(const SoftFloat80 &lhs, const SoftFloat80 &rhs) {
        return SoftFloat::floatx80_div(lhs.repr, rhs.repr, status());
    }

    static int compare(const SoftFloat80 &lhs, const SoftFloat80 &rhs) {
        return SoftFloat::floatx80_compare(lhs.repr, rhs.repr, status());
    }

    static int compare_quiet(const SoftFloat80 &lhs, const SoftFloat80 &rhs) {
        return SoftFloat::floatx80_compare_quiet(lhs.repr, rhs.repr, status());
    }

  private:
    static SoftFloat::float_status_t &status() { return SoftFloatEnv::get().status; }

    SoftFloat::floatx80 repr;
};

inline SoftFloat80 operator+(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    return SoftFloat80::add(lhs, rhs);
}

inline SoftFloat80 operator-(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    return SoftFloat80::sub(lhs, rhs);
}

inline SoftFloat80 operator*(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    return SoftFloat80::mul(lhs, rhs);
}

inline SoftFloat80 operator/(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    return SoftFloat80::div(lhs, rhs);
}

inline bool operator<=(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    const int relation(SoftFloat80::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_less ||
        relation == SoftFloat::float_relation_equal;
}

inline bool operator>=(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    const int relation(SoftFloat80::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_greater ||
        relation == SoftFloat::float_relation_equal;
}

inline bool operator<(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    const int relation(SoftFloat80::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_less;
}

inline bool operator>(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    const int relation(SoftFloat80::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_greater;
}

inline bool operator==(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    const int relation(SoftFloat80::compare_quiet(lhs, rhs));
    return relation == SoftFloat::float_relation_equal;
}

inline bool operator!=(const SoftFloat80 &lhs, const SoftFloat80 &rhs)
{
    const int relation(SoftFloat80::compare_quiet(lhs, rhs));
    return relation != SoftFloat::float_relation_equal &&
        relation != SoftFloat::float_relation_unordered ;
}

inline std::ostream &operator<<(std::ostream &os, const SoftFloat80 &obj)
{
    os << (double)obj;
    return os;
}


#endif
