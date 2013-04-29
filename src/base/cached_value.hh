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

#ifndef __BASE_CACHED_VALUE_HH__
#define __BASE_CACHED_VALUE_HH__

/**
 * Abstract container of a cached value
 *
 * The cached container simplifies implementing wrappers for values
 * that can be cached. For example, when running gem5 in virtualized
 * mode, it can be used to cache target register values. In such a
 * case, an entry into the VM would invalidate the value and the first
 * access to the container would read the value from the VM.
 *
 * For example, the following code could be used to cache a MSR
 * register:
 * \code {.cc}
 *   class CachedMSR : public CachedValueBase<uint64_t>
 *   {
 *     public:
 *       CachedMSR(KvmX86CPU &cpu, uint32_t index) : _cpu(cpu), _index(index) {}
 *
 *     protected:
 *       const uint64_t getNonCaching() const { return _cpu.getMSR(_index); }
 *
 *     private:
 *       KvmX86CPU &_cpu;
 *       const uint32_t _index;
 *   };
 * \endcode
 *
 * The value can be accessed using the get() method or through an
 * implicit conversion to the stored type. If the container doesn't
 * hold a cached value, it will automatically all refresh() which uses
 * getNonCaching() to update the value. The stored value can be
 * invalidated by calling the invalidate() method.
 */
template<typename T>
class CachedValueBase
{
  public:
    /** Construct a new cached value that is invalid. */
    CachedValueBase() : _valid(false) {}
    /**
     * Construct a new cached value from an existing value.
     *
     * A CachedValue constructed with using this constructor will
     * initially be valid.
     *
     * @param value Value to initialize the container with.
     */
    CachedValueBase(const T &value) : _valid(true), _value(value) {}
    virtual ~CachedValueBase() {}

    /** Implicitly get the value stored in the cache. */
    operator const T &() const { return get(); }

    /**
     * Get a reference to the cached value.
     *
     * If the cached value isn't valid, it is automatically updated
     * through a call to refresh(), which in turn calls
     * getNonCaching() to retrieve the new value.
     *
     * @return a constant reference to the cached value.
     */
    const T &get() const {
        if (!_valid)
            refresh();

        return _value;
    }

    /** Is the cached copy valid? */
    bool valid() const { return _valid; }

    /**
     * Force the cached value to be updated.
     *
     * The cached value is updated irregardless of whether it is
     * currently valid or not. The new value is retrieved by calling
     * getNonCaching().
     */
    void refresh() const {
        _value = getNonCaching();
        _valid = true;
    }

    /** Invalidate the cached value. */
    void invalidate() const { _valid = false; }

  protected:
    /**
     * Retrieve a new version of the value cached by this container.
     *
     * @return New value to store in the cache.
     */
    virtual const T getNonCaching() const = 0;

  private:
    /** Is the cached copy valid? */
    mutable bool _valid;
    /** Cached data */
    mutable T _value;
};

#endif

