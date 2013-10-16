/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Andreas Hansson
 */

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/user.h>
#include <fcntl.h>
#include <unistd.h>
#include <zlib.h>

#include <cerrno>
#include <climits>
#include <cstdio>
#include <iostream>
#include <string>
#include <utility>

#include "base/trace.hh"
#include "debug/BusAddrRanges.hh"
#include "debug/Checkpoint.hh"
#include "mem/abstract_mem.hh"
#include "mem/physical.hh"

using namespace std;

size_t BackingStore::huge_page_size = 0;

BackingStore::BackingStore(AddrRange range)
    : _range(range),
      _mem(NULL),
      alloc_size(NULL)
{
}

BackingStore::BackingStore(BackingStore &&other)
    : _range(other._range),
      _mem(other._mem),
      alloc_size(other.alloc_size)
{
    other._mem = NULL;
    other.alloc_size = NULL;
}

BackingStore::~BackingStore()
{
    if (_mem)
        deallocate();
}

BackingStore &
BackingStore::operator=(BackingStore &&rhs)
{
    if (_mem)
        deallocate();

    _mem = std::move(rhs._mem);
    _range = std::move(rhs._range);
    alloc_size = std::move(rhs.alloc_size);

    rhs._mem = NULL;
    rhs.alloc_size = NULL;

    return *this;
}

void
BackingStore::allocate()
{
    if (huge_page_size)
        allocate_huge(huge_page_size);
    else
        allocate_small();
}

void
BackingStore::allocate_small()
{
    if (!allocate(_range.size(), MAP_ANON | MAP_PRIVATE)) {
        perror("mmap");
        fatal("Could not mmap %d bytes for range %s!\n", _range.size(),
              _range.to_string());
    }
}

void
BackingStore::allocate_huge(size_t huge_size)
{
    size_t size(_range.size());
    size_t pages((size / huge_size) + (size % huge_size ? 1 : 0));

    DPRINTF(BusAddrRanges, "Trying to allocate %s bytes using huge pages (%i)\n",
            _range.size(), huge_size);

    if (!allocate(pages * huge_size, MAP_ANON | MAP_PRIVATE | MAP_HUGETLB)) {
        perror("mmap");
        warn("Failed to allocate huge pages for range %s. Falling back to "
             "normal pages.",
             _range.to_string());

        allocate_small();
    }
}

bool
BackingStore::allocate(size_t size, int flags)
{
    if (_mem)
        panic("Trying to re-allocate backing store without deallocate\n");

    alloc_size = size;
    _mem = mmap(NULL, size, PROT_READ | PROT_WRITE, flags, -1, 0);

    if (_mem == MAP_FAILED)
        return false;
    else
        return true;
}

void
BackingStore::deallocate()
{
    if (!_mem)
        panic("Trying to de-allocate backing store without allocation\n");

    if (munmap(_mem, alloc_size) == -1) {
        perror("munmap");
        warn("munmap for range %s failed, continuing anyway.\n",
             _range.to_string());
    }

    _mem = NULL;
    alloc_size = 0;
}

void
BackingStore::serialize(ostream& os, unsigned int store_id)
{
    // we cannot use the address range for the name as the
    // memories that are not part of the address map can overlap
    const string filename(name() + ".store" + to_string(store_id) + ".pmem");
    const long range_size(_range.size());

    if (!_mem) {
        panic("Can't serialize backing store for range %s, "
              "store not allocated.\n",
              _range.to_string());
    }

    DPRINTF(Checkpoint, "Serializing physical memory %s with size %d\n",
            filename, range_size);

    SERIALIZE_SCALAR(filename);
    SERIALIZE_SCALAR(range_size);

    // write memory file
    string filepath = Checkpoint::dir() + "/" + filename.c_str();
    int fd = creat(filepath.c_str(), 0664);
    if (fd < 0) {
        perror("creat");
        fatal("Can't open physical memory checkpoint file '%s'\n",
              filename);
    }

    gzFile compressed_mem = gzdopen(fd, "wb");
    if (compressed_mem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
              filename);

    uint64_t pass_size = 0;

    // gzwrite fails if (int)len < 0 (gzwrite returns int)
    for (uint64_t written = 0; written < range_size; written += pass_size) {
        pass_size = (uint64_t)INT_MAX < (range_size - written) ?
            (uint64_t)INT_MAX : (range_size - written);

        if (gzwrite(compressed_mem, (uint8_t *)_mem + written,
                    (unsigned int) pass_size) != (int) pass_size) {
            fatal("Write failed on physical memory checkpoint file '%s'\n",
                  filename);
        }
    }

    // close the compressed stream and check that the exit status
    // is zero
    if (gzclose(compressed_mem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);

}

void
BackingStore::unserialize(Checkpoint* cp, const string& section)
{
    const uint32_t chunk_size = 16384;

    string filename;
    UNSERIALIZE_SCALAR(filename);
    string filepath = cp->cptDir + "/" + filename;

    // mmap memoryfile
    int fd = open(filepath.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        fatal("Can't open physical memory checkpoint file '%s'", filename);
    }

    gzFile compressed_mem = gzdopen(fd, "rb");
    if (compressed_mem == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
              filename);

    long range_size;
    UNSERIALIZE_SCALAR(range_size);

    DPRINTF(Checkpoint, "Unserializing physical memory %s with size %d\n",
            filename, range_size);

    if (range_size != _range.size())
        fatal("Memory range size has changed! Saw %lld, expected %lld\n",
              range_size, _range.size());

    uint64_t curr_size = 0;
    long* temp_page = new long[chunk_size];
    long* pmem_current;
    uint32_t bytes_read;
    while (curr_size < _range.size()) {
        bytes_read = gzread(compressed_mem, temp_page, chunk_size);
        if (bytes_read == 0)
            break;

        assert(bytes_read % sizeof(long) == 0);

        for (uint32_t x = 0; x < bytes_read / sizeof(long); x++) {
            // Only copy bytes that are non-zero, so we don't give
            // the VM system hell
            if (*(temp_page + x) != 0) {
                pmem_current = (long*)((uint8_t*)_mem +
                                       curr_size + x * sizeof(long));
                *pmem_current = *(temp_page + x);
            }
        }
        curr_size += bytes_read;
    }

    delete[] temp_page;

    if (gzclose(compressed_mem))
        fatal("Close failed on physical memory checkpoint file '%s'\n",
              filename);
}

void
BackingStore::setHugePageSize(size_t size)
{
    if (huge_page_size)
        fatal("Huge page size already set.\n");

    huge_page_size = size;
}

PhysicalMemory::PhysicalMemory(const string& _name,
                               const vector<AbstractMemory*>& _memories) :
    _name(_name), size(0)
{
    // add the memories from the system to the address map as
    // appropriate
    for (vector<AbstractMemory*>::const_iterator m = _memories.begin();
         m != _memories.end(); ++m) {
        // only add the memory if it is part of the global address map
        if ((*m)->isInAddrMap()) {
            memories.push_back(*m);

            // calculate the total size once and for all
            size += (*m)->size();

            // add the range to our interval tree and make sure it does not
            // intersect an existing range
            if (addrMap.insert((*m)->getAddrRange(), *m) == addrMap.end())
                fatal("Memory address range for %s is overlapping\n",
                      (*m)->name());
        } else {
            DPRINTF(BusAddrRanges,
                    "Skipping memory %s that is not in global address map\n",
                    (*m)->name());
            // this type of memory is used e.g. as reference memory by
            // Ruby, and they also needs a backing store, but should
            // not be part of the global address map

            // simply do it independently, also note that this kind of
            // memories are allowed to overlap in the logic address
            // map
            vector<AbstractMemory*> unmapped_mems;
            unmapped_mems.push_back(*m);
            createBackingStore((*m)->getAddrRange(), unmapped_mems);
        }
    }

    // iterate over the increasing addresses and chunks of contigous
    // space to be mapped to backing store, also remember what
    // memories constitute the range so we can go and find out if we
    // have to init their parts to zero
    vector<AddrRange> intlv_ranges;
    vector<AbstractMemory*> curr_memories;
    for (AddrRangeMap<AbstractMemory*>::const_iterator r = addrMap.begin();
         r != addrMap.end(); ++r) {
        // simply skip past all memories that are null and hence do
        // not need any backing store
        if (!r->second->isNull()) {
            // if the range is interleaved then save it for now
            if (r->first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r->first)) {
                    AddrRange merged_range(intlv_ranges);
                    createBackingStore(merged_range, curr_memories);
                    intlv_ranges.clear();
                    curr_memories.clear();
                }
                intlv_ranges.push_back(r->first);
                curr_memories.push_back(r->second);
            } else {
                vector<AbstractMemory*> single_memory;
                single_memory.push_back(r->second);
                createBackingStore(r->first, single_memory);
            }
        }
    }

    // if there is still interleaved ranges waiting to be merged, go
    // ahead and do it
    if (!intlv_ranges.empty()) {
        AddrRange merged_range(intlv_ranges);
        createBackingStore(merged_range, curr_memories);
    }
}

void
PhysicalMemory::createBackingStore(AddrRange range,
                                   const vector<AbstractMemory*>& _memories)
{
    if (range.interleaved())
        panic("Cannot create backing store for interleaved range %s\n",
              range.to_string());

    // perform the actual mmap
    DPRINTF(BusAddrRanges, "Creating backing store for range %s with size %d\n",
            range.to_string(), range.size());

    // Create a backing store and add it to the list of backing stores
    // so we can checkpoint it and unmap it appropriately
    backingStore.push_back(BackingStore(range));
    BackingStore &bs(*backingStore.rbegin());
    bs.allocate();

    // point the memories to their backing store, and if requested,
    // initialize the memory range to 0
    for (vector<AbstractMemory*>::const_iterator m = _memories.begin();
         m != _memories.end(); ++m) {
        DPRINTF(BusAddrRanges, "Mapping memory %s to backing store\n",
                (*m)->name());
        (*m)->setBackingStore(bs.get());
    }
}

PhysicalMemory::~PhysicalMemory()
{
}

bool
PhysicalMemory::isMemAddr(Addr addr) const
{
    // see if the address is within the last matched range
    if (!rangeCache.contains(addr)) {
        // lookup in the interval tree
        AddrRangeMap<AbstractMemory*>::const_iterator r = addrMap.find(addr);
        if (r == addrMap.end()) {
            // not in the cache, and not in the tree
            return false;
        }
        // the range is in the tree, update the cache
        rangeCache = r->first;
    }

    assert(addrMap.find(addr) != addrMap.end());

    // either matched the cache or found in the tree
    return true;
}

AddrRangeList
PhysicalMemory::getConfAddrRanges() const
{
    // this could be done once in the constructor, but since it is unlikely to
    // be called more than once the iteration should not be a problem
    AddrRangeList ranges;
    vector<AddrRange> intlv_ranges;
    for (AddrRangeMap<AbstractMemory*>::const_iterator r = addrMap.begin();
         r != addrMap.end(); ++r) {
        if (r->second->isConfReported()) {
            // if the range is interleaved then save it for now
            if (r->first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r->first)) {
                    ranges.push_back(AddrRange(intlv_ranges));
                    intlv_ranges.clear();
                }
                intlv_ranges.push_back(r->first);
            } else {
                // keep the current range
                ranges.push_back(r->first);
            }
        }
    }

    // if there is still interleaved ranges waiting to be merged,
    // go ahead and do it
    if (!intlv_ranges.empty()) {
        ranges.push_back(AddrRange(intlv_ranges));
    }

    return ranges;
}

void
PhysicalMemory::access(PacketPtr pkt)
{
    assert(pkt->isRequest());
    Addr addr = pkt->getAddr();
    AddrRangeMap<AbstractMemory*>::const_iterator m = addrMap.find(addr);
    assert(m != addrMap.end());
    m->second->access(pkt);
}

void
PhysicalMemory::functionalAccess(PacketPtr pkt)
{
    assert(pkt->isRequest());
    Addr addr = pkt->getAddr();
    AddrRangeMap<AbstractMemory*>::const_iterator m = addrMap.find(addr);
    assert(m != addrMap.end());
    m->second->functionalAccess(pkt);
}

void
PhysicalMemory::serialize(ostream& os)
{
    // serialize all the locked addresses and their context ids
    vector<Addr> lal_addr;
    vector<int> lal_cid;

    for (vector<AbstractMemory*>::iterator m = memories.begin();
         m != memories.end(); ++m) {
        const list<LockedAddr>& locked_addrs = (*m)->getLockedAddrList();
        for (list<LockedAddr>::const_iterator l = locked_addrs.begin();
             l != locked_addrs.end(); ++l) {
            lal_addr.push_back(l->addr);
            lal_cid.push_back(l->contextId);
        }
    }

    arrayParamOut(os, "lal_addr", lal_addr);
    arrayParamOut(os, "lal_cid", lal_cid);

    // serialize the backing stores
    unsigned int nbr_of_stores = backingStore.size();
    SERIALIZE_SCALAR(nbr_of_stores);

    unsigned int store_id = 0;
    // store each backing store memory segment in a file
    for (vector< BackingStore >::iterator s = backingStore.begin();
         s != backingStore.end(); ++s) {
        nameOut(os, csprintf("%s.store%d", name(), store_id++));
        s->serialize(os, store_id++);
    }
}

void
PhysicalMemory::unserialize(Checkpoint* cp, const string& section)
{
    // unserialize the locked addresses and map them to the
    // appropriate memory controller
    vector<Addr> lal_addr;
    vector<int> lal_cid;
    arrayParamIn(cp, section, "lal_addr", lal_addr);
    arrayParamIn(cp, section, "lal_cid", lal_cid);
    for(size_t i = 0; i < lal_addr.size(); ++i) {
        AddrRangeMap<AbstractMemory*>::const_iterator m =
            addrMap.find(lal_addr[i]);
        m->second->addLockedAddr(LockedAddr(lal_addr[i], lal_cid[i]));
    }

    // unserialize the backing stores
    unsigned int nbr_of_stores;
    UNSERIALIZE_SCALAR(nbr_of_stores);

    for (unsigned int i = 0; i < nbr_of_stores; ++i) {
        backingStore[i].unserialize(
            cp, csprintf("%s.store%d", section, i));
    }

}
