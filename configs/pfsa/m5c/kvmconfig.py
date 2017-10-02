#!/usr/bin/env python
#
# Copyright (c) 2013 Andreas Sandberg <andreas@sandberg.pp.se>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import m5
import math
import os
import shutil
import FSConfig
from abc import *

fail_codes = {
    0 : "Failed to mount benchmark disk",
    1 : "Failed to find benchmark directory",
    2 : "Failed to enter benchmark run configuration",
    3 : "Failed to execute benchmark",
    4 : "Verification failed",

    128 : "Non-fatal base (not a failure)",
    253 : "Compare starting (not a failure)",
    254 : "Benchmark starting (not a failure)",
    255 : "OS boot completed (not a failure)",
    }

class BenchmarkBase(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        pass

    def get_insts(self):
        raise NotImplementedError("Instruction not known for this configuration.")

    def valid(self):
        return True

    @abstractmethod
    def script(self, **kwargs):
        pass

class Spec2006(BenchmarkBase):
    valid_datasets = [ "test", "train", "ref" ]
    valid_benchmarks = [
        "400.perlbench", "401.bzip2", "403.gcc", "410.bwaves", "416.gamess",
        "429.mcf", "433.milc", "434.zeusmp", "435.gromacs", "436.cactusADM",
        "437.leslie3d", "444.namd", "445.gobmk", "447.dealII", "450.soplex",
        "453.povray", "454.calculix", "456.hmmer", "458.sjeng", "459.GemsFDTD",
        "462.libquantum", "464.h264ref", "465.tonto", "470.lbm", "471.omnetpp",
        "473.astar", "481.wrf", "482.sphinx3", "483.xalancbmk", 
        "998.specrand", "999.specrand",
        ]
    insts = {
        "ref" : {
            "400.perlbench" : 2.09678105135e+12,
            "401.bzip2" : 2.28777883197e+12,
            "403.gcc" : 1.07612749444e+12,
            "410.bwaves" : 3.36100584167e+12,
            "416.gamess" : 5.54134629097e+12,
            "429.mcf" : 3.18820596253e+11,
            "433.milc" : 1.21538179124e+12,
            "434.zeusmp" : 1.79438840336e+12,
            "435.gromacs" : 1.98017870059e+12,
            "436.cactusADM" : 2.49813165243e+12,
            "437.leslie3d" : 1.54471003348e+12,
            "444.namd" : 2.25624368408e+12,
            "445.gobmk" : 1.69819278026e+12,
            "447.dealII" : 1.90954115846e+12,
            "450.soplex" : 6.98555646145e+11,
            "453.povray" : 9.53092159023e+11,
            "454.calculix" : 6.20001551825e+12,
            "456.hmmer" : 2.77434219769e+12,
            "458.sjeng" : 2.1950912917e+12,
            "459.GemsFDTD" : 1.55327145099e+12,
            "462.libquantum" : 1.86027237232e+12,
            "464.h264ref" : 3.19278458368e+12,
            "465.tonto" : 2.2663445214e+12,
            "470.lbm" : 1.24595452674e+12,
            "471.omnetpp" : 5.8103157121e+11,
            "473.astar" : 1.17036533324e+12,
            "481.wrf" : 3.11459841453e+12,
            "482.sphinx3" : 3.30805399418e+12,
            "483.xalancbmk" : 1.066460692e+12,
            }
        }

    def __init__(self, bench, dataset):
        super(Spec2006, self).__init__()
        self.bench = bench
        self.dataset = dataset
        self.disk = "linux-x86-uart-bench.img"

    def get_insts(self):
        try:
            return self.insts[self.dataset][self.bench]
        except KeyError:
            super(Spec2006, self).get_insts()

    def valid(self):
        return self.bench in self.valid_benchmarks and \
            self.dataset in self.valid_datasets

    def script(self, verify=False, no_varand=False, enable_coredump=False,
               no_redirect=False):
        script_name = os.path.abspath(os.path.join(m5.options.outdir, "script"))

        copy_core = ( "    for F in core*; do",
                      "        [ -e \"$F\" ] && m5 writefile $F",
                      "    done" ) if enable_coredump else ()

        verify = ( "m5 fail 253",
                   "specinvoke -E -r compare.cmd || m5 fail 4" ) \
                   if verify else ()

        specinvoke_flags = ["-E"]
        if no_redirect:
            specinvoke_flags.append("-r")

        sys_config = []
        if no_varand:
            sys_config.append("sysctl -w kernel.randomize_va_space=0")
        if enable_coredump:
            sys_config.append("ulimit -c unlimited\n")

        fout = open(script_name, "w")
        fout.write("""#!/bin/sh

mkdir /data
mount /dev/sdb1 /data || m5 fail 0

%(sys_config)s

cd /data/spec2006.inst
. ./shrc
cd /data/spec2006.gem5/benchspec/CPU2006/%(bench)s/run || m5 fail 1
cd run_base_%(dataset)s_amd64-m64-gcc47.0000 || m5 fail 2

m5 fail 254
specinvoke %(specinvoke_flags)s speccmds.cmd
ERR=$?
if [ $ERR -ne 0 ]; then
    echo "Benchmark exited with error: $ERR"

%(copy_core)s
    m5 fail 3
fi

%(verify)s

m5 exit
""" % {
                "sys_config" : "\n".join(sys_config),
                "specinvoke_flags" : " ".join(specinvoke_flags),
                "bench" : self.bench,
                "dataset" : self.dataset,
                "copy_core" : "\n".join(copy_core),
                "verify" : "\n".join(verify),
                })

        fout.close()

        return script_name

class SpecJbb2005(BenchmarkBase):
    def __init__(self):
        super(SpecJbb2005, self).__init__()
        self.disk = "linux-x86-uart-jbb.img"

    def valid(self):
        return True

    def get_insts(self):
        return 3878269669580

    def script(self, verify=False, no_varand=False, enable_coredump=False,
               no_redirect=False):
        script_name = os.path.abspath(os.path.join(m5.options.outdir, "script"))

        copy_core = ( "    for F in core*; do",
                      "        [ -e \"$F\" ] && m5 writefile $F",
                      "    done" ) if enable_coredump else ()

        sys_config = []
        if no_varand:
            sys_config.append("sysctl -w kernel.randomize_va_space=0")
        if enable_coredump:
            sys_config.append("ulimit -c unlimited\n")

        fout = open(script_name, "w")
        fout.write("""#!/bin/sh

mkdir /data
mount /dev/sdb1 /data || m5 fail 0

%(sys_config)s

export PATH=/data/java/bin:$PATH
cd /data/SPECjbb2005

export CLASSPATH=./jbb.jar:./check.jar:$CLASSPATH

m5 fail 254
java -Xms512m -Xmx512m spec.jbb.JBBmain -propfile SPECjbb.props
ERR=$?
if [ $ERR -ne 0 ]; then
    echo "Benchmark exited with error: $ERR"

%(copy_core)s
    m5 fail 3
fi

m5 exit
""" % {
                "sys_config" : "\n".join(sys_config),
                "copy_core" : "\n".join(copy_core),
                })
        fout.close()

        return script_name

class BenchBinary(BenchmarkBase):
    def __init__(self, binary):
        super(BenchBinary, self).__init__()
        self.binary = binary
        self.disk = None

    def valid(self):
        return os.path.exists(self.binary)

    def script(self, **kwargs):
        base = os.path.basename(self.binary)
        local_name = os.path.abspath(os.path.join(m5.options.outdir, base))
        shutil.copy2(self.binary, local_name)

        return local_name

def get_fail_str(event):
    return fail_codes.get(event.getCode(), "Unknown error")

class IOCache(m5.objects.BaseCache):
    assoc = 8
    hit_latency = 50
    response_latency = 50
    mshrs = 20
    size = '1kB'
    tgts_per_mshr = 12
    forward_snoops = False
    is_top_level = True

class L1Cache(m5.objects.BaseCache):
    assoc = 2
    hit_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20
    is_top_level = True

class L1ICache(L1Cache):
    size = '64kB'

class L1DCache(L1Cache):
    size = '64kB'
    mshrs = 8

class L2Cache(m5.objects.BaseCache):
    size= '2MB'
    assoc = 8
    hit_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    write_buffers = 8

class L2CachePF(L2Cache):
    prefetch_on_access = 'true'
    prefetcher = m5.objects.StridePrefetcher(degree=16, latency=1)

def create_memory(system, channels=1):
    dram_class = m5.objects.SimpleDDR3

    if channels == 1:
        system.dram = dram_class(range=m5.objects.AddrRange("2GB"))
        system.membus.master = system.dram.port
    else:
        system.dram = dram_class.makeMultiChannel(
            channels, 0, "2GB",
            intlv_high_bit=(11 + math.log(channels, 2)))

        for dram in system.dram:
            system.membus.master = dram.port

class Membus(m5.objects.CoherentBus):
    width = 64

class O3CPU(m5.objects.DerivO3CPU):
    wbDepth = 8
    LQEntries = 64
    SQEntries = 64

class AtomicCPU(m5.objects.AtomicSimpleCPU):
    pass


def create_system(mdesc, boot_cpu_class,
                  cpu_freq="2GHz", sys_freq="1GHz",
                  l2_size="2MB",
                  no_kvm=False, script=None):
    test_sys = FSConfig.makeLinuxX86System(boot_cpu_class.memory_mode(), 1,
                                           mdesc=mdesc)

    test_sys.kernel = mdesc.kernel()
    if not no_kvm:
        test_sys.vm = m5.objects.KvmVM()

    # Setup clock and voltage domains
    test_sys.voltage_domain = m5.objects.VoltageDomain()
    test_sys.clk_domain = m5.objects.SrcClockDomain(
        clock="1GHz",
        voltage_domain=test_sys.voltage_domain)

    test_sys.cpu_voltage_domain = m5.objects.VoltageDomain()
    test_sys.cpu_clk_domain = m5.objects.SrcClockDomain(
        clock=cpu_freq,
        voltage_domain=test_sys.cpu_voltage_domain)

    boot_cpu = boot_cpu_class(cpu_id=0, clk_domain=test_sys.cpu_clk_domain)
    test_sys.cpu_boot = boot_cpu

    test_sys.cpu_atomic = AtomicCPU(cpu_id=0,
                                    clk_domain=test_sys.cpu_clk_domain,
                                    switched_out=True)

    test_sys.cpu_o3 = O3CPU(cpu_id=0,
                            clk_domain=test_sys.cpu_clk_domain,
                            switched_out=True)

    # Setup branch predictor warming
    test_sys.branchPred = m5.objects.BranchPredictor(numThreads=1)
    test_sys.cpu_o3.branchPred = test_sys.branchPred
    test_sys.cpu_atomic.branchPred = test_sys.branchPred

    boot_cpu.createThreads()

    # Setup the memory system
    test_sys.iocache = IOCache(addr_ranges=test_sys.mem_ranges)
    test_sys.iocache.cpu_side = test_sys.iobus.master
    test_sys.iocache.mem_side = test_sys.membus.slave

    boot_cpu.addTwoLevelCacheHierarchy(L1ICache(), L1DCache(),
                                       L2CachePF(size=l2_size))

    boot_cpu.createInterruptController()
    boot_cpu.connectAllPorts(test_sys.membus)

    return test_sys
