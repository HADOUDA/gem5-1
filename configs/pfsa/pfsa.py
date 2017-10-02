#
# Copyright (c) 2013-2014 Andreas Sandberg <andreas@sandberg.pp.se>
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
import argparse
import os
import sys
import math
import random
import re
from functools import partial

import m5

from m5c.Benchmarks import SysConfig
from m5c.utils import *
import m5c.SysPaths

from utils.osutils import CpuInfo
from utils import osutils

from sched.runutils import Scheduler
from sched.gem5 import SimTask


sched = Scheduler(active=True, concurrency=1)

tasks = [
    "smarts",
    ]


run_types = (
    "reference",
    "sample",
    "interactive",
    "nativeref",
    )

try:
    kvm_cpu_class = m5.objects.X86KvmCPU
except AttributeError:
    kvm_cpu_class = None

def parse_options():
    from optparse import OptionParser

    global sched

    re_int_suffix = re.compile("^([+-]?[0-9]+)(.)?$")
    dec_suffixes = {
        None : 1,
        "k" : 10**3,
        "M" : 10**6,
        "G" : 10**9,
        "T" : 10**12,
        }
    bin_suffixes = {
        None : 1,
        "k" : 2**10,
        "M" : 2**20,
        "G" : 2**30,
        "T" : 2**40,
        }
    def int_suffix(string, type=int, suffixes=dec_suffixes):
        m = re_int_suffix.match(string)
        if m is None:
            raise argparse.ArgumentTypeError(
                "%r is not a valid integer with suffix" % string)

        value, suffix = m.groups()
        if suffix not in suffixes:
            raise argparse.ArgumentTypeError(
                "%r does not have a valid suffix" % string)

        try:
            return type(value) * suffixes[suffix]
        except ValueError:
            raise "%s is not a valid %s." % (value, type.__name__)

    int_bin_suffix = partial(int_suffix, suffixes=bin_suffixes)

    long_suffix = partial(int_suffix, type=long)
    long_bin_suffix = partial(int_suffix, type=long, suffixes=bin_suffixes)

    def int_tuple(string):
        try:
            int_lst = [ int(x) for x in string.split(",") ]
        except ValueError:
            raise argparse.ArgumentTypeError(
                "%r is not a list of integers" % string)
        return tuple(int_lst)

    cpuinfo = CpuInfo()

    parser = argparse.ArgumentParser(description='Run a forking simulation')

    option = parser.add_argument

    parser.add_argument('bench', type=str, nargs='*',
                        help='Benchmark specification')

    option("--verify", action="store_true", default=False,
           help="Enable result verification")

    option("--cpu-freq", type=float, default=2.5,
           help="Simulated CPU clock frequency [GHz]")

    option("--l2-size", type=str, default="2MB",
           help="L2 cache size.")

    option("--host-freq", type=float, default=(cpuinfo.freq() / 1000),
           help="Host clock frequency [GHz]")

    option("--cpt-restore-boot", type=str, default=None,
           help="Boot from checkpoint")

    option("--cpt-create-boot", type=str, default=None,
           help="Create boot checkpoint")

    option("--atomic-warm", type=long_suffix, default=2000000,
           help="Number of instructions to execute in atomic mode")
    option("--o3-warm", type=long_suffix, default=30000,
           help="Number of O3 instructions to simulate when warming")
    option("--o3-sim", type=long_suffix, default=20000,
           help="Number of O3 instructions to simulate as reference")

    option("--smarts-period", type=long_suffix, default=430000000,
           help="Number of instructions to simulate in KVM mode")

    option("--smarts-samples", type=long_suffix, default=None,
           help="Request N samples")

    option("--kvm-exp", action="store_true", default=False,
           help="Use exponentially distributed KVM instruction counts")

    option("--max-insts", type=long_suffix, default=None,
           help="Maximum number of instructions to simulate")

    option("--no-kvm", action="store_true", default=False,
           help="Don't use KVM")

    option("--no-fork", action="store_true", default=False,
           help="Don't fork")

    option("--task", type=str, choices=tasks, default="smarts",
           help="Task to run in child")

    option("--run-type", type=str, choices=run_types, default="sample",
           help="What kind of run is this")

    option("--no-flush", action="store_true", default=False,
           help="Refresh instead of flushing caches")

    option("--kvm-fadj", type=float, default=0.95,
           help="KVM factor adjusted for frequency differences")

    option("--halt-failed-task", action="store_true", default=False,
           help="Halt immediately if a child fails.")

    option("--hit-on-cold", action="store_true", default=False,
           help="Enable hit on cold warming delta")

    option("--redirect-o3-tlb", action="store_true", default=False,
           help="Redirect the O3 CPU's TLB")

    option("--sched-concurrency", type=int, default=-2,
           help="Number of concurrent processes. Negative values are relative the number of CPUs.")

    option("--sched-nodes", type=int_tuple, default=());
    option("--sched-cores", type=int_tuple, default=());
    option("--sched-threads", type=int_tuple, default=());

    args = parser.parse_args()

    if kvm_cpu_class is None and not args.no_kvm:
        print >> sys.stderr, "warn: KVM not supported, forcing --no-kvm"
        args.no_kvm = True

    if args.sched_concurrency > 0:
        sched.concurrency = args.sched_concurrency
    elif args.sched_concurrency <= 0:
        sched.concurrency = cpuinfo.no_cpus() + args.sched_concurrency
        if sched.concurrency < 1:
            parser.error("Invalid concurrency specification.")
    else:
        parser.error("Invalid concurrency specification.")

    if args.smarts_period < args.atomic_warm + args.o3_warm + args.o3_sim:
        parser.error("SMARTS period too small")

    bench_spec = args.bench
    if len(bench_spec) == 2:
        bench = kvmconfig.Spec2006(bench_spec[0], bench_spec[1])
        if not bench.valid():
            parser.error("Invalid benchmark specified.")
    elif len(bench_spec) == 1:
        barray = bench_spec[0].split(":", 1)
        btype, name = barray[0], barray[1] if len(barray) == 2 else ""
        if btype == "bin":
            if not os.path.exists(name):
                parser.error("Invalid binary specified.")
            bench = kvmconfig.BenchBinary(name)
        elif btype == "jbb2005":
            bench = kvmconfig.SpecJbb2005()
        else:
            parser.error("Invalid benchmark type specified")
    elif len(bench_spec) == 0:
        bench = None
    else:
        parser.error("Incorrect number of arguments.")

    if args.smarts_samples is not None:
        try:
            if args.max_insts is not None:
                insts = args.max_insts
            else:
                insts = bench.get_insts()
        except NotImplementedError, e:
            print >> sys.stderr, "Can't determine number of instructions " \
                "in benchmark: %s" % e
            exit(1)

        args.smarts_period = long(insts / args.smarts_samples)
        print "Using automatic SMARTS-period: %i" % args.smarts_period

    return args, bench

args, bench = parse_options()

print "Setup:"
print "Args: %s" % (args, )
print "Bench: %s" % bench

osutils.set_affinity(nodes=args.sched_nodes,
                     cores=args.sched_cores,
                     threads=args.sched_threads)

# Disable all listeners since they will mess up forks
m5.disableAllListeners()

cpu_freq = args.cpu_freq * 1E9
script_name = bench.script(verify=args.verify,
                           no_varand=True,
                           enable_coredump=True,
                           ) if bench else None

if args.no_kvm:
    fast_cpu_class = kvmconfig.AtomicCPU
else:
    fast_cpu_class = kvm_cpu_class

mdesc = SysConfig(script=script_name,
                  kernel="x86_64-vmlinux-3.2.44",
                  root_disk="linux-debian-wheezy.img",
                  data_disk=bench.disk if bench is not None else None)
test_sys = kvmconfig.create_system(mdesc, fast_cpu_class,
                                   cpu_freq="%iHz" % cpu_freq,
                                   no_kvm=args.no_kvm,
                                   l2_size=args.l2_size)

if args.redirect_o3_tlb:
    test_sys.cpu_o3.itb = test_sys.cpu_atomic.itb
    test_sys.cpu_o3.dtb = test_sys.cpu_atomic.dtb

if kvm_cpu_class is not None and issubclass(fast_cpu_class, kvm_cpu_class):
    test_sys.cpu_boot.hostFreq = "%fGHz" % args.host_freq
    test_sys.cpu_boot.hostFactor = args.kvm_fadj * float(args.host_freq) / args.cpu_freq

root = m5.objects.Root(full_system=True, system=test_sys)

all_caches = ( test_sys.cpu_boot.dcache,
               test_sys.cpu_boot.icache,
               test_sys.cpu_boot.l2cache,
               )

def setHitOnCold(enabled):
    for c in all_caches:
        c.setHackHitOnCold(enabled)

def handle_sim_exit(event):
    """Handle a simulation exit event and return False if it was fatal."""

    print_exit_event(event)

    return not (is_exit_fatal(event) or is_exit_service(event))


cur_cpu = test_sys.cpu_boot
def switch_cpus(next_cpu, do_drain=True):
    global cur_cpu

    exit_events = None
    print "Switching CPUs: %s -> %s" % (cur_cpu.__class__.__name__,
                                        next_cpu.__class__.__name__)

    if do_drain:
        drained, exit_events = drain(root)
        if not drained:
            return False, exit_events

    if args.no_flush:
        if next_cpu.memory_mode() == "atomic_noncaching" and \
                next_cpu.memory_mode() != cur_cpu.memory_mode():
            print "Writing dirty buffers to memory..."
            m5.memWriteback(test_sys)
            # m5.switchCpus won't try to switch memory modes if we
            # switch to the right mode first.
            test_sys.setMemoryMode(m5.objects.params.atomic_noncaching)

        if cur_cpu.memory_mode() == "atomic_noncaching" and \
                next_cpu.memory_mode() != "atomic_noncaching":
            print "Refreshing memory components..."
            m5.memRefresh(test_sys)


    if cur_cpu != next_cpu:
        m5.switchCpus(test_sys, [ ( cur_cpu, next_cpu ) ],
                      do_drain=False)

    cur_cpu = next_cpu

    if do_drain:
        print "Resuming..."
        m5.resume(root)

    return True, exit_events
    
def handle_fatal_exit(event, fail_code):
    sys.exit(0 if is_m5_exit(event) else fail_code)

class BaseChildTask(SimTask):
    def __init__(self, init_cpu):
        super(BaseChildTask, self).__init__()
        self.init_cpu = init_cpu

    def init(self):
        if not args.no_fork:
            redirect_output("simout")
        switch_ok, exit_events = switch_cpus(self.init_cpu, do_drain=False)
        if exit_events:
            print "Unexpected exit event when switching CPU in child"
            exit(1)
        m5.stats.reset()

class SmartsTask(BaseChildTask):
    def __init__(self, **kwargs):
        super(SmartsTask, self).__init__(
            test_sys.cpu_atomic if args.atomic_warm else test_sys.cpu_o3,
            **kwargs)

    def main(self):
        warm_atomic = args.atomic_warm
        warm_o3 = args.o3_warm
        sim_o3 =  args.o3_sim

        print
        print "*" * 70
        print "Starting simulation cycle"
        print "*" * 70

        if warm_atomic:
            print "Warming caches..."
            test_sys.cpu_atomic.scheduleInstStop(0, warm_atomic, "switchCPU")
            exit_event = m5.simulate()
            if not handle_sim_exit(exit_event):
                handle_fatal_exit(exit_event, 21)

        m5.stats.dump()
        m5.stats.reset()

        hit_cold_pid = None
        if args.hit_on_cold:
            print "Forking hit on cold child..."
            hit_cold_pid = os.fork()
            if hit_cold_pid == 0:
                setHitOnCold(True)
            else:
                print "Waiting for hit on cold child to complete."
                _pid, _status = osutils.eintr_retry_call(os.waitpid,
                                                         hit_cold_pid, 0)
                status = osutils.handle_exit_status(_status)
                if status != 0:
                    print >> sys.stderr, \
                        "Unexpected hit on cold child status: %i" % status
                    exit(31)

                print "Continuing normal execution..."

        print "Warming O3 structures for %i instructions." % warm_o3
        if warm_atomic:
            switch_ok, exit_events = switch_cpus(test_sys.cpu_o3)
            if not switch_ok:
                handle_fatal_exit(get_priority_event(exit_events), 22)

        test_sys.cpu_o3.scheduleInstStop(0, warm_o3, "switchCPU")
        exit_event = m5.simulate()
        if not handle_sim_exit(exit_event):
            handle_fatal_exit(exit_event, 23)


        m5.stats.dump()
        m5.stats.reset()

        print "Running real O3 simulation for %i instructions." % sim_o3
        test_sys.cpu_o3.scheduleInstStop(0, sim_o3, "switchCPU")
        exit_event = m5.simulate()
        m5.stats.dump()
        if not handle_sim_exit(exit_event):
            handle_fatal_exit(exit_event, 24)

        # Exit if this is a hit on cold child
        if hit_cold_pid == 0:
            print "Hit cold child completed."
            exit(0)


task_classes = {
    "smarts" : SmartsTask,
}

def run_sample():
    if bench:
        print "Preparing benchmark..."
        exit_event = m5.simulate()
        print_exit_event(exit_event)
        if not is_exit_fail(exit_event, 254):
            print "Huh? Didn't get expected m5_fail instruction..."
            exit(1)

        print "Benchmark ready. Starting real simulation..."
    else:
        # Workaround for stupid gem5 semantics drain/resume
        # semantics. When restoring from a checkpoint, gem5 internally
        # maintains a list of objects that need to be resumed. This
        # conflicts with our CPU switching cycle which wants to switch
        # immediately after instantiation. What you lack in design,
        # you make up for in duct tape and piano wire...
        m5.simulate(1)

    m5.stats.dump()
    m5.stats.reset()

    task_class = task_classes[args.task]
    print "Task class: %s" % (task_class.__name__  if task_class is not None \
                                  else "drain-test")

    kvm_sim = args.smarts_period if not args.no_fork else \
        (args.smarts_period - args.atomic_warm - args.o3_warm - args.o3_sim)
    if args.max_insts is not None:
        test_sys.cpu_boot.scheduleInstStop(0, args.max_insts,
                                           "instruction limit reached")
    while True:
        drained, exit_events = m5c.utils.drain(root)
        if not drained:
            [ m5c.utils.print_exit_event(e) for e in exit_events ]
            print "Drain failed."
            handle_fatal_exit(get_priority_event(exit_events), 1)

        task = task_class() if task_class is not None else None

        if task is None:
            # Do nothing since we don't actually want to start a task
            pass
        elif not args.no_fork:
            task.fork(sched)
            print "master: Scheduling task: %s" % task
            sched.add_task(task)

            # Complete some tasks if we have reached the concurrency
            # limit.
            while not sched.has_free_slots():
                sched.wait_and_schedule()
            if args.halt_failed_task and sched.tasks_error:
                print >> sys.stderr, "Scheduler reported failed tasks."
                sched.print_status()
                exit(1)
        else:
            task.init()

        m5.resume(root)

        if task is not None and args.no_fork:
            print "parent: Running task main in parent context"
            task.main()
            print "parent: Task completed, switching back to main CPU"
            switch_ok, exit_events = switch_cpus(test_sys.cpu_boot)
            if exit_events:
                print "Unexpected exit event when switching CPU in child"
                exit(1)
            m5.stats.reset()

        ff_kvm = long(random.expovariate(1.0 / kvm_sim)) if args.kvm_exp else kvm_sim
        test_sys.cpu_boot.scheduleInstStop(0, ff_kvm, "sample")
        exit_event = m5.simulate()
        print_exit_event(exit_event)
        m5.stats.dump()
        m5.stats.reset()

        if exit_event.getCause() != "sample":
            break

    if args.verify:
        if is_exit_ilimit(exit_event):
            print "Verification requested, running to completion..."
            while True:
                exit_event = m5.simulate()
                if exit_event.getCause() != "sample":
                    break

            print_exit_event(exit_event)

        if not is_exit_fail(exit_event, 253):
            print "Warning: Unexpected exit event while waiting verification start."
            print "Trying to continue anyway..."
    else:
        if not is_exit_ilimit(exit_event) and \
                not is_m5_exit(exit_event):
            print "Unexpected exit event."
            exit(1)

def run_verify():
    print "Benchmark completed, starting verification run..."
    while True:
        exit_event = m5.simulate()
        print_exit_event(exit_event)

        if is_m5_exit(exit_event):
            print "Verification sucessful"
            return
        elif is_exit_fail(exit_event, 4):
            print "Verification failed."
            exit(16)
        elif exit_event.getCause() == "sample":
            # Ignore sample events
            pass
        else:
            print "Unexpected exit event when verifying results"
            exit(1)

def run_reference():
    sample_period = 10000000

    print "Preparing benchmark..."
    exit_event = m5.simulate()
    print_exit_event(exit_event)
    if not is_exit_fail(exit_event, 254):
        print "Huh? Didn't get expected m5_fail instruction..."
        exit(1)

    switch_ok, exit_events = switch_cpus(test_sys.cpu_o3)
    if not switch_ok:
        handle_fatal_exit(get_priority_event(exit_events), 21)

    print "Benchmark ready. Starting real simulation..."
    if args.max_insts is not None:
        test_sys.cpu_o3.scheduleInstStop(0, args.max_insts,
                                         "instruction limit reached")
    while True:
        m5.stats.reset()
        test_sys.cpu_o3.scheduleInstStop(0, sample_period, "switchCPU")
        exit_event = m5.simulate()

        m5.stats.dump()

        print_exit_event(exit_event)
        if is_m5_exit(exit_event) or \
                is_exit_fatal(exit_event) or \
                is_exit_fail(exit_event, 253) or \
                is_exit_ilimit(exit_event):
            break

    m5.stats.reset()
    if args.verify:
        if not is_exit_fail(exit_event, 253):
            print "Error: Unexpected exit event while waiting verification start."
            print "Trying to continue anyway..."
            exit(1)
    else:
        if not is_m5_exit(exit_event) and \
                not is_exit_ilimit(exit_event):
            print "Unexpected exit event."
            exit(1)


def run_interactive():
    m5.stats.dump()
    m5.stats.reset()
    while True:
        exit_event = m5.simulate()
        print_exit_event(exit_event)
        if is_m5_exit(exit_event) or \
                is_exit_fatal(exit_event) or \
                is_exit_fail(exit_event, 253):
            break
    m5.stats.dump()
    m5.stats.reset()
    if args.verify:
        if not is_exit_fail(exit_event, 253):
            print "Error: Unexpected exit event while waiting verification start."
            print "Trying to continue anyway..."
            exit(1)
    else:
        if not is_m5_exit(exit_event):
            print "Unexpected exit event."
            exit(1)

def run_nativeref():
    print "Preparing benchmark..."
    exit_event = m5.simulate()
    print_exit_event(exit_event)
    if not is_exit_fail(exit_event, 254):
        print "Huh? Didn't get expected m5_fail instruction..."
        exit(1)

    if args.max_insts is not None:
        test_sys.cpu_boot.scheduleInstStop(0, args.max_insts,
                                           "instruction limit reached")

    return run_interactive()

run_funs = {
    "reference" : run_reference,
    "interactive" : run_interactive,
    "sample" : run_sample,
    "nativeref" : run_nativeref,
    }

if args.cpt_restore_boot is not None:
    print "Restoring checkpoint of booted system..."
    m5.instantiate(args.cpt_restore_boot)
else:
    print "Instantiating a new system..."
    m5.instantiate()

    print "Fast forwarding through the boot process..."
    exit_event = m5.simulate()
    print_exit_event(exit_event)
    if not is_exit_fail(exit_event, 255):
        print "Huh? Didn't get expected m5_fail instruction..."
        exit(1)

    print "Boot completed. Starting real simulation..."

if args.cpt_create_boot is not None:
    print "Creating checkpoint of booted system..."
    m5.checkpoint(args.cpt_create_boot)
    sys.exit(0)

run_funs[args.run_type]()

if args.verify:
    run_verify()
    m5.stats.dump()

print "Waiting for children to terminate..."
sched.run()
sched.print_status()
