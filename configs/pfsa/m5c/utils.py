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
import kvmconfig
import sys
import os

def is_exit_fail(exit_event, code=None):
    return exit_event.getCause() == "m5_fail instruction encountered" and \
        (code == None or exit_event.getCode() == code)

def is_exit_ilimit(exit_event):
    return exit_event.getCause() == "instruction limit reached"

def is_m5_exit(exit_event):
    return exit_event.getCause() == "m5_exit instruction encountered"

def is_exit_fatal(event):
    cause = event.getCause()
    non_fatal = [ "switchCPU",
                  "sample",
                  "simulate() limit reached",
                  "instruction limit reached",
                  ]
    if cause in non_fatal:
        return False

    if is_exit_fail(event) and event.getCode() >= 128:
        return False

    return True

def is_exit_service(event):
    cause = event.getCause()
    if is_exit_fail(event) and event.getCode() >= 128:
        return True
    else:
        return False

def print_exit_event(event):
    cause = event.getCause()
    print 'Exiting @ tick %i because %s' % (m5.curTick(), cause)

    if is_exit_fail(event):
        if event.getCode() < 128:
            print "Simulation failed: %s" % kvmconfig.get_fail_str(event)
        else:
            print "Simulation event: %s" % kvmconfig.get_fail_str(event)

def get_priority_event(events):
    fatal_events = [ e for e in events if is_exit_fatal(e) ]
    if fatal_events:
        return fatal_events[0]
    else:
        return events[-1]

def m5drain(root):
    """Try to a system in preparation of a checkpoint, memory mode
    switch or CPU handover.

    Warning: Draining of the system can fail if the system is
    inconsistent and needs further simulation and that triggers an
    exit event. It is the responsability of the caller to ensure that
    adequate draining is performed by calling drain again if the event
    is non-fatal.

    Arguments:
      system -- Simulated system.

    Returns:
      A (drained, exit_event) tuple. drained is true if draining
      suceeded, false otherwise. exit_event is set if drained is
      False, it's None otherwise.
    """

    # Try to drain all objects. Draining might not be completed unless
    # all objects return that they are drained on the first call. This
    # is because as objects drain they may cause other objects to no
    # longer be drained.
    def _drain():
        dm = m5.internal.drain.createDrainManager()
        unready_objs = sum(obj.drain(dm) for obj in root.descendants())
        # If we've got some objects that can't drain immediately, then simulate
        if unready_objs > 0:
            dm.setCount(unready_objs)
            exit_event = m5.simulate()
            m5.internal.drain.cleanupDrainManager(dm)
            return False, exit_event
        else:
            m5.internal.drain.cleanupDrainManager(dm)
            return True, None

    while True:
        all_drained, event = _drain()
        if all_drained:
            assert event == None, "Unexpected exit event from _drain()"
            return (True, None)
        elif event != None and event.getCause() != "Finished drain":
            return (False, event)

def drain(root):
    pending_events = []

    print "Draining..."
    drained = False
    while not drained:
        print "Calling m5.drain(root)..."
        drained, exit_event = m5drain(root)
        if not drained:
            pending_events.append(exit_event)
            print "Drain terminated early due to exit event:"
            print_exit_event(exit_event)

            if is_exit_fatal(exit_event):
                print "Exit event was fatal, aborting drain."
                return False, pending_events

    print "Drain OK. Pending events: %s" % str(pending_events)
    return True, pending_events

def redirect_output(name):
    stdout_file = os.path.join(m5.options.outdir, name)
    redir_fd = os.open(stdout_file, os. O_WRONLY | os.O_CREAT | os.O_TRUNC)
    os.dup2(redir_fd, sys.stdout.fileno())
    os.dup2(redir_fd, sys.stderr.fileno())
