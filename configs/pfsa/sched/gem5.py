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
from abc import *
import sys
import os
import errno

import multiprocessing

import m5c.utils # For fixed drain and event handling
import m5

from runutils import MiniPOpen

_task_count = 0
class SimTask(object):
    __metaclass__ = ABCMeta

    def __init__(self, name="task%(task_id)i"):
        global _task_count
        self.process = MiniPOpen()
        self._sem_child = multiprocessing.Semaphore(0)
        self.name = name
        self.task_id = _task_count
        _task_count += 1

    def __str__(self):
        name = self.name % {
            "task_id" : self.task_id,
            "pid" : self.process.pid if self.process else 0,
            }

        if self.process and self.process.returncode == None:
            return "%s [running: %i]" % (name, self.process.pid)
        elif self.process and self.process.returncode != None:
            return "%s [exited: %i]" % (name, self.process.returncode)
        else:
            return "%s [pending]" % (name)

    def fork(self, sched):
        assert self.process.pid is None, \
            "Task must not have been started already."

        sem_parent = multiprocessing.Semaphore(0)
        root = m5.objects.Root.getInstance()
        pid = m5.fork(do_drain=False)
        if pid == 0:
            sched.release()
            self.process.pid = os.getpid()
            self.init()
            m5.resume(root)
            sem_parent.release()
            self.wait()
            self.main()
            sys.exit(0)
        else:
            self.process.pid = pid
            sem_parent.acquire()

    def wait(self):
        """Wait to be scheduled by the parent."""
        self._sem_child.acquire()

    def start(self):
        """Tell a child to start running"""
        self._sem_child.release()

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def main(self):
        pass
