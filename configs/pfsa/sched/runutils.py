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

import atexit
import os
import signal
import subprocess
import errno

def _eintr_retry_call(call, *args, **kwargs):
    while True:
        try:
            return call(*args, **kwargs)
        except OSError as ose:
            # Ignore errno 4 (Interrupted system call)
            if ose.errno != errno.EINTR:
                raise ose

class MiniPOpen(object):
    def __init__(self, pid=None):
        self.returncode = None
        self.pid = pid

    def _handle_exitstatus(self, status):
        if os.WIFSIGNALED(sts):
            self.returncode = -os.WTERMSIG(sts)
        elif os.WIFEXITED(sts):
            self.returncode = os.WEXITSTATUS(sts)
        else:
            # Should never happen
            raise RuntimeError("Unknown child exit status!")

    def poll(self):
        if self.returncode is None:
            pid, status = os.waitpid(self.pid, os.WNOHANG)
            if pid == self.pid:
                self._handle_exitstatus(status)
        return self.returncode

    def wait(self):
        if self.returncode is None:
            pid, status = _eintr_retry_call(os.waitpid, self.pid, 0)
            self._handle_exitstatus(status)

        return self.returncode

    def send_signal(self, sig):
        os.kill(self.pid, sig)

    def terminate(self):
        self.send_signal(signal.SIGTERM)

    def kill(self):
        self.send_signal(signal.SIGKILL)

class Task(object):
    def __init__(self, argv, executable=None, cwd=None,
                 stdin=None, stdout=None, stderr=None):
        self.process = None
        self.argv = argv
        self.executable = executable
        self.cwd = cwd if cwd else os.getcwd()
        self.stdin=stdin
        self.stdout=stdout
        self.stderr=stderr

    def start(self):
        assert self.process == None, "Process already running"

        self.process = subprocess.Popen(self.argv, executable=self.executable, cwd=self.cwd,
                                        stdin=self.stdin, stdout=self.stdout, stderr=self.stderr,
                                        close_fds=True)
    def _format_path(self, path):
        if path.startswith(self.cwd):
            return path[len(self.cwd)+1:]
        else:
            return path

    def __str__(self):
        if self.process and self.process.returncode == None:
            return "%s[running: %i]" % (self.argv[0], self.process.pid)
        elif self.process and self.process.returncode != None:
            return "%s[exited: %i]" % (self.argv[0], self.process.returncode)
        else:
            return "%s[pending]" % (self.argv[0])

    def verbose_str(self):
        ret = "%s\n" % str(self)
        if self.executable:
            ret += "\texecutable: %s\n" % self.executable
        ret += "\targv: %s\n" % self.argv
        ret += "\tcwd: %s\n" % self.cwd

        return ret

class SimpleTask(Task):
    def __init__(self, base, argv, executable=None, stdin=None):
        self.basedir = base
        self.log = open(os.path.join(base, "task.out"), "w")
        stdin = open("/dev/null", "r") if stdin == None else stdin

        Task.__init__(self, argv, executable, cwd=base,
                      stdin=stdin, stdout=self.log, stderr=self.log)

    def verbose_str(self):
        ret = Task.verbose_str(self)
        ret += "\tlog: %s\n" % self._format_path(self.log.name)
        ret += "\tstdin: %s\n" % self._format_path(self.stdin.name)

        return ret

class Scheduler(object):
    def __init__(self, active=False, concurrency=1):
        self.concurrency = concurrency
        self.active = active

        # List of tasks that are ready to be scheduled
        self.tasks_ready = []
        # Currently executing tasks
        self.tasks_active = {}
        # Tasks that have completed sucessfully
        self.tasks_done = []
        # Tasks that have failed
        self.tasks_error = []

        def exit_handler():
            self.active = False
            if self.tasks_active:
                print "Killing child processes"
                self.killall(signo=signal.SIGKILL)

        atexit.register(exit_handler)

    def add_task(self, task):
        self.tasks_ready.append(task)
        if self.active:
            self.schedule_new_tasks()

    def pop_ready(self):
        """Remove a task from the ready queue and return it."""

        task = self.tasks_ready[0]
        self.tasks_ready = self.tasks_ready[1:]
        return task

    def has_ready(self):
        return len(self.tasks_ready) > 0

    def has_active(self):
        return len(self.tasks_active) > 0

    def has_free_slots(self):
        return self.concurrency > len(self.tasks_active)

    def wait_for_tasks(self, blocking=True, interrupt_signo=None):
        """Suspend execution until a task needs to be waited
        for. Returns the task that needs service without affecting its
        wait state."""

        assert self.tasks_active, "wait_for_tasks expects active tasks"

        try:
            pid, status = _eintr_retry_call(os.waitpid, -1,
                                            0 if blocking else os.WNOHANG)
            if pid == 0:
                # Non-blocking wait didn't find ready process
                assert not blocking, "only non-block waitpid should return 0"
                return None
        except KeyboardInterrupt:
            self.tasks_error += self.tasks_ready
            self.tasks_ready = []
            if interrupt_signo == None:
                print "Got SIGINT, waiting for tasks to exit..."
                return self.wait_for_tasks(interrupt_signo=signal.SIGTERM)
            else:
                print "Got SIGINT, sending signal %i to active tasks..." % \
                    (interrupt_signo, )
                self.killall(interrupt_signo)
                return self.wait_for_tasks(interrupt_signo=signal.SIGKILL)

        try:
            task = self.tasks_active[pid]
        except KeyError:
            # This is can happen if a task is active, but halted when
            # it is scheduled. We'll just ignore such this situation.
            return None

        if os.WIFEXITED(status):
            task.process.returncode = os.WEXITSTATUS(status)
        elif os.WIFSIGNALED(status):
            task.process.returncode = -os.WTERMSIG(status)
        else:
            raise RuntimeError("Unexpected exit status: (%s, %s)" % \
                                   (pid, status))

        return task


    def schedule_new_tasks(self):
        """Schedule new tasks until there are no more ready tasks or
        the maximum concurrency is reached."""

        while self.has_free_slots() and self.has_ready() and self.active:
            task = self.pop_ready()
            print "%s: Starting..." % str(task)
            try:
                task.start()
                self.tasks_active[task.process.pid] = task
            except OSError as ose:
                print "Failed to start '%s': %s" % (str(task), str(ose))
                self.tasks_error.append(task)

    def reap_task(self, task):
        rc = task.process.returncode
        if rc == None:
            rc = task.process.poll()
            if rc == None:
                raise RuntimeError("Task doesn't require service.")

        if rc < 0:
            print "%s: Killed by signal %i" % (str(task), -rc)
            self.tasks_error.append(task)
        elif rc == 0:
            print "%s: Completed sucessfully." % str(task)
            self.tasks_done.append(task)
        else:
            print "%s: Task returned non-zero exit code (%i)." % (
                str(task), rc)
            self.tasks_error.append(task)

        del self.tasks_active[task.process.pid]

    def wait_and_schedule(self, blocking=True):
        task = self.wait_for_tasks(blocking=blocking)
        if task is not None:
            self.reap_task(task)
            self.schedule_new_tasks()
        return task

    def run(self):
        self.active = True
        while (self.has_active() or self.has_ready()) and self.active:
            self.wait_and_schedule(blocking=True)

    def on_sigchld(self, signum, frame):
        """SIGCHLD handler"""
        self.wait_and_schedule(blocking=False)

    def release(self):
        """Release control of all monitored subprocesses."""

        self.tasks_ready = []
        self.tasks_active = {}
        self.tasks_done = []
        self.tasks_error = []

    def killall(self, signo=signal.SIGTERM):
        for t in self.tasks_active.values() + self.tasks_ready:
            # We need to kill ready tasks as well since they might be
            # existing processes that are halted
            if t.process is None:
                continue

            print "%s: Sending signal '%s'" % (str(t), signo)
            try:
                t.process.send_signal(signo)
            except OSError as ose:
                # Ignore processes that don't exist (errno == 3, ESRCH)
                if ose.errno != 3:
                    raise ose

    def print_tasks(self):
        def print_list(lst):
            for t in lst:
                print str(t)

        print_list(self.tasks_ready)
        print_list(self.tasks_active.values())
        print_list(self.tasks_done)
        print_list(self.tasks_error)

    def print_status(self, verbose=False):
        def print_list(lst, name):
            print "-" * 70
            print name
            print "-" * 70
            for t in lst:
                if verbose:
                    print t.verbose_str()
                else:
                    print str(t)

        print_list(self.tasks_ready, "Ready Tasks")
        print
        print_list(self.tasks_active.values(), "Active Tasks")
        print
        print_list(self.tasks_done, "Completed Tasks")
        print
        print_list(self.tasks_error, "Failed Tasks")

def _selftest():
    devnull = open("/dev/null", "rw")

    def create_ttrue():
        return Task(["/bin/true" ],
                    stdin=devnull, stdout=devnull, stderr=devnull)

    def create_tfalse():
        return Task(["/bin/false" ],
                    stdin=devnull, stdout=devnull, stderr=devnull)

    def create_sleep(time):
        return Task(["/bin/sleep", str(time) ],
                    stdin=devnull, stdout=devnull, stderr=devnull)

    def create_illegal():
        return Task(["/dev/null"],
                    stdin=devnull, stdout=devnull, stderr=devnull)

    sched = Scheduler(concurrency=2)
    sched.add_task(create_ttrue())
    sched.add_task(create_sleep(1))
    sched.add_task(create_ttrue())
    sched.add_task(create_sleep(1))
    sched.add_task(create_tfalse())
    sched.add_task(create_illegal())

    sched.print_status()
    print "Starting scheduler..."
    sched.run()
    print "Done..."
    sched.print_status()

if __name__ == "__main__":
    _selftest()
