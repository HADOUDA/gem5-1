# Copyright (c) 2006-2007 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Ali Saidi

from SysPaths import script, disk, binary
from os import environ as env
from m5.defines import buildEnv

class SysConfig:
    def __init__(self, script=None, root_disk=None, data_disk=None,
                 kernel=None,
                 mem_size='2GB'):
        self.scriptname = script
        self.rootdiskname = root_disk
        self.datadiskname = data_disk
        self.mem_size = mem_size
        self.kernelname = kernel

    def mem(self):
        return self.mem_size

    def script(self):
        if self.scriptname:
            return script(self.scriptname)
        else:
            return ''

    def kernel(self):
        return binary(self.kernelname) if self.kernelname is not None else ""

    def root_disk(self):
        if self.rootdiskname:
            return disk(self.rootdiskname)
        elif buildEnv['TARGET_ISA'] == 'x86':
            return env.get('LINUX_IMAGE', disk('linux-x86-kvm.img'))
        elif buildEnv['TARGET_ISA'] == 'arm':
            return env.get('LINUX_IMAGE', disk('linux-arm-ael.img'))
        else:
            print "Don't know what default disk image to use for %s ISA" % \
                buildEnv['TARGET_ISA']
            exit(1)

    def data_disk(self):
        if self.datadiskname:
            return disk(self.datadiskname)
        else:
            return None
