#!/bin/sed -f

# Copyright (c) 2013 Andreas Sandberg
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
# Authors: Andreas Sandberg

# READ BEFORE EDITING:
#
# * Avoid adding or removing newlines (e.g., deleting matched lines). It's
#   much easier to understand the Doxygen logs if they point to the right
#   line in the source files.
#
# * SED can be hard to read, so please document what your replacement rules
#   are supposed to do and why.
#

# Ignore bit unions for now. They confuse Doxygen.
/BitUnion[0-9]*(/,/EndBitUnion(.*)/ {
     # Replace matched lines with empty lines, this preserves correct
     # line numbers in the Doxygen output.
     s/.*//g
}

# Match something returning a pointer (i.e., line starting with Foo *)
/^[^ ]\+ \?\*$/ {
     # Read the next line and append it to the pattern space so we can
     # match the whole function signature.
     N;

     # Ignore *Params::create(). Doxygen gets confused because it doesn't
     # see the declaration.
     /.*\n[^ ]\+Params::create()$/ {
          s/[^\n]//g
     }
}
