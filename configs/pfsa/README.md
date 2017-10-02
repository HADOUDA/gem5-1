
What is pFSA?
=============
pFSA is a technique to combine hardware virtualization and detailed
simulation to build an extremely efficient simulation
environment. More information is available in
[this tech report][tech_pfsa] and Andreas Sandberg's
[thesis][thesis_sandberg].


Patching gem5
=============
In order to run pFSA, you need to build a slightly modified gem5
version. Particularly, the upstream version of gem5 currently lacks
support for forking and treating cold misses as hits. The required
patches are in the patches/ directory and apply cleanly on the
stable_2014_02_15 release of gem5.

    git clone http://github.com/gem5/gem5.git
    git checkout -b pfsa stable_2014_02_15
    git am /path/to/patches/*.patch

The patched gem5 version is built according to the build instructions
on [gem5.org](http://gem5.org). Note that hardware virtualization is
only supported on ARM and x86. Also, please note that our pFSA
implementation has only been tested on x86.


Running pFSA
============

  /path/to/gem5/gem5.opt pfsa.py


[thesis_sandberg]: http://urn.kb.se/resolve?urn=urn:nbn:se:uu:diva-220652 
  "Sandberg, A. (2014). Understanding Multicore Performance: Efficient Memory System Modeling and Simulation."

[tech_pfsa]: http://urn.kb.se/resolve?urn=urn:nbn:se:uu:diva-220649
  "Sandberg, A., Hagersten, E. Black-Schaffer, D. (2014). Full Speed Ahead: Detailed Architectural Simulation at Near-Native Speed."

[iiswc_pfsa]: http://dx.doi.org/10.1109/IISWC.2015.29
  "Sandberg, A., et al. 'Full speed ahead: Detailed architectural simulation at near-native speed.' IISWC, 2015"
