# Adolphus - Multi-Camera System Coverage Modeling Suite


## Overview

Adolphus is a set of tools for modeling and visualizing the coverage of
multi-camera systems.

See our [screencast] [screencast] for an introduction.


## Dependencies

Adolphus requires [Python] [python] 2.6 or later, [PyYAML] [pyyaml] 3.09 or later, [Cython] [cython] 0.14 or later, [pycollada][collada] 0.4 or later, and [setuptools] [setuptools].

[Visual] [visual] 5.4 or later is required for 3D visualization and interaction
(optional, recommended). [PyGTK] [pygtk] 2.22 or later is required for the
interactive tool panel (optional).

[Epydoc] [epydoc] is required for generating API documentation (optional).


## Development

### Cython Modules

After modifying any `.pyx` file, regenerate the C source and build it in-place
for testing by issuing the following commands in the repository root:

    cython adolphus/*.pyx
    python setup.py build_ext --inplace


## Related Publications

* A. Mavrinac and X. Chen, "[Modeling Coverage in Camera Networks: A Survey] [mav_surv],"
*Int. J. Computer Vision*, vol. 101, no. 1, pp. 205-226, 2013.
* J. L. Alarcon-Herrera and X. Chen, ”[Online Configuration of PTZ Camera Networks] [icdsc12],” 
in *Proc. 6th ACM/IEEE Intl. Conf. on Distributed Smart Cameras*, 2012.
* A. Mavrinac, D. Rajan, Y. Tan, and X. Chen, "[Task-Oriented Optimal View Selection 
in a Calibrated Multi-Camera System] [mav_aim12]," Proc. IEEE/ASME Int. Conf. Advanced Intelligent 
Mechatronics, 2012.
* A. Mavrinac and X. Chen, "[Optimizing Load Distribution in Camera Networks
with a Hypergraph Model of Coverage Topology] [m11_load]," in *Proc 5th ACM/IEEE
Int. Conf. Distributed Smart Cameras*, 2011.
* J. L. Alarcon-Herrera, A. Mavrinac, and X. Chen, "[Sensor Planning for Range
Cameras via a Coverage Strength Model] [a11_sprg]," in *Proc. IEEE/ASME Int.
Conf. Advanced Intelligent Mechatronics*, 2011, pp. 838-843.
* A. Mavrinac, J. L. Alarcon-Herrera, and X. Chen, "[Evaluating the Fuzzy
Coverage Model for 3D Multi-Camera Network Applications] [m10_eval]," in *Proc.
Intl. Conf. on Intelligent Robotics and Applications*, 2010, pp. 692-701.
* A. Mavrinac, J. L. Alarcon-Herrera, and X. Chen, "[A Fuzzy Model for Coverage
Evaluation of Cameras and Multi-Camera Networks] [m10_fcov]," in *Proc. 4th
ACM/IEEE Intl. Conf. on Distributed Smart Cameras*, 2010, pp. 95-102.


[python]: http://www.python.org
[cython]: http://cython.org
[pyyaml]: http://pyyaml.org
[visual]: http://vpython.org
[epydoc]: http://epydoc.sourceforge.net
[pygtk]: http://www.pygtk.org/
[setuptools]: http://pypi.python.org/pypi/setuptools
[screencast]: http://www.youtube.com/watch?v=M-l79fkmmmA
[collada]: https://github.com/pycollada/pycollada

[icdsc12]: http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=6470136&sortType%3Dasc_p_Sequence%26filter%3DAND%28p_IS_Number%3A6470120%29
[mav_aim12]: http://mavrinac.com/files/academic/mavrinac12_camsel.pdf
[mav_surv]: http://mavrinac.com/files/academic/mavrinac12_cncov.pdf
[m10_fcov]: http://mavrinac.com/files/academic/mavrinac10_fuzzycoverage.pdf
[m10_eval]: http://mavrinac.com/files/academic/mavrinac10_fcm3deval.pdf
[a11_sprg]: http://mavrinac.com/files/academic/alarcon11_sprange.pdf
[m11_load]: http://mavrinac.com/files/academic/mavrinac11_loaddist.pdf
