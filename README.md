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

####Journals

* A. Mavrinac, X. Chen, and J.L. Alarcon-Herrera, "[Semi-Automatic Model-Based View Planning for Active Triangulation 3D Inspection Systems][tmech-m-14]," to appear in *IEEE/ASME Trans. on Mechatronics*, 2014.
* A. Mavrinac, X. Chen, and Y. Tan, "[Coverage Quality and Smoothness Criteria for Real-Time View Selection in a Multi-Camera Network][tosn-m-14]," in *ACM Trans. Sensor Networks*, vol. 10, no. 2, 2014.
* J.L. Alarcon-Herrera and X. Chen, "Graph-Based Deployment of Visual Sensor Networks and Optics Optimization,” submitted to *IEEE/ASME Transactions on Mechatronics*, manuscript no. TMECH-07-2014-3852, 2014.
* X. Zhang, X. Chen, J.L. Alarcon-Herrera, and Y. Fang, "3D Model-based Multi-Camera Deployment: A Recursive Convex Optimization," submitted to *IEEE/ASME Transactions on Mechatronics*, manuscript no. TMECH-07-2014-3843, 2014.
* A. Mavrinac and X. Chen, "[Modeling Coverage in Camera Networks: A Survey] [mav_surv]," in *International Journal of Computer Vision*, vol. 101, no. 1, pp. 205-226, 2013.

####Conference Proceedings

* J.L. Alarcon-Herrera, X. Chen, and X. Zhang, "Viewpoint Selection for Vision Systems in Industrial Inspection,” in *IEEE International Conference on Robotics and Automation*, 2014, pp. 4934-4939.
* J.L. Alarcon-Herrera and X. Chen, "[Deployment of Visual Sensor Networks Using a Graph-Based Approach][iscas-14],” in *IEEE International Symposium on Circuits and Systems*, 2014, pp. 2575-2578.
* X. Zhang, J.L. Alarcon-Herrera and X. Chen, "Optimization for 3D Model-based Multi-Camera Deployment,” accepted for publication in *World Congress of the International Federation of Automatic Control*, 2014.
* J.L. Alarcon-Herrera and X. Chen, ”Graph-Based Visual Sensor Deployment and Optics Optimization,” accepted for publication in *Chinese Control Conference*, 2014.
* J.L. Alarcon-Herrera, X. Chen, and M. Ahmadi, "[Re-Configuration Strategy for PTZ Camera Networks][ifac-a-13]," in *IFAC Symposium on Mechatronic Systems*, 2013, pp. 563-568.
* J.L. Alarcon-Herrera and X. Chen, "[Online Configuration of PTZ Camera Networks] [icdsc12],” in *ACM/IEEE International Conference on Distributed Smart Cameras*, 2012.
* J.L. Alarcon-Herrera and X. Chen, "[Consensus Algorithms in a Multi-Agent Framework to Solve PTZ Camera Reconfiguration in UAVs][icira-a-12]," in *International Conference on Intelligent Robotics and Applications*, 2012, pp. 331-340.
* K. Shaya, A. Mavrinac, J.L. Alarcon-Herrera, and X. Chen, "[A Self-Localization System with Global Error Reduction and Online Map-Building Capabilities][icira-s-12]," in *International Conference on Intelligent Robotics and Applications*, 2012, pp. 13-22.
* A. Mavrinac, D. Rajan, Y. Tan, and X. Chen, "[Task-Oriented Optimal View Selection in a Calibrated Multi-Camera System] [mav_aim12]," in *IEEE/ASME International Conference on Advanced Intelligent Mechatronics*, 2012.
* A. Mavrinac and X. Chen, "[Optimizing Load Distribution in Camera Networks with a Hypergraph Model of Coverage Topology] [m11_load]," in *ACM/IEEE International Conference on Distributed Smart Cameras*, 2011.
* J. L. Alarcon-Herrera, A. Mavrinac, and X. Chen, "[Sensor Planning for Range Cameras via a Coverage Strength Model] [a11_sprg]," in *IEEE/ASME International Conference on Advanced Intelligent Mechatronics*, 2011, pp. 838-843.
* A. Mavrinac, J. L. Alarcon-Herrera, and X. Chen, "[Evaluating the Fuzzy Coverage Model for 3D Multi-Camera Network Applications] [m10_eval]," in *International Conference on Intelligent Robotics and Applications*, 2010, pp. 692-701.
* A. Mavrinac, J. L. Alarcon-Herrera, and X. Chen, "[A Fuzzy Model for Coverage Evaluation of Cameras and Multi-Camera Networks] [m10_fcov]," in *ACM/IEEE International Conference on Distributed Smart Cameras*, 2010, pp. 95-102.


[python]: http://www.python.org
[cython]: http://cython.org
[pyyaml]: http://pyyaml.org
[visual]: http://vpython.org
[epydoc]: http://epydoc.sourceforge.net
[pygtk]: http://www.pygtk.org/
[setuptools]: http://pypi.python.org/pypi/setuptools
[screencast]: http://www.youtube.com/watch?v=M-l79fkmmmA
[collada]: https://github.com/pycollada/pycollada

[iscas-14]:http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6865699&refinements%3D4277101843%26sortType%3Dasc_p_Sequence%26filter%3DAND%28p_IS_Number%3A6865048%29
[ifac-a-13]:http://www.ifac-papersonline.net/Detailed/59145.html
[icira-a-12]:http://dl.acm.org/citation.cfm?id=2404301#!
[icira-s-12]:http://dl.acm.org/citation.cfm?id=2404347#!
[tosn-m-14]:http://mavrinac.com/files/academic/mavrinac14_camsel.pdf
[tmech-m-14]:http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6818412&tag=1
[icdsc12]: http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=6470136&sortType%3Dasc_p_Sequence%26filter%3DAND%28p_IS_Number%3A6470120%29
[mav_aim12]: http://mavrinac.com/files/academic/mavrinac12_camsel.pdf
[mav_surv]: http://mavrinac.com/files/academic/mavrinac12_cncov.pdf
[m10_fcov]: http://mavrinac.com/files/academic/mavrinac10_fuzzycoverage.pdf
[m10_eval]: http://mavrinac.com/files/academic/mavrinac10_fcm3deval.pdf
[a11_sprg]: http://mavrinac.com/files/academic/alarcon11_sprange.pdf
[m11_load]: http://mavrinac.com/files/academic/mavrinac11_loaddist.pdf
