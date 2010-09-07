# Adolphus - Multi-Camera Network Coverage Modeling Suite


## Overview

Adolphus is a set of tools for modeling and visualizing the coverage of
multi-camera networks.


## Requirements

Adolphus requires [Python] [1] 2.6 or later, [FuzzPy] [2] 0.4.0 or later, and
[PyYAML] [3] 3.09 or later.

The [Visual] [4] module of VPython is required for visualization (optional,
recommended).

[Epydoc] [5] is required for generating API documentation (optional).


## Visualization Interface

Within the visualization, right-click drag controls rotation and middle-click
drag controls zoom. The left/right, down/up, and page-down/page-up keys control
the position of the center of rotation in the *x*, *y*, and *z* directions,
respectively.

Left-clicking a camera toggles the active state of the camera in the model
(inactive cameras are shown semi-transparent with a red indicator light).
CTRL-left-clicking a camera toggles displaying an approximation of its field of
view boundaries. ALT-left-clicking a camera enters camera view.

Left-clicking a stored point displays the coordinates and membership degree of
the point in the controlling terminal.

Global functions are controlled by the function keys as follows:

* **F2**: recompute the FCM for the stored points
* **F3**: recompute coverage performance for all relevance models
* **F5**: toggle display of membership degree tags for points
* **F6**: toggle display of camera name tags
* **F7**: toggle display of world coordinate system axes
* **F8**: toggle display of the center of rotation
* **F11**: exit camera view
* **F12**: end interactive event loop (to run subsequent code)


## Related Publications

* A. Mavrinac, J. L. Alarcon Herrera, and X. Chen, "[A Fuzzy Model for Coverage Evaluation of Cameras and Multi-Camera Networks] [6]," *Proc. 4th ACM/IEEE Intl. Conf. on Distributed Smart Cameras*, 2010.
* A. Mavrinac, J. L. Alarcon Herrera, and X. Chen, "[Evaluating the Fuzzy Coverage Model for 3D Multi-Camera Network Applications] [7]," *Proc. Intl. Conf. on Intelligent Robotics and Applications*, 2010.


[1]: http://www.python.org
[2]: http://mavrinac.com/index.cgi?page=fuzzpy
[3]: http://pyyaml.org
[4]: http://vpython.org
[5]: http://epydoc.sourceforge.net
[6]: http://mavrinac.com/files/academic/mavrinac10_fuzzycoverage.pdf
[7]: http://mavrinac.com/files/academic/mavrinac10_fcm3deval.pdf
