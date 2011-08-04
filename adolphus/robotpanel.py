"""\
Robot panel module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import threading
import visual.controls

from .visualization import visual


class RobotPanel(threading.Thread):
    """\
    Robot configurator panel.
    """
    def __init__(self, robot):
        """\
        Constructor.

        @param robot: The robot to configure.
        @type robot: L{posable.Robot}
        """
        super(RobotPanel, self).__init__()
        self.robot = robot
        self.config = robot.config
        self.panel = visual.controls.controls(height=len(robot.joints) * 20 \
            - 20, range=100)
        offset = (self.panel.height / 2)
        self.slider = []
        for i, joint in enumerate(robot.joints):
            self.slider.append(visual.controls.slider(pos=(-50, offset),
                width=5, length=100, text=joint['name'], axis=(1, 0, 0)))
            self.slider[i].value = 100.0 * (self.config[i] - \
                joint['limits'][0]) / (joint['limits'][1] - joint['limits'][0])
            self.slider[i].action = lambda: self.update_config()
            offset -= 20
        visual.controls.button()
        self.quit = False

    def run(self):
        """\
        Run the robot configurator thread.
        """
        while not self.quit: 
            visual.rate(50)
            self.panel.interact()
        self.panel.display.visible = False

    def update_config(self):
        """\
        Update the configuration and visualization of the robot.
        """
        for i, joint in enumerate(self.robot.joints):
            self.config[i] = (self.slider[i].value / 100.0) \
                * (joint['limits'][1] - joint['limits'][0]) + joint['limits'][0]
        self.robot.config = self.config
        self.robot.update_visualization()
