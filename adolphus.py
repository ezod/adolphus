#!/usr/bin/env python

import sys
import os
import subprocess


def main():
    vr, pw = os.pipe()
    pr, vw = os.pipe()
    argviewer = [sys.executable, os.path.join(os.path.dirname(sys.argv[0]),
        'adolphusviewer.py'), '-s'] + sys.argv[1:]
    viewer = subprocess.Popen(argviewer, stdin=vr, stdout=vw)
    argpanel = [sys.executable, os.path.join(os.path.dirname(sys.argv[0]),
        'adolphuspanel.py')]
    panel = subprocess.Popen(argpanel, stdin=pr, stdout=pw)
    viewer.wait()
    panel.wait()


if __name__ == '__main__':
    main()
