#!/usr/bin/env python

import sys
import os.path
import subprocess


def main():
    argviewer = [os.path.join(os.path.dirname(sys.argv[0]),
        'adolphusviewer.py'), '-s'] + sys.argv[1:]
    viewer = subprocess.Popen(argviewer, stdout=subprocess.PIPE)
    port = viewer.stdout.readline().rstrip().strip('.').split(' ')[3]
    argpanel = [os.path.join(os.path.dirname(sys.argv[0]), 'adolphuspanel.py'),
        '-H', 'localhost', '-p', port]
    panel = subprocess.Popen(argpanel)
    viewer.wait()


if __name__ == '__main__':
    main()
