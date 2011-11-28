#!/usr/bin/env python

import sys
from optparse import OptionParser

from adolphus.interface import Experiment, Controller


def viewer_main(modelfile=None, config='', zoom=False, port=None):
    experiment = Experiment(zoom=zoom)
    if modelfile:
        experiment.execute('loadmodel %s' % modelfile)
    experiment.execute('loadconfig %s' % config)
    if port is not None:
        controller = Controller(experiment, port=port)
        print('Listening on port %d.' % controller.port)
        sys.stdout.flush()
        controller.main()
    else:
        experiment.start()
        experiment.join()


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-s', '--socket', dest='socket', default=False,
        action='store_true', help='enable socket controller')
    parser.add_option('-p', '--port', dest='port', default=0, type=int,
        help='port for the socket controller to listen on')
    parser.add_option('-c', '--conf', dest='conf', default='', 
        help='custom configuration file to load')
    parser.add_option('-z', '--zoom', dest='zoom', default=False,
        action='store_true', help='disable camera view and use visual zoom')
    opts, args = parser.parse_args()
    if opts.socket:
        port = opts.port
    else:
        port = None
    viewer_main(modelfile=(args and args[0] or None), config=opts.conf,
        zoom=opts.zoom, port=port)
