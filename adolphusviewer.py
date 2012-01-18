#!/usr/bin/env python

try:
    import cPickle as pickle
except ImportError:
    import pickle

from sys import stdin, stdout
from threading import Thread
from optparse import OptionParser

from adolphus.interface import Experiment
from adolphus.commands import CommandError

def receive(experiment, rf):
    while not experiment.exit:
        try:
            cmd = stdin.readline().rstrip()
        except IOError:
            continue
        try:
            response = experiment.execute(cmd, response=rf)
        except CommandError, e:
            response = pickle.dumps(e)
        except IndexError:
            continue
        if not response:
            response = pickle.dumps(None)
        stdout.write(response + '\n')
        stdout.flush()


def viewer_main(modelfile=None, config='', zoom=False, server=False,
                response='pickle'):
    experiment = Experiment(zoom=zoom)
    if modelfile:
        experiment.execute('loadmodel %s' % modelfile)
    experiment.execute('loadconfig %s' % config)
    experiment.start()
    if server:
        receiver = Thread(target=receive, args=(experiment, response))
        receiver.start()
    experiment.join()


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-c', '--conf', dest='conf', default='',
        help='custom configuration file to load')
    parser.add_option('-z', '--zoom', dest='zoom', default=False,
        action='store_true', help='disable camera view and use visual zoom')
    parser.add_option('-s', '--server', dest='server', default=False,
        action='store_true', help='listen for commands on standard input')
    parser.add_option('-r', '--response', dest='response', default='pickle',
        help='command response format')
    opts, args = parser.parse_args()
    viewer_main(modelfile=(args and args[0] or None), config=opts.conf,
        zoom=opts.zoom, server=opts.server, response=opts.response)
