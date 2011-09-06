#!/usr/bin/env python

from optparse import OptionParser

from adolphus import Experiment, Panel


def main():
    parser = OptionParser()
    parser.add_option('-c', '--conf', dest='conf', default=None, 
        help='custom configuration file to load')
    parser.add_option('-z', '--zoom', dest='zoom', default=False,
        action='store_true', help='disable camera view and use visual zoom')

    opts, args = parser.parse_args()
    try:
        model_file = args[0]
    except IndexError:
        model_file = None

    experiment = Experiment(model_file, config_file=opts.conf, zoom=opts.zoom)
    panel = Panel(experiment=experiment)
    panel.main()


if __name__ == '__main__':
    main()
