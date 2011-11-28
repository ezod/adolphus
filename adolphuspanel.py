#!/usr/bin/env python

from optparse import OptionParser

from adolphus.panel import Panel


def panel_main(host, port):
    Panel(host=host, port=port).main()

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-H', '--host', dest='host', default='localhost',
        help='hostname to connect to')
    parser.add_option('-p', '--port', dest='port', default=None, type=int,
        help='port for the socket controller to connect')
    opts, args = parser.parse_args()
    panel_main(opts.host, opts.port)
