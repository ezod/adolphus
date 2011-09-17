#!/usr/bin/env python

from optparse import OptionParser

from adolphus import Panel


def main():
    parser = OptionParser()
    parser.add_option('-H', '--host', dest='host', default=None,
        help='hostname to connect to')
    parser.add_option('-p', '--port', dest='port', default=None, type=int,
        help='port for the socket controller to listen on')

    opts, args = parser.parse_args()

    Panel(host=opts.host, port=opts.port).main()

if __name__ == '__main__':
    main()
