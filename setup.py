#!/usr/bin/env python

from adolphus import __version__
VERSION = "%s.%s.%s" % __version__[0:3]

from setuptools import setup
from distutils.cmd import Command
from shutil import rmtree
from glob import glob
import os
import sys
import epydoc.cli

NAME = 'adolphus'
URL = 'http://github.com/ezod/adolphus'
PACKAGE = 'adolphus'

class GenerateDoc(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        rmtree('doc', ignore_errors=True)
        os.mkdir('doc')
        sys.argv = ['epydoc', '-v', '--name', NAME, '--url', URL, '-o', 'doc', PACKAGE]
        options, names = epydoc.cli.parse_arguments()
        epydoc.cli.main(options, names)

setup(
    name = NAME,
    version = VERSION,
    license = "GPL",
    description = "Multi-camera network coverage modeling suite.",
    author = "Aaron Mavrinac",
    author_email = "mavrin1@uwindsor.ca",
    url = URL,
    keywords = "fuzzy vision multicamera model",
    packages = [PACKAGE],
    #test_suite = "test",
    cmdclass = {'doc': GenerateDoc},
)
