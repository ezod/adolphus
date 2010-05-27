#!/bin/sh

echo "Generating documentation for Adolphus..."
rm -rf doc/
epydoc -v --name Adolphus -o doc ./coverage
