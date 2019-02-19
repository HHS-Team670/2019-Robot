#!/usr/bin/python

from subprocess import Popen
import sys

filename = sys.argv[1]
while True:
    p = Popen(filename, shell=True)
    p.wait()
