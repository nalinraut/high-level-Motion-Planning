#! /usr/bin/env python

from reflex_sf_hand import ReflexSFHand

if __name__ == '__main__':
    import sys
    if len(sys.argv) <= 1:
        print "USAGE: rosrun reflex_sf calibrate.py NAME"
        exit(0)
    rfx = ReflexSFHand(sys.argv[1])
    rfx.calibrate()
