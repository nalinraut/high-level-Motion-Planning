#!/usr/bin/python

import time
from sspp import spiffy
from sspp.spiffy import Spiffy

try:
    print "Starting Spiffy client"
    spiffy.start(('localhost',4567),context='.spiffy_test')

    print "Creating variable foo"
    foo = Spiffy('foo')
    print "Foo 1 uninitialized",foo.get()
    foo.set(100)
    print "Foo 1 after initialization",foo.get()

    print "Creating second variable foo"
    foo2 = Spiffy('foo')
    foo2.setPushMode()
    print "Foo 2",foo2.get()
    time.sleep(0.1)
    print "Foo 2 after sleep",foo2.get()

    foo.set(200)
    print "Foo 2 immediately after foo1 set",foo2.get()
    
    print "Foo 1 after foo1 set",foo.get()

    print "Foo 2 after foo1 set and wait",foo2.get()

    print "Creating third variable foo and getting write-only access"
    foo3 = Spiffy('foo')
    foo3.setExclusiveWrite()
    print "Setting to 'bar'"
    foo3.set('bar')
    print "Foo 1 after foo 3 set",foo.get()
    print "Foo 2 after foo 3 set",foo2.get()
    print "Trying to set foo2 to 'bar'"
    print "  Result:",foo2.set(100)
    print "Foo 3 value",foo3.get()
    

finally:
    print "Stopping Spiffy client"
    spiffy.stop()
