#!/usr/bin/python

from topic import TopicServer

print "Starting topic server on localhost:4567"
service = TopicServer(('localhost',4567))
#processes messages at 100Hz
service.run(100)
