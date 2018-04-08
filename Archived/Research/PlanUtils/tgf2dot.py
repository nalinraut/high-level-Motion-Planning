#!/usr/bin/python
import sys

if len(sys.argv) < 2:
    print("Usage: tgf2dot file.tgf")

# maps id strings to data
nodes = dict({})
# maps (id1,id2) strings to data
edges = dict({})

f = open(sys.argv[1],'r')
readingEdges = False
index=0
for line in f.readlines():
    index += 1
    line = line.lstrip().rstrip()
    if not readingEdges:
        if line[0]=='#':
            readingEdges=True
        else:
            (id,space,line) = line.partition(' ')
            if space=='':
                id = line
            if id in nodes:
                raise ValueError("Duplicate node %s on line %d"%(id,index))
            #save it
            nodes[id] = line
    else:
        if len(line)==0:
            #done
            break
        (id1,space,line) = line.partition(' ')
        if space=='':
            raise IOError("Couldn't read first id from line %d"%(index))
        (id2,space,line) = line.partition(' ')
        if id2=='' and space=='':
            id2=line
        if (id1,id2) in edges:
            raise ValueError("Duplicate edge (%s,%s) on line %d"%(id1,id2,index))
        #save it
        edges[(id1,id2)]=line
f.close()

print "Done reading TGF file"
print "%d nodes, %d edges"%(len(nodes),len(edges))

print "Writing to graph.dot"
f = open("graph.dot","w")
f.write("graph G {\n")
for (node,data) in nodes.items():
    f.write("\t%s [shape=circle,label=\"%s\"]\n"%(node,data))
for ((id1,id2),data) in edges.items():
    f.write("\t%s -- %s [label=\"%s\"]\n"%(id1,id2,data))
f.write("}\n")
f.close()
