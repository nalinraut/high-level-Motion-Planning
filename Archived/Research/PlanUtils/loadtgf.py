from pygraph.classes.graph import graph
from pygraph.classes.digraph import digraph
#from pygraph.algorithms.searching import breadth_first_search
#from pygraph.algorithms.filters.find import find

def loadTGF(file,isDigraph=False):
    """Loads a Pygraph graph or digraph from the TGF file.
    Stores all node and edge data strings in the 'data' attribute.
    """
    g = None
    if isDigraph:
        g = digraph()
    else:
        g = graph()
    readingEdges = False
    for line in file.readlines():
        if line[0]=='#':
            if readingEdges:
                raise IOError("Found two # signs in TGF file")
            readingEdges=True
        elif not readingEdges:
            val = ''
            id = ''
            try:
                (id,val)=line.split(None,1)
            except:
                id = line
            id = id.strip()
            val = val.strip()
            attributes = []
            if len(val)>0:
                attributes = [('data',val)]
            g.add_node(id,attrs=attributes)
        else:
            id1 = id2 = val = ''
            try:
                (id1,id2,val)=line.split(None,2)
            except:
                try:
                    (id1,id2)=line.split(None,1)
                except:
                    raise IOError("Didn't find two values in line '"+line+"'")
            id1 = id1.strip()
            id2 = id2.strip()
            val = val.strip()
            attributes = []
            if len(val)>0:
                attributes = [('data',val)]        
            g.add_edge((id1,id2),attrs=attributes)
    return g


"""
f = open('test.tgf','r')
g = loadTGF(f,True)
f.close()

print g.nodes()[0:10]
print g.edges()[0:10]

print breadth_first_search(g,'1',find('1'))
"""
