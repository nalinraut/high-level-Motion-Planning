#ifndef GEOMETRY_CLUSTERIZE_H
#define GEOMETRY_CLUSTERIZE_H

#include <KrisLibrary/Graph/UndirectedGraph.h>
#include "Averager.h"
#include <map>

template <typename T,typename EqualFunc>
void Clusterize(std::vector<T>& items,EqualFunc& eq)
{
  typedef Averager<T> NodeType;
  Graph::UndirectedGraph<NodeType,int> net;
  net.Resize(items.size());
  for(size_t i=0;i<items.size();i++) {
    net.nodes[i].value = items[i];
    net.nodes[i].weight = 1;
  }
  //swap(items,net.nodes);
  for(size_t i=0;i<items.size();i++) {
    for(size_t j=i+1;j<items.size();j++) {
      if(eq(net.nodes[i].value,net.nodes[j].value))
	net.AddEdge(i,j);
    }
  }
  std::vector<int> oneRing;
  std::vector<int> twoRing;
  std::multimap<size_t,size_t> queue;
  //order by degree
  for(size_t i=0;i<net.nodes.size();i++) {
    size_t d=net.Degree(i);
    if(d > 0) 
      queue.insert(std::pair<size_t,size_t>(d,i));
  }
  while(!queue.empty()) {
    size_t index = queue.begin()->second;
    queue.erase(queue.begin());
    Assert(index < net.nodes.size());

    //collapse the 1-ring associated with index to a single vertex
    oneRing.resize(0);
    typename Graph::UndirectedGraph<NodeType,int>::Iterator e;
    for(net.Begin(index,e);!e.end();e++) 
      oneRing.push_back(e.target());

    //collapse
    for(size_t i=0;i<oneRing.size();i++) {
      int t=oneRing[i];
      net.nodes[index].inc(net.nodes[t]);
    }

    net.DeleteOutgoingEdges(index);
    net.DeleteIncomingEdges(index);
    net.DeleteNodes(oneRing);
    //oneRing is now set to the node remapping
    Assert(oneRing[index] >= 0 && oneRing[index] < (int)net.nodes.size());
    index = (size_t)oneRing[index];
    std::multimap<size_t,size_t>::iterator i=queue.begin();
    while(i!=queue.end()) {
      Assert(i->second < oneRing.size());
      if(oneRing[i->second] < 0) {
	std::multimap<size_t,size_t>::iterator n=i; ++n;
	queue.erase(i);
	i=n;
      }
      else {
	i->second = (size_t)oneRing[i->second];
	Assert(i->second >= 0 && i->second < net.nodes.size());
	i++;
      }
    }

    //TODO: speed this up by only doing the two-ring
    //update the proximity to index
    for(size_t i=0;i<net.nodes.size();i++) 
      if(i!=index && eq(net.nodes[i].value,net.nodes[index].value))
	net.AddEdge(i,index);
    
    size_t d=net.Degree(index);
    if(d > 0) queue.insert(std::pair<size_t,size_t>(d,index));
  }
  items.resize(net.nodes.size());
  for(size_t i=0;i<net.nodes.size();i++) {
    items[i] = net.nodes[i].value;
  }
  //swap(cp,net.nodes);
}


#endif
