#!/usr/bin/python
import asyncore
from sspp.service import Service
from threading import Thread,Lock
from Tkinter import *
import ttk

def lookup(structure,path):
    if len(path)==0:
        return structure
    key = path[0]
    if key not in structure:
        print "lookup error:",key,"not in",structure
    return lookup(structure[key],path[1:])

def nested_set(structure,path,value):
    if len(path)==0:
        return value
    key = path[0]
    if key not in structure:
        print "lookup error:",key,"not in",structure
    structure[key] = nested_set(structure[key],path[1:],value)
    return structure

class SpiffyGUI(Service):
    def __init__(self,addr):
        Service.__init__(self)
        self.open(addr)
        self.sendMessage({'type':'name','data':'Spiffy GUI'})
        self.root = Tk()
        self.tree = ttk.Treeview(self.root,height=20)
        self.tree["columns"]=("value")
        self.tree.column("value",width = 200)
        self.tree.heading("value",text="Value")
        self.tree.pack(side=LEFT,fill=Y,expand=1)
        self.tree.bind('<<TreeviewSelect>>',self.treeSelect)
        self.editFrame = ttk.Frame(self.root,width=400)
        self.editFrameLabel = ttk.Label(self.editFrame,text="Edit")
        self.editFrameLabel.pack()
        self.editFrame.pack(side=RIGHT,fill=BOTH,expand=1)
        self.editItem = None
        self.editFrameItems = []
        self.structure = {}
        self.items = {}
        
    def run(self):
        def callback():
            if self.connected:
                self.sendMessage({'type':'get','path':'.'})
            asyncore.loop(timeout = 0.01, count=1)
            self.root.after(100,callback)
        self.root.after(100,callback)
        self.root.mainloop()

    def onMessage(self,msg):
        self.setStructure(msg)

    def itemEdited(self,event):
        path,oldValue = self.editItem
        newValue = self.editFrameItems[0].get()
        strpath = '.'+'.'.join([str(p) for p in self.editItem[0]])
        print "Set value",strpath,newValue
        try:
            value = type(oldValue)(newValue)
        except ValueError:
            value = newValue
        self.sendMessage({'type':'set','path':strpath,'data':value})
        self.structure = nested_set(self.structure,path,value)
    def updateEditBox(self,item):
        if self.editItem == None or item != self.editItem[1]:
            self.editFrame.pack_forget()
            for w in self.editFrameItems:
                w.destroy()
            self.editFrameItems = []
            self.editFrameLabel.pack()
            #create edit box items
            if isinstance(item,(bool,int,str,float,unicode)):
                e = ttk.Entry(self.editFrame)
                e.bind('<Return>',self.itemEdited)
                e.delete(0,END)
                e.insert(0,str(item))
                e.pack()
                self.editFrameItems.append(e)
            elif isinstance(item,(dict,list)):
                pass
            self.editFrame.pack(side=RIGHT,fill=BOTH,expand=1)
        else:
            #modify edit box items
            if isinstance(item,(bool,int,str,float,unicode)):
                self.editFrameItems[0].delete(0,END)
                self.editFrameItems[0].insert(0,str(item))
    def treeSelect(self,arg):
        print "treeSelect",self.tree.focus()
        if self.tree.focus()==None:
            self.updateEditBox(None)
            return
        for (k,v) in self.items.iteritems():
            if v == self.tree.focus():
                value = lookup(self.structure,k)
                self.updateEditBox(value)
                self.editItem = (k,value)
    def setStructure(self,msg):
        self.present = dict((k,False) for k in self.items.iterkeys())
        self._setStructure(msg,'.','',())
        for (k,present) in self.present.iteritems():
            if not present:
                if self.tree.exists(self.items[k]):
                    self.tree.delete(self.items[k])
                del self.items[k]
        if self.editItem != None:
            if not self.present[self.editItem[0]]:
                self.updateEditBox(None)
                self.editItem = None
            else:
                newval = lookup(msg,self.editItem[0])
                if newval != self.editItem[1]:
                    self.updateEditBox(newval)
                    self.editItem = (self.editItem[0],newval)
        self.structure = msg
        self.present = {}

    def _setStructure(self,item,name,parent,path):
        #mark
        self.present[path] = True
        #add if not present
        if isinstance(item,list):
            if path not in self.items:
                treeitem = self.tree.insert(parent,len(self.items),text=name)
                self.tree.see(treeitem)
                self.items[path] = treeitem
            else:
                treeitem = self.items[path]
            for i,subitem in enumerate(item):
                self._setStructure(subitem,'['+str(i)+']',treeitem,path+(i,))
        elif isinstance(item,dict):
            if path not in self.items:
                treeitem = self.tree.insert(parent,len(self.items),text=name)
                self.tree.see(treeitem)
                self.items[path] = treeitem
            else:
                treeitem = self.items[path]                
            for key,subitem in item.iteritems():
                self._setStructure(subitem,key,treeitem,path+(key,))
        else:
            if path not in self.items:
                treeitem = self.tree.insert(parent, len(self.items), text=name, values=(str(item),))
                self.tree.see(treeitem)
                self.items[path] = treeitem
            else:
                treeitem = self.items[path]
                self.tree.item(treeitem,values=(str(item),))

gui = SpiffyGUI(('localhost',4567))
gui.run()
