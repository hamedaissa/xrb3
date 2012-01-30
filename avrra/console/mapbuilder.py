#!/usr/bin/env python2.5
""" MapBuilder Add-In

Add-in for XR console that displays maps built by the
mapbuilder behavior (see example\mapbuilder.h).

Further information at:
    http://www.blunderingbotics.com/avrra/ """

__author__ = "Michael Ferguson <mfergs7@gmail.com>"
__date__ = "26 December 2008"
__version__ = "$Revision: 1.0 $"
__copyright__ = "Copyright 2008 Michael E. Ferguson"

from console2 import *
from Tkinter import *
from tkFileDialog   import askopenfilename, asksaveasfilename

MAPBUILDER_MODE_NONE = 0
MAPBUILDER_MODE_INMAP = 1

class interface(Dialog):

    def __init__(self,parent,title=None):
        Dialog.__init__(self,parent,"Map Builder")

    # initialize the mapbuilder interface
    def body(self, master):
        # mapbuilder data structures
        self.halt = self.parent.halt        
        self.mode = MAPBUILDER_MODE_NONE
        self.map_data = []     
        # 400 = 25 * 16 (15 + 1)
        self.map = Canvas(master, width=399, height=399)
        self.map.pack()
        self.save = Button(master, text="Save Map", command=self.savemap)
        self.save.pack(side=RIGHT, pady=5)
        self.load = Button(master, text="Load Map", command=self.loadmap)
        self.load.pack(side=RIGHT, pady=5, padx=5) 
        self.pause = Button(master, text="Pause", command=self.pauseButton)
        if self.halt:
            self.pause.config(text="GO")
        self.pause.pack(side=RIGHT, pady=5, padx=5)         
        for x in range(25):
            self.map.create_line(0, x * 16, 399, x * 16, fill="#C0C0C0")
            self.map.create_line(x *16, 0, x *16, 399, fill="#C0C0C0")
        for x in range(25):
            self.map_data.append(list())
            for y in range(25):
                # each entry in map_data[x][y] is a list[display, int]
                self.map_data[x].append([self.map.create_rectangle(x*16+1,y*16+1,x*16+16,y*16+16, fill="#F0F0F0", width=0),0])
                pass
        
    def buttonbox(self):
        pass

    def pauseButton(self):
        if self.halt:
            # robot is paused
            self.pause.config(text="Pause")
            self.parent.ser.write("GO\n")
            self.halt = False
            self.mode = MAPBUILDER_MODE_NONE
        else:
            self.pause.config(text="Go")
            self.parent.ser.write("HT\n")
            self.halt = True
        self.parent.halt = self.halt

    # fills a box in the map
    def boxFill(self, x, y, val):
        self.map_data[x][y][1] = val
        if val == 0:
            self.map.itemconfigure(self.map_data[x][y][0], fill = "#F0F0F0")
        elif val > 10:
            self.map.itemconfigure(self.map_data[x][y][0], fill = "#FF0000")
        elif val < 2:
            self.map.itemconfigure(self.map_data[x][y][0], fill = "#CCCCCC")
        elif val < 3:
            self.map.itemconfigure(self.map_data[x][y][0], fill = "#999999")
        elif val < 5:
            self.map.itemconfigure(self.map_data[x][y][0], fill = "#333333")
        else:
            self.map.itemconfigure(self.map_data[x][y][0], fill = "#000000")
    
    # load a map from a file    
    def loadmap(self, event=None):
        filename = askopenfilename()        
        print "MapBuilder: load " + filename
        if filename == None:
            return
        new_map = open(filename,"r").readlines()    
        x = 0
        y = 0
        for line in new_map:
            x = 0
            #print line[0:-1]
            for c in range(25):
                if line[c*2] == " ":
                    self.boxFill(x,y,0)
                elif line[c*2] >= '0' and line[c*2] <= '9':
                    self.boxFill(x,y,int(line[c*2]))
                else:
                    self.boxFill(x,y,line[c*2])
                    print line[c*2]
                x += 1
            y += 1

    # save a map to a file
    def savemap(self, event=None):
        filename = asksaveasfilename()
        print "MapBuilder: save to " + filename
        if filename == None:
            return
        outmap = open(filename, "w")
        for y in range(25):
            for x in range(25):
                if self.map_data[x][y][1] == 0:     # empty space
                    print>>outmap, "0",
                elif self.map_data[x][y][1] > 10:   # robot position
                    print>>outmap, chr(self.map_data[x][y][1]), 
                else:                               # wall hit count
                    print>>outmap, str(self.map_data[x][y][1]),
            print>>outmap                           # \n after all points
        outmap.close()

    # handler for serial data
    def serialMsg(self, line):
        if line[0:4] == "#map":
            self.mode = MAPBUILDER_MODE_INMAP
            self.X = 0
            self.Y = 0       
        elif line[0:4] == "#end":
            self.mode = MAPBUILDER_MODE_NONE  
        elif self.mode == MAPBUILDER_MODE_INMAP:
            # map info = raw numbers plus ^, >, <, and R
            for c in line[0:-1]:
                self.boxFill(self.X,self.Y,ord(c))
                self.X += 1
                if self.X > 25: # temporary hack
                    self.Y += 1
                    self.X = 0
            # reset counters for next line of map
            self.Y += 1
            self.X = 0

