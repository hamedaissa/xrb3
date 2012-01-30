#!/usr/bin/env python2.5
""" Connects to 2nd Generation Vanadium XR Series Robots

This program connects to the XR Series of robots via XBEE
wireless. More information can be found at:

    http://www.blunderingbotics.com/avrra/ """

__author__ = "Michael Ferguson <mfergs7@gmail.com>"
__date__ = "9 October 2009"
__version__ = "$Revision: 0.7.2 $"
__copyright__ = "Copyright 2008, 2009 Michael E. Ferguson"

# TODO: camera functionality

import sys, Tkinter, time, serial, threading, re

# Implements the main window. 
class Console(Tkinter.Tk):
    def __init__(self):
        """ Creates connection to robot. See also the XR Config program. """
        Tkinter.Tk.__init__(self) 
        print "Starting XR Console Version 0.7.2"
        # important variables
        self.ser = None
        self.halt = True
        self.lastCmd = ""        
        self.behcount = 0
        self.numout = 0      

        # setup our main window
        self.config(bg = "#BBC6A1")         # make window nice green color
        self.resizable(0,0)

        # get settings (robots, ports, and addins) from configuration file
        self.ADDINS = [] 
        self.ROBOT_NAME = ""       
        try:
            confFile = open('xr.conf', 'r').readlines()
        except:
            print "Unable to open xr.conf"
            self.comPort = "/dev/ttyUSB0"
            self.baudRate = 38400
        # read in data, strip off \n's
        self.comPort = confFile[0][0:-1]
        print "comPort:", self.comPort
        for line in confFile[1:]:
            if line.find("ADDIN:") == 0: 
                print "Addin: " + str(line[6:-1].split(":"))
                self.ADDINS.append(line[6:-1].split(":"))                       
            elif line.find("#") == -1 and line.find("ADDIN:") != 0 and self.ROBOT_NAME == "": 
                robot = line[0:-1].split(",")
                self.ROBOT_NAME = robot[0]
                self.baudRate = robot[1]                
        print "baudRate:", self.baudRate
        self.title('Console Beta3: ' + self.ROBOT_NAME)

        # setup the main window - left side =  behaviors box
        self.behBox = Tkinter.Listbox(self,width=25)
        self.behBox.grid(rowspan=6,columnspan=2,sticky=Tkinter.W+Tkinter.N+Tkinter.S,padx=5,pady=5)
        self.behBox.bind("<ButtonRelease-1>",self.switchBeh)
        self.curbeh = -1
        
        # add-ins/settings buttons
        Tkinter.Button(self, text="Add-ins", command=self.showAddins).grid(row=6,sticky=Tkinter.E,padx=2,ipadx=5)
        Tkinter.Button(self, text="Settings",command=self.showSettings).grid(row=6,column=1,sticky=Tkinter.W,padx=2,ipadx=5)
        
        # right side = raw/seg cam
        Tkinter.Button(self, text="Raw Cam", command=lambda: self.ser.write("DR\r"), state=Tkinter.DISABLED).grid(row=0,column=2,sticky=Tkinter.W,pady=5,padx=5,ipadx=5)
        Tkinter.Button(self, text="Seg Cam", command=lambda: self.ser.write("DS\r"), state=Tkinter.DISABLED).grid(row=1,column=2,sticky=Tkinter.W,padx=5,ipadx=5)
        # Robot output label/list (list and scrollbar are inside a frame)
        Tkinter.Label(self, text="Robot Output:", bg="#BBC6A1").grid(row=2,column=2,
                                                                     sticky=Tkinter.W,pady=2,padx=3)
        self.outFrame = Tkinter.Frame(self)
        self.outFrame.grid(row=3,column=2,padx=5)
        self.outBox = Tkinter.Listbox(self.outFrame,width=18)
        self.outScroll = Tkinter.Scrollbar(self.outFrame, orient=Tkinter.VERTICAL)
        self.outBox.config(yscrollcommand=self.outScroll.set)
        self.outScroll.config(command=self.outBox.yview)
        self.outBox.pack(side=Tkinter.LEFT, fill=Tkinter.BOTH)
        self.outScroll.pack(side=Tkinter.RIGHT, fill=Tkinter.Y)

        # send label/textbox
        Tkinter.Label(self, text="Send:", bg="#BBC6A1").grid(row=4,column=2,sticky=Tkinter.W,padx=3)
        self.sendBox = Tkinter.Entry(self)
        self.sendBox.grid(row=5,column=2,pady=5)
        self.sendBox.bind("<Return>", self.sendCmd)
        self.sendBox.bind("<Key>", self.sendUp)

        # Halt/go button
        self.stop = Tkinter.Button(self, text="Go!")
        self.stop.grid(row=6,column=2,sticky=Tkinter.E,pady=2,padx=5,ipadx=5)
        self.stop.config(command=self.stopButton)

        # regular expressions
        self.behNum = re.compile(r"[a-zA-Z]")
        self.behName = re.compile(r"[0-9]")

        self.ser = ConsoleSerial(self)
        self.ser.open(self.baudRate, self.comPort)    
        t = threading.Thread(target=self.ser.thread)
        t.start()
        self.protocol("WM_DELETE_WINDOW",self.end);
        self.ser.write("QU\n")
        self.mainloop()

    # Unlike W, we have defined our exit strategy
    def end(self):
        print "Exiting..."
        self.ser.exitcond = True
        self.destroy()    
    
    # switches behaviors when user selects a new one
    def switchBeh(self,event):
        newbeh = int(self.behBox.curselection()[0])
        if newbeh != self.curbeh:
            # new behavior selected - add the #
            newname = self.behBox.get(newbeh)
            self.behBox.delete(newbeh)
            self.behBox.insert(newbeh, newname[0:2] + "#" + newname[3: ])
            # remove the # from old behavior
            s = self.behBox.get(self.curbeh)
            self.behBox.delete(self.curbeh)
            self.behBox.insert(self.curbeh, s[0:2] + "  " + s[3: ])
            # inform robot
            self.ser.write("BB " + str(newbeh) + "\n")
            self.curbeh = newbeh       
            # if the new behavior name matches an add-in name, load the add-in 
            for addin in self.ADDINS:
                if addin[0] == newname[4:]:
                    loadDialog(newname[4:], self)
                    break

    # show a dialog witha  list of robots to control
    def showSettings(self, event=None):
        dialog = SettingsDialog(self, "Settings")    

    # show a dialog with a list of possible addins    
    def showAddins(self, event=None):
        dialog = AddinsDialog(self, "Add-ins")   
    
    # sends the current command in the user entry box, saves as last command, and clears the box
    def sendCmd(self, event):
        self.lastCmd = self.sendBox.get()
        self.ser.write(self.sendBox.get() + "\n")
        print self.sendBox.get() + ":",
        self.sendBox.delete(0,Tkinter.END)

    # brings up last command in user entry box
    def sendUp(self, event):
        if event.keycode == 98: #38:
            self.sendBox.delete(0,Tkinter.END)
            self.sendBox.insert(0,self.lastCmd)        

    # handles stop/go button
    def stopButton(self):
        if self.halt:
            # robot is paused
            self.stop.config(text="!HALT!")
            self.ser.write("GO\n")
            self.halt = False
        else:
            self.stop.config(text="Go!")
            self.ser.write("HT\n")
            self.halt = True

    # process a line of serial data
    def serialMsg(self, line):
        line = line.rstrip()
        print line[1:]
        if line[0] == '?':
            # is a message
            self.outBox.insert(Tkinter.END,line[1:])
            self.numout = self.numout + 1
            if self.numout > 30:
                 self.outBox.delete(0)
                 self.numout = self.numout -1
            if line.startswith("?Uploading"):
                 self.behBox.delete(0,Tkinter.END)
                 self.curbeh = -1
                 self.behcount = 0        
            self.outBox.see(self.numout-1)    # scroll to end...   
        elif line[0:4] == "#def":
            # is a behavior, or variable setting
            name = self.behName.sub("", line[4:])            
            num = self.behNum.sub("", line[4:])
            if int(num) > 99:
                self.behBox.insert(Tkinter.END, str(self.behcount) + ".#" + name)
                self.curbeh = self.behcount
            else:
                self.behBox.insert(Tkinter.END, str(self.behcount) + ". " + name)
            self.behcount = self.behcount + 1
        elif line[0] == "#":        
            # is a reading
            self.outBox.insert(Tkinter.END,self.lastCmd+": "+line[1:])
            self.numout = self.numout + 1
            if self.numout > 30:
                 self.outBox.delete(0)
                 self.numout = self.numout -1
            self.outBox.see(self.numout-1)    # scroll to end...   

    
###############################################################################
# Implements a serial port handler.  
class ConsoleSerial:
    def __init__(self, parent):
        self.ser = serial.Serial()         
        self.exitcond = False
        self.parent = parent
        self.dialog = None
        
    def open(self, baudrate, port):
        try:
            self.ser.baudrate = baudrate
            self.ser.port = port
            self.ser.timeout = 3
            self.ser.open()
            print port + " opened."
        except:
            print "Cannot open port"
            sys.exit(0) 

    def write(self, text):
        self.ser.write(text)

    def thread(self):
        """ this thread handles the serial comms """    
        while not self.exitcond:
            while self.ser.inWaiting() > 0:
                # take line from serial port (timeout=1sec, mainly for \r problem)
                s = self.ser.readline()   
                # TODO: strip \n and \r from line????
                # send data to a dialog, if one is active             
                if self.dialog != None:
                    self.dialog.serialMsg(s)
                # send messages, readings, etc to console
                if s[0] == "#" or s[0] == "?":
                    self.parent.serialMsg(s)
            # anything else we want to do?
            time.sleep(0.1)


###############################################################################
# a stock dialog that we can extend and use for all sorts of things
class Dialog(Tkinter.Toplevel):
    def __init__(self, parent, title = None):
        Tkinter.Toplevel.__init__(self,parent)
        self.transient(parent)
        self.resizable(0,0)
        self.config(bg = "#BBC6A1")         # make window nice green color
        
        if title:
            self.title(title)
        self.parent = parent
        self.result = None
        
        body = Tkinter.Frame(self, bg = "#BBC6A1")
        self.initial_focus = self.body(body)
        body.pack(padx=5, pady=5)
        
        self.buttonbox()
        self.grab_set()
        if not self.initial_focus:
            self.initial_focus = self

        self.protocol("WM_DELETE_WINDOW",self.cancel)
        self.geometry("+%d+%d" % (parent.winfo_rootx() + 50, parent.winfo_rooty() + 50))

        self.parent.ser.dialog = self

        self.initial_focus.focus_set()
        self.wait_window(self)

    # create dialog body, override this for your uses 
    def body(self, master): 
        pass
    
    # draw buttons, override this for your uses
    def buttonbox(self):
        box = Tkinter.Frame(self)
        Tkinter.Button(box, text="OK", width=10, command=self.ok, default=ACTIVE).pack(side=Tkinter.LEFT,padx=5, pady=5)
        Tkinter.Button(box, text="Cancel",width=10,command=self.cancel).pack(side=LEFT,padx=5, pady=5)
        self.bind("&lt;Return>",self.ok)
        self.bind("&lt;Escape>",self.cancel)
        box.pack()
    
    def ok(self, event=None):
        self.withdraw()
        self.update_idletasks()
        self.cancel()

    def cancel(self, event=None):
        self.parent.ser.dialog = None
        self.parent.focus_set()
        self.destroy()

    # process a line of serial data
    def serialMsg(self, line):
        pass


###############################################################################
# dialog to configure which robot we want to communicate to
class SettingsDialog(Dialog):
    def __init__(self,parent,title=None):
        Dialog.__init__(self,parent,"Settings")

    # create listbox of robots to select
    def body(self, master):
        self.robots = Tkinter.Listbox(master,width=25)
        self.robots.grid(row=0,column=1,columnspan=2)        
        self.robots.bind("<ButtonRelease-1>", self.UpdateDialog)
        # get settings from configuration file
        # config file format:
        # line 1:   /dev/ttyUSB0
        # line > 1: Robot Name,BaudRate,XBEE ID
        try:
            confFile = open('xr.conf', 'r').readlines()
        except:
            print "Unable to open xr.conf"
            self.cancel()
        # read in data, strip off \n's
        self.ROBOTS = []
        for line in confFile[1:]:
            if line.find("#") == -1 and line.find("ADDIN:") != 0:
                self.ROBOTS.append(line[0:-1].split(","))
        for robot in self.ROBOTS:
            self.robots.insert(Tkinter.END, robot[0]) 
        self.robotlabel = Tkinter.Label(master, text="", bg = "#BBC6A1")
        self.robotlabel.grid(row=1,column=1,sticky=Tkinter.W)
        self.robots.bind("<ButtonRelease-1>", self.UpdateDialog)
        Tkinter.Button(master, text="OK", command=self.FinishDialog).grid(row=1,column=2,sticky=Tkinter.E,pady=5)

    def buttonbox(self):
        pass

    # when a new robot is selected, update our status bar       
    def UpdateDialog(self, event=None):
        sel = self.robots.get(self.robots.curselection()[0])
        for robot in self.ROBOTS:
            if robot[0] == sel:
                newlabel = robot[0] + ", ADDR: " + robot[2]
                self.robotlabel.config(text=newlabel)
        
    # on OK, if a new robot is selected, update our settings   
    def FinishDialog(self):
        if self.robots.curselection() != tuple() and self.robots.get(self.robots.curselection()[0]) != self.parent.ROBOT_NAME:
            sel = self.robots.get(self.robots.curselection()[0])        
            # get ADDR & serial
            newaddr = ""   
            newser = ""
            for robot in self.ROBOTS:
                if robot[0] == sel:
                    newaddr = robot[2]
                    newser = robot[1]
            print('Settings: new robot selected ' + sel)
            # set XBEE to new channel - ser.write()
            print('Entering AT mode')
            self.robotlabel.config(text="Writing to Xbee")
            self.parent.ser.write('+++')
            time.sleep(2)                
            self.parent.ser.write('ATDL' + newaddr + '\r')
            time.sleep(0.1)
            self.parent.ser.write('ATWR\r')
            time.sleep(2)
            self.parent.title('Console Beta2: ' + sel)
            # edit configuration file
            self.robotlabel.config(text="Writing xr.conf")
            try:
                confInfo = open('xr.conf', 'r').readlines()
                #time.sleep(1)
                confFile = open('xr.conf', 'w')
                for line in confInfo:
                    if line.find(sel) == 0:
                        pass # nothing to print here
                    elif line.find(self.parent.ROBOT_NAME) == 0:
                        print>>confFile, sel + ',' + newser + ',' + newaddr
                        print>>confFile, line[0:-1]
                    else:
                        print>>confFile, line[0:-1] 
                print "Wrote xr.conf"
            except:
                print "Unable to open xr.conf" 
            self.parent.ROBOT_NAME = sel           
        self.cancel()        

    # process a line of serial data
    def serialMsg(self, line):
        print "Settings: " + line

    def cancel(self, event=None):
        Dialog.cancel(self)
        print "Exiting AT mode"


###############################################################################
# dialog to select add-in to load
class AddinsDialog(Dialog):
    def __init__(self,parent,title=None):
        Dialog.__init__(self,parent,"Add-ins")
        self.parent = parent

    # create listbox of addins to select
    def body(self, master):
        self.addins = Tkinter.Listbox(master,width=25)
        self.addins.grid(row=0,column=1,columnspan=2)        
        self.addins.bind("<ButtonRelease-1>", self.updateDialog)
        for addin in self.parent.ADDINS:
            self.addins.insert(Tkinter.END, addin[0])
        self.addinlabel = Tkinter.Label(master, text="", bg = "#BBC6A1")
        self.addinlabel.grid(row=1,column=1,sticky=Tkinter.W)
        Tkinter.Button(master, text="OK", command=self.finishDialog).grid(row=1,column=2,sticky=Tkinter.E,pady=5)

    def buttonbox(self):
        pass

    # when a new addin is selected, update our status bar       
    def updateDialog(self, event=None):
        sel = self.addins.get(self.addins.curselection()[0])
        for addin in self.parent.ADDINS:
            if addin[0] == sel:
                newlabel = addin[1]
                self.addinlabel.config(text=newlabel)
        
    # on OK, load the addin   
    def finishDialog(self):
        if self.addins.curselection() != tuple():
            sel = self.addins.get(self.addins.curselection()[0])                    
            self.cancel()    
            loadDialog(sel,self.parent)        
        else:
            self.cancel()

# this will dynamically load a new module
def loadDialog(modulename, parent):
    module = __import__(modulename, globals(), locals(), ["interface"])
    dialogClass = getattr(module, "interface") 
    dialog = dialogClass(parent) 

if __name__ == "__main__":
    app = Console()
