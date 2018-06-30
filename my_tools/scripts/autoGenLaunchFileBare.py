#!/usr/bin/env python
# Patrik Kolaric (patrikkolaric92@gmail.com)
import os
import sys

class LaunchElem():
    _Yelem = ''
    _head = ''
    _body = ''
    _outputText = ''

    def __init__(self, elem, head, body):
        self._elem = elem
        self._head = head
        self._body = body

        self.createString()

    def getString(self):
        return self._outputText
        
    def setBody(self, newBody):
        self._body = newBody

    def setHead(self, newHead):
        self._head = newHead

    def createString(self):
        text = [] #first clear existing string 

        if not self._body[0]:
            text.append('<' + self._elem + ' ' + self._head + '/>\n')
        else:
            text.append('<' + self._elem + ' ' + self._head + '>\n')
            for el in self._body:
                text.append('  ' + el)            
            text.append('</' + self._elem + '>\n')

        self._outputText = text

def addCFagent(ID):
    arg = LaunchElem('arg', 'name="frame" value="$(arg frame' + str(ID) + ')"', [''])
    ctrl = LaunchElem('include', 'file="$(find crazyflie_controller)/launch/crazyflie2.launch"', arg.getString())
    argLE1 = LaunchElem('arg', 'name="uri" value="$(arg uri'+str(ID)+')"', [''])
    argLE2 = LaunchElem('arg', 'name="tf_prefix" value="crazyflie'+str(ID)+'"', [''])
    argLE3 = LaunchElem('arg', 'name="enable_logging" value="False"', [''])
    args = argLE1.getString() + argLE2.getString() + argLE3.getString()
    driver = LaunchElem('include', 'file="$(find crazyflie_driver)/launch/crazyflie_add.launch"', args)    
    par1 = LaunchElem('param', 'name="use_crazyflie_controller" value="True"', [''])
    par2 = LaunchElem('param', 'name="joy_topic" value="/joy"', [''])
    pars = par1.getString() + par2.getString()
    joyNode = LaunchElem('node', 'name="joystick_controller" pkg="crazyflie_demo" type="controller.py" output="screen"', pars)
    #CFbody = ctrl.getString() + driver.getString() + joyNode.getString()
    CFbody = ctrl.getString() + joyNode.getString()
    CF = LaunchElem('group', 'ns="crazyflie' + str(ID) + '"', CFbody)
    return CF

if __name__ == '__main__':

    path = os.path.abspath("")
    if "CWS" in path:
        while os.path.basename(path) != "CWS" and os.path.basename(path) != "home":
            path, tail = os.path.split(path)
        path = os.path.join(path, "src")
        path2script = os.path.join(path, "my_tools/scripts")
        path2config = os.path.join(path, "my_tools/config")
        path2launch = os.path.join(path, "crazyflie_demo/launch")
    else:
        sys.exit("ERROR: .launch autogenerating script called outside CWS project")


    file = open(os.path.join(path2config, "ConfigFile.txt"), 'r')
    lines = file.readlines()
    file.close()

    launchBody = []

    joyArg = LaunchElem('arg', 'name="joy_dev" default="/dev/input/js0"', [''])
    world = LaunchElem('arg', 'name="worldFrame" default="world"', [''])
    inclServ = LaunchElem('include', 'file="$(find crazyflie_driver)/launch/crazyflie_server.launch"', [''])
    param = LaunchElem('param', 'name="dev" value="$(arg joy_dev)"', [''])
    nodeJoy = LaunchElem('node', 'name="joy" pkg="joy" type="joy_node" output="screen"', param.getString())
    launchBody = launchBody+['\n']+joyArg.getString()+['\n']+world.getString()+['\n']+inclServ.getString()+['\n']+nodeJoy.getString()

    numCF = 0
    CFdata = []
    for line in lines:
        numCF = numCF + 1
        CFfr    = LaunchElem('arg', 'name="frame' + str(numCF) + '" default="/vicon/crazyflie' + str(numCF) + '/crazyflie' + str(numCF) + '"', [''])
        CFuri   = LaunchElem('arg', 'name="uri'   + str(numCF) + '" default="' + line[:-1] + '"', [''])
        CF = addCFagent(numCF)
        CFdata = CFdata+['\n\n']+CFfr.getString()+CFuri.getString()+['\n']+CF.getString()

    launchBody = launchBody+CFdata

    arg = LaunchElem('arg', 'name="worldFrame" value="$(arg worldFrame)"', [''])
    inclLE = LaunchElem('include', 'file="$(find crazyflie_coop)/launch/crazyflie_coop.launch"', arg.getString())
    bag = LaunchElem('node', 'pkg="rosbag" type="record" name="crazy_bag" args="-a"', [''])
    vicon = LaunchElem('include', 'file="$(find vicon_bridge)/launch/vicon.launch"', [''])
    launchBody = launchBody+inclLE.getString()+['\n']+bag.getString()+['\n']+vicon.getString()

    finalElem = LaunchElem('launch', '', launchBody)

    file = open(os.path.join(path2launch, "Testing.launch"), 'w')
    file.write('<?xml version="1.0"?>\n\n') # initial line in launch file
    file.writelines(finalElem.getString())
    file.close()

    sys.exit(".launch file successfully generated")
