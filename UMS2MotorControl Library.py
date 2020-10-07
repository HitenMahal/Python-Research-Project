'''-----------------------------------------------------------------------------
Programmer: Hiten Mahalwar, July 15 2019
Edit2: - August 9, 2019
Function:   A program allowing a user to communicate with the
            Focused Ultrasound Tank through serial communication.
-----------------------------------------------------------------------------'''

import serial
from time import sleep

def initializeMotor(portName):
    '''
    Initializes the motor.

    :param portname: portname that the tank is connected to.
    :returns: Com object for the serial communication.
    :raises Error: Raises Error if the program is unable to communicate with the motor.
    '''

    com = serial.Serial(portName,baudrate=19200,timeout=1)
    initializeCounter = 0
    while initializeCounter < 4:
            send(com,'#1\r\r') # Sending '#1' to the motor as a way to confirm the com is working
            initializeCounter += 1
            echo=listen(com)
            print(echo)
            if echo == '4' or echo == '#4': # The motor replies with either '#4' or '4'
                com.reset_input_buffer()
                com.reset_output_buffer()
                break
            if initializeCounter == 3:
                com.close()
                raise Exception('Error: Please be sure that motors are turned on, connected to {} and restart the program'.format(portName))
            else:
                sleep(0.1)
                com.reset_output_buffer()
                com.reset_input_buffer()
                continue
    send(com,'#1(485)\r\r')
    return com

def initializeMotorSettings(com):
    '''
    Sets the Initial Motor Settings.

    :param com: The serial port com object.
    :returns: None.
    '''

    setDistanceUnits(com)
    setResponseSettings(com)
    setVelocity(com,'15')
    setAcceleration(com,'20')
    setDeceleration(com,'20')
    positionMaintainance(com,'4')
    setInPositionTimer(com,'500')
    enableStopButton(com)
    enableArmEventCode(com)
    motorOn(com)
    sleep(0.1)
    com.reset_input_buffer()
    com.reset_output_buffer()

def send(com, message):
    '''
    A function to send a string command to the motor.

    :param com: The serial port com object.
    :param message: The string that will be sent.
    :returns: None.
    '''
    print(message)
    com.write(bytes(message + '\r', 'utf-8'))


def listen(com):
    '''
    A function to recieve strings sent by the motor.

    :param com: The serial port com object.
    :returns: A string message.
    '''

    echo = com.read_until()
    print(echo)
    echo = echo.decode('utf-8')
    return echo.rstrip('\r\n')

def  mmtostep(xmm,ymm,zmm):
    '''
    Unit Convertor from millimeters into steps, 1mm = 1000steps.

    :param xmm: Millimeters in the x plane to be converted to steps.
    :param ymm: Millimeters in the y plane to be converted to steps.
    :param zmm: Millimeters in the z plane to be converted to steps.
    :returns: Steps in the x, y, and z plane.
    '''

#   ----1 millimeter is equal to 1000 steps---
    xStep = xmm*1000
    yStep = ymm*1000
    zStep = zmm*1000
#   -------Ensure a resolution of 1 step------
    xStep = int(round(xStep))
    yStep = int(round(yStep))
    zStep = int(round(zStep))
    return xStep, yStep, zStep


def steptomm(xStep,yStep,zStep):
    '''
    Unit Convertor from steps to millimeters, 1mm = 1000steps.

    :param xStep: Steps in the x plane to be converted to Millimeters.
    :param yStep: Steps in the y plane to be converted to Millimeters.
    :param zStep: Steps in the z plane to be converted to Millimeters.
    :returns: Millimeters in the x, y, and z plane.
    '''

#   ----1 millimeter is equal to 1000 steps----
    xmm = xStep/1000
    ymm = yStep/1000
    zmm = zStep/1000
#   -------Ensure a resolution of 0.01mm------
    xmm = round(xmm,2)
    ymm = round(ymm,2)
    zmm = round(zmm,2)
    return xmm, ymm, zmm

def relativeMotion(com,xRel,yRel,zRel):
    '''
    Function to move in Relative Motion

    :param com: The serial port com object.
    :param xRel: Millimeters to move in the x plane.
    :param yRel: Millimeters to move in the y plane.
    :param zRel: Millimeters to move in the z plane.
    :returns: None.
    '''

    (xCurrent,yCurrent,zCurrent) = mmtostep(xRel,yRel,zRel)
    move3D(com,xCurrent,yCurrent,zCurrent)

def absoluteMotion(com,xAbs,yAbs,zAbs):
    '''
    Function to move in Absolute Motion.

    :param com: The serial port com object.
    :param xAbs: Millimeters to move in the x plane.
    :param yAbs: Millimeters to move in the y plane.
    :param zAbs: Millimeters to move in the z plane.
    :returns: None.
    '''

    (xStep,yStep,zStep) = mmtostep(xAbs,yAbs,zAbs)
    (xCurrent,yCurrent,zCurrent) = currentPositionInSteps(com)
    xMovement = xStep - xCurrent
    yMovement = yStep - yCurrent
    zMovement = zStep - zCurrent
    move3D(com,xMovement,yMovement,zMovement)


def positionMaintainance(com,stepdeadband):
    '''
    Sets step deadband for the motors.

    :param com: The serial port com object.
    :param stepdeadband: +- steps range as the deadband, can be input as a string or integer.
    :returns: None.
    '''

    #Enable Position Maintainance with 4 step deadband outputs no settle
    # Essentially it gives an accuracy of +- 4 steps of the target location
    send(com,'1POSMAIN1({},3,0)'.format(stepdeadband))
    send(com,'2POSMAIN1({},3,0)'.format(stepdeadband))
    send(com,'3POSMAIN1({},3,0)'.format(stepdeadband))


def enableLimits(com,mask,type1,model):
    '''
    Enable Limits for the motor movements, ie Doesn't allow for a movement to exceed the limit amount of steps,
    This feature is not crucial and not needed for general use.

    :param com: The serial port com object.
    :param mask:  See ViX-IE Guide and go to the 'Limits' section.
    :param type1: See ViX-IE Guide and go to the 'Limits' section.
    :param model: See ViX-IE Guide and go to the 'Limits' section.
    :returns: None.
    '''

    #Defines Limits, see ViX-IE Guide and go to the 'Limits' section
    send(com,'1LIMITS({},{},{},200)'.format(mask,type1,model))
    send(com,'2LIMITS({},{},{},200)'.format(mask,type1,model))
    send(com,'3LIMITS({},{},{},200)'.format(mask,type1,model))


def setInPositionTimer(com,time):
    '''
    Sets how long the motor waits after movement has stopped to take the current position reading (between 1-500ms)

    :param com: The serial port com object.
    :param time: In milliseconds how long the motor waits, between 1-500ms, can be input as a integer or string.
    :returns: None.
    '''

    #Sets how long the motor waits after movement has stopped to take the current position reading (between 1-500ms)
    send(com,'1W(IT,{})'.format(time))
    send(com,'2W(IT,{})'.format(time))
    send(com,'3W(IT,{})'.format(time))


def enableStopButton(com):
    #Enables the stop button
    send(com,'1STOP1(0)')
    send(com,'2STOP1(0)')
    send(com,'3STOP1(0)')


def enableArmEventCode(com):
    #Enables Arm Event Code which runs the startup code for the motor
    send(com,'1ARM1')
    send(com,'2ARM1')
    send(com,'3ARM1')


def motorOn(com):
    #Turns on motor
    send(com,'1ON')
    send(com,'2ON')
    send(com,'3ON')

def motorNumber(axis):
    '''
    Converts an axis into the corresponding motor number that navigates that axis.

    :param axis: Either 'x', 'y', or 'z' as a string.
    :returns: Returns '1', '2', or '3' corresponding to the axis.
    '''

    if axis == 'x':
        motor = '1'
    if axis == 'y':
        motor = '2'
    if axis == 'z':
        motor = '3'
    return motor


def askControllerPosition(com,axis):
    '''
    Function to ask the motors their current position.

    :param com: The serial port com object.
    :param axis: Either 'x','y', or 'z'
    :returns: Returns the coordinates as an Integer.
    :raises CommunicationError: Error communicating with the motor so motors are turned off and connection closed.
    '''

    counter = 0
    send(com,'{}R(PA)'.format(motorNumber(axis)))
    while (1):
        errFound = False
        read = listen(com)
        if not read:
            # The motor is not giving any reply
            errFound = True
            counter += 1
            read = 'nocomms'
            com.reset_output_buffer()
            com.reset_input_buffer()
            portName = com.port
            closeConnection(com)
            com = initializeMotor(portName)
            sleep(0.1)
            initializeMotorSettings(com)
            send(com,'{}R(PA)'.format(motorNumber(axis)))
            if counter >= 3:
                com.reset_input_buffer()
                com.reset_output_buffer()
                closeConnection(com)
                raise Exception('Error: Communication with the Motor through {} is not working, restart the program'.format(portname))

        if errFound == False:
            # The motor sent a reply
            positionRead = listen(com) # Listen again to read motor coordinates, sent as '*number'
            if positionRead[0] == '*':
                positionRead = positionRead.replace('*','') #Remove the * to get just the coordinate
                return int(positionRead)


def currentPositionInSteps(com):
    '''
    Recieves current position in steps from all 3 motors.

    :param com: The serial port com object.
    :returns: Returns the coordinates as an Integer.
    '''

    com.reset_input_buffer()
    com.reset_output_buffer()
#   --------Recieve position as integer---------
    xCurrent = askControllerPosition(com,'x')
    yCurrent = askControllerPosition(com,'y')
    zCurrent = askControllerPosition(com,'z')

    return xCurrent, yCurrent, zCurrent


def currentPosition(com):
    '''
    Function to ask the motors their current position.

    :param com: The serial port com object.
    :returns: Returns the Current Position in Millimeters as an Integer.
    '''

    (xCurrentStep,yCurrentStep,zCurrentStep) = currentPositionInSteps(com)
    (xCurrent,yCurrent,zCurrent) = steptomm(xCurrentStep,yCurrentStep,zCurrentStep)
    return xCurrent, yCurrent, zCurrent


def inPosition(com,axis):
    '''
    Function to check whether a motor is in position or still moving.

    :param com: The serial port com object.
    :param axis: Either 'x','y', or 'z'
    :returns: Returns 1 for motor in position or a 0 for not in position, Returns the 1 or 0 as an Integer.
    :raises CommunicationError: Error communicating with the motor so motors are turned off and connection closed.
    '''

    allInPosition = 1
    while allInPosition != 0:
            commError = False
            send(com,'{}R(IP)'.format(motorNumber(axis))) #Ask for if the motor is in position (1) or still moving(0)
            inPosRead = listen(com)
            if not inPosRead:
                commError = True #The motor is not sending any reply
                raise Exception('Error: Issue with communication with motor, reinitializing motor')
                send(com,'0OFF')
                listen(com)
                send(com,'0Z')
                listen(com)
                com.reset_input_buffer()
                com.reset_output_buffer()
                portName = com.port
                com.close()
                com = initializeMotor(portName)
                sleep(0.1)
                initializeMotorSettings(com)
                if a < 4:
                    a += 1
                    continue
                if a == 4:
                    allInPosition = 0
                    closeConnection(com)
                    raise Exception('Communications Failed: Please Restart Everything')
                    return 0
            if commError == False:
                inPosRead = listen(com)
                if inPosRead[0] == '*':
                    inPosRead = inPosRead.replace('*','')
                    posAtZero = int(inPosRead)
                    allInPosition = 0
                    com.reset_input_buffer()
                    com.reset_output_buffer()
                    return posAtZero


def inPositionCheck(com):
    '''
    Function to check whether all motors are in position or one or more are still moving.

    :param com: The serial port com object.
    :returns: Returns 1 for motors in position or a 0 for not in position, Returns the 1 or 0 as an Integer.
    '''

    allInPosition = 0
    while allInPosition != 1:
        xInPos = inPosition(com,'x')
        yInPos = inPosition(com,'y')
        zInPos = inPosition(com,'z')

        allInPosition=xInPos*yInPos*zInPos
    if allInPosition == 1:
        return allInPosition


def move(com,axis,steps):
    '''
    Function to move a motor a certain distance in steps.

    :param com: The serial port com object.
    :param axis: Either 'x','y', or 'z'
    :param steps: The number of steps that the motor should move, can be input as a string or integer.
    :returns: None.
    '''

    com.reset_input_buffer()
    com.reset_output_buffer()
    if steps != 0:
        #Change the steps into a 6 digit number, from 2000 steps to 002000 steps.
        send(com,'{}D{}'.format(motorNumber(axis),str(steps).zfill(6)))
        listen(com)
        #Send Go Command
        send(com,'{}G'.format(motorNumber(axis)))
        listen(com)





def move3D(com, xStep, yStep, zStep):
    '''
    Function to move in all three planes.

    :param com: The serial port com object.
    :param xSteps: Steps to move in the x plane.
    :param ySteps: Steps to move in the y plane.
    :param zSteps: Steps to move in the z plane.
    :returns: None.
    '''

    move(com,'x',xStep)
    move(com,'y',yStep)
    move(com,'z',zStep)




def movex(com,xmm):
    '''
    Function to move Millimeters in the x plane.

    :param com: The serial port com object.
    :param xmm: Millimeters to move in the x plane, can be input as a string or an integer.
    :returns: None.
    '''

    xmm = int(xmm)
    if xmm != 0:
        (xStep,dummy,dummy2) = mmtostep(xmm,0,0)
        move(com,'x',xStep)


def movey(com,ymm):
    '''
    Function to move Millimeters in the y plane.

    :param com: The serial port com object.
    :param ymm: Millimeters to move in the y plane, can be input as a string or an integer.
    :returns: None.
    '''

    ymm = int(ymm)
    if ymm != 0:
        (dummy,yStep,dummy2) = mmtostep(0,ymm,0)
        move(com,'y',yStep)


def movez(com,zmm):
    '''
    Function to move Millimeters in the z plane.

    :param com: The serial port com object.
    :param zmm: Millimeters to move in the z plane, can be input as a string or an integer.
    :returns: None.
    '''

    zmm = int(zmm)
    if zmm != 0:
        (dummy,dummy2,zStep) = mmtostep(0,0,zmm)
        move(com,'z',zStep)

def setDistanceUnits(com):
    # Sets Distance Units to Encoder Step
    send(com,'1W(DU,1)')
    send(com,'2W(DU,1)')
    send(com,'3W(DU,1)')


def setResponseSettings(com):
    #Sets response settings to "speak when spoken to, echo on"
    send(com,'1W(EX,2)')
    send(com,'2W(EX,2)')
    send(com,'3W(EX,2)')


def setVelocity(com,velocity):
    '''
    Sets the Velocity in revs/s.

    :param com: The serial port com object.
    :param velocity: The velocity for the motors, can be input as either a string or integer.
    :returns: None.
    '''

    #Sets Motor 1 Velocity to 15revs/s when '1V15' is sent
    send(com,'1V{}'.format(velocity))
    send(com,'2V{}'.format(velocity))
    send(com,'3V{}'.format(velocity))


def setAcceleration(com,acceleration):
    '''
    Sets acceleration for the motors in revs/s^2

    :param com: The serial port com object.
    :param acceleration: The acceleration for the motor, can be input as either a string or integer.
    :returns: None.
    '''

    #Sets Motor 1 Acceleration to 15revs/s^2 when '1AA15' is sent
    send(com,'1AA{}'.format(acceleration))
    send(com,'2AA{}'.format(acceleration))
    send(com,'3AA{}'.format(acceleration))


def setDeceleration(com,deceleration):
    '''
    Sets deceleration for the motors in revs/s^2

    :param com: The serial port com object.
    :param deceleration: The deceleration for the motor, can be input as a string or integer.
    :returns: None.
    '''

    #Sets Motor 1 Deceleration to 15revs/s^2 when'1AD15' is sent
    send(com,'1AD{}'.format(deceleration))
    send(com,'2AD{}'.format(deceleration))
    send(com,'3AD{}'.format(deceleration))

def motorOff(com):
    '''
    Turns Off Motors

    :param com: The serial port com object.
    :returns: None.
    '''
    send(com,'0OFF')
    listen(com)

def turnOffMotor(com):
    '''
    Turns Off Motors and then Closes the Serial Port connection.

    :param com: The serial port com object.
    :returns: None.
    '''
    motorOff(com)
    sleep(0.1)
    closeConnection(com)


def closeConnection(com):
    '''
    Closes Serial Port Connection.

    :param com: The serial port com object.
    :returns: None.
    '''

    com.close()


def goToZero(com):
    '''
    Tells all motors to Go To Zero.

    :param com: The serial port com object.
    :returns: None.
    '''
    absoluteMotion(com,0,0,0)

def safeGoToZero(com):
    '''
    Go to Zero function however the hydrophone moves back, goes to x-zero and y-zero, then moves to z-zero.

    :param com: The Serial port com object.
    :returns: None.
    '''
    movez(com,-30)
    sleep(1)
    safetyCheck = inPositionCheck(com)
    if safetyCheck == 1:
        (xCurrent,yCurrent,zCurrent) = currentPosition(com)
        safetyCheck = 0
        absoluteMotion(com,0,0,zCurrent)
        safetyCheck = inPositionCheck(com)
        if safetyCheck == 1:
            absoluteMotion(com,0,0,0)


def setZero(com):
    '''
    Sets current position as the new zero.

    :param com: The serial port com object.
    :returns: None.
    '''

    (xmmCurrent,ymmCurrent,zmmCurrent) = currentPosition(com)
    if xmmCurrent != 0:
        send(com,'1Z')
        listen(com)
    if ymmCurrent != 0:
        send(com,'2Z')
        listen(com)
    if zmmCurrent != 0:
        send(com,'3Z')
        listen(com)
