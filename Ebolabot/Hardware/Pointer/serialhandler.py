from struct import pack, unpack
from time import time

class SerialReqReplyHandler():
    """
    Implementation of request-reply serial communication for sending up-to-date
    status messaged to other devices. Specifically designed for communicating with
    an Arduino device using corresponding code.

    All input and output is done through modifying or reading the dataIn and dataOut
    arrays, which are filled in and written out to the other device unpon the call of
    certain functions.

    Usage:
        Setup:
            1. Create a SerialReqReplyHandler object, passing it an intialized serial object and specifying the number of bytes needed for each input and output data variable
            2. Create a data-in processing function that accepts a SerialReqReplyHandler object, reads from dataIn, and executes on it as desired (eg to run commands, update variables, etc)
        In main loop:
            1. Modify dataOut to contain the data you would like to write to the other device
            2. Call handleSerialAndPerformOn(), which will read in data, write out data, hadle all the protocol functions, and run your data-in processing function upon the specified condition
    """
    # Message identifiers
    DATA_REQUEST = 'r'
    DATA_MSG_START = 'd'
    
    def __init__(self, ser, outByteLens, inByteLens, requestWaitTimeout=1000):
        """ 
        Args:
            ser - a serial object to send data over
            outByteLens - list containing the number of bytes each data-out value should be sent as.
                i.e. outByteLens[i] contains the number of bytes to write dataOut[i] as.
            inByteLens - list containing the number of bytes each data-in value is expected to be recieved as.
                i.e. inByteLens[i] contains the number of bytes to read into dataIn[i].
            requestWaitTimeout - minimum milliseconds to wait for new data reply. If this time elapses, a data request will be sent again.
        """
        self.ser = ser
        
        # Plublic variables
        self.dataOut = [None]*len(outByteLens)   # data to send the other device upon its request (int list)
        self.dataIn = [None]*len(inByteLens)     # complete, up-to-date data recieved from the other device (int list)
        self.outByteLens = outByteLens           # the number of bytes each data-out value should be sent as (int list). i.e. outByteLens[i] contains the number of bytes to write dataOut[i] as.
        self.inByteLens = inByteLens             # the number of bytes each data-in value is expected to be recieved as (int list). i.e. inByteLens[i] contains the number of bytes to read into dataIn[i].

        # Private variables
        self._readyForData = True                # if True, handler will request more data on the next call of _handleSerial(). Turned True after handling dataIn.
        self._newDataIn = False                  # True when _handleSerial() finishes collecting a data message. Turned False after handling dataIn.
        self._dataInContentNew = True            # True if the values in dataIn are different than those in the last message
        self._totInBytes = sum(inByteLens)       # total bytes per serial message in. Note: technically, inBytesLen+1 bytes will be read over serial per message sent from the other device. The first byte is the data identifier, which we never save into _bytesIn, and thus we do not account for it here.
        self._bytesIn = [None]*self._totInBytes  # current input message being collected (char list)
        self._collectingData = False
        self._collectionIndx = 0
        self._lastdataIn = [-1]*len(outByteLens)
        self._RequestWaitTimer = NonBlockTimer(requestWaitTimeout)
        
        self._RequestWaitTimer.update()
    
    def handleSerialAndPerformOn(self, performOn, func):
        """
        Handles all aspects of serial by sending out the data from dataOut, reading in data to dataIn, and 
        replaying to data requests from the other device. This function should be called frequently.
        
        Args:
            performOn - a control string that dictates when func should be run. It can be set to the below:
                'new data-in content' to run func when values in dataIn are different than those in the last message (thus ignoring repeated messages)
                'new data-in message' to run func when any new message is read in
            func - a function that accepts a SerialReqReplyHandler object that processes dataIn
        
        Returns:
            List of booleans [dataSent, recvdMsg]. dataSent is true if data was sent out. newDataIn is true if
            a complete message was read.
        """
        [dataSent, recvdMsg] = self._handleSerial()
        if self._newDataIn:
            if performOn == 'new data-in content':
                if self._dataInContentNew:
                    func(self)
            elif performOn == 'new data-in message':
                func(self)
            self.doneWithDataIn()
        return [dataSent, recvdMsg]
    
    def _handleSerial(self):
        """ Handles ther request-reply protocol over serial and keeps the SerialReqReplyHandler object up to date.
        Replies to data requests, sends data requests, collects incoming bytes, and updates dataIn on the completion of a message.
        
        Returns:
            List of booleans [dataSent, recvdMsg]. dataSent is true if data was sent out. newDataIn is true if
            a complete message was read.
        """
        dataSent = False
        if self._readyForData or self._RequestWaitTimer.hasElapsed():
            self.ser.write(SerialReqReplyHandler.DATA_REQUEST)
            self._RequestWaitTimer.update()
            self._readyForData = False
        while self.ser.inWaiting():
            rb = self.ser.read()
            if not self._collectingData:
                if (rb == SerialReqReplyHandler.DATA_REQUEST):
                    self._sendOutData()
                    dataSent = True
                    continue
                elif (rb == SerialReqReplyHandler.DATA_MSG_START):
                    self._collectingData = True
            else:
                if self._collectionIndx < self._totInBytes-1:
                    self._bytesIn[self._collectionIndx] = rb
                    self._collectionIndx += 1
                else:
                    # recieved the final byte
                    self._bytesIn[self._collectionIndx] = rb
                    self._lastdataIn = self.dataIn[:]
                    self._updateDataIn()
                    self._collectingData = False
                    self._collectionIndx = 0
                    self._newDataIn = True
                    self._dataInContentNew = self._lastdataIn != self.dataIn
                    return [dataSent, True]
        return [dataSent, False]
                                      
    def doneWithDataIn(self):
        """Marks current dataIn as being processed into external program and flags handler as being ready for 
        more data from other device, which will be retrieved on the next call of _handleSerial() and overwrite
        dataIn. Should be called after processing and executing on dataIn"""
        self._readyForData = True
        self._newDataIn = False

    def _sendOutData(self):
        """Sends ints from dataOut over serial to other device, according to package conventions"""
        package = SerialReqReplyHandler.DATA_MSG_START + packIntsAsBytes(self.dataOut, self.outByteLens)
        self.ser.write(package)
        return package
        
    def _updateDataIn(self):
        """Converts completed bytesIn to integers and stores them in dataIn"""
        byte_i = 0
        for data_i in range(len(self.dataIn)):
            fieldLen = self.inByteLens[data_i]
            self.dataIn[data_i] = unpackBytesAsInts(self._bytesIn[byte_i : byte_i+fieldLen])
            byte_i += fieldLen
	    
def packIntsAsBytes(vals, fieldLens):
    """
    Creates a continuous sting of byte characters from a list of values and the desired byte lengths for each
    value, properly formatted to be sent over serial. This allows for byte minimum messages. Packing on the bit
    level could reduce message size even further.
    
    Args:
        fieldLens - List where fieldLens[i] contains the field length (in bytes) for vals[i]
        vals - List of positive integers to convert to bytes.
    Returns:
        A string of continuous bytes, where each vals[i] is stored in order, taking up fieldLens[i] each
    """
    bytes = ''
    for i in range(len(vals)):
        bytes += uintToByteString(vals[i],fieldLens[i])
    return bytes

def uintToByteString(val, fieldLen):
    """
    Converts unsigned integer to a string of bytes with fieldLen length.
    
    Args:
        fieldLen - Number of bytes to return val as. Must be  1-4.
        val - Positive integer to convert to bytes.
    Returns:
        String with fieldLen byte characters. Effectively an array of characters or bytes.
        Example: uintToByteString(1, 8) = '\x08'
    """
    if fieldLen > 0 and fieldLen <= 4:
        fullBytes = pack('>I', val)
        numRemoved = len(fullBytes)-fieldLen
        if fullBytes[0:numRemoved] != '\x00'*numRemoved:
            raise ValueError('Cannot fit the value ' + str(val) + ' in ' + str(fieldLen) + ' bytes.')
        return fullBytes[-fieldLen:]
    else:
        raise ValueError("fieldLen of " + str(fieldLen) + " not supported. Must be from 1 to 4.")

def unpackBytesAsInts(byte_chars):
    """
    Unpacks bytes (represented as chars) and returns their equivalant int value.
    Warning: this currently only accepts chars and strings, not ints.
    
    Args:
        byte_chars - a string or a list of chars to interpret as a single int, of length 1 to 4.
    Returns:
        An integer equal to the bytes passed in.
        Example: unpackBytesAsInts('1') = 49
                 unpackBytesAsInts(['a', 'b']) = 24930
    """
    try: 
        byte_chars = ['\x00']*(4-len(byte_chars)) +  byte_chars # expand to 4 bytes so unpack will work universally
        bytes_str = ''.join(byte_chars)  # turns array of chars into string
    except TypeError:
        # may be a string already
        try:
            bytes_str = '\x00'*(4-len(byte_chars)) +  byte_chars
        except:
            raise ValueError("byte_chars should be a list of chars or a string")
    fieldLen = len(bytes_str)
    if fieldLen > 0 and fieldLen <= 4:
        return unpack('>I', bytes_str)[0]
    else:
        raise ValueError("char list of length" + str(fieldLen) + " not supported. Must be from 1 to 4.")

def str2ByteStr(strng):
    """
    Returns a string containing the hexidecimal values of the character in a string
    
    Args:
        strng - a string to convert
    Return:
        A string of the hexidecimal values for each character in strng
        Example: str2ByteStr('abc123') = '\x61\x63\x31\x32\x33'
    """
    return r'\x' + r'\x'.join(x.encode('hex') for x in strng)

class NonBlockTimer():
    def __init__(self, maxMS=None):
        self.maxMS = maxMS
        self.lastMS = None
    def update(self):
        self.lastMS = time()*1000
    def hasElapsed(self):
        """True if it has been maxMS milliseconds since last update()"""
        return (time()*1000 - self.lastMS) > self.maxMS
    def msElapsed(self):
        return time()*1000 - self.lastMS

"""Test Functions"""
def selfTest():
    """
    Checks that if the PPU writes data to itself (circumventing serial), it is parsed correctly back to the
    origonal data. Returns True if the test passes.
    """
    from serial import Serial 
    
    dumser = Serial('/dev/ttyUSB1', 115200, timeout=0)
    ph = SerialReqReplyHandler(dumser,[1,2,1],[1,2,1])
    ph.dataOut = [20,5000,20]
    print 'dataOut = ' +str(ph.dataOut)
    out = list(ph._sendOutData())
    print 'bytes sent: ' +str(out)
    ph._bytesIn = out[1:] # removes identifier, simulating how _handleSerial() never saves it
    print 'bytesIn = ' +str(ph._bytesIn)
    ph._updateDataIn()
    print 'dataIn = ' +str(ph.dataIn)
    return ph.dataOut == ph.dataIn
    
def shiftLeftTest():
    """
    If the Arduino set to echoTest(), Python will send [49,50,51] to it, which will then echo those
    values back - which python will recieve and print. Then python will shift the vales left and repeat.
    """
    from serial import Serial 
    
    ser = Serial('/dev/ttyUSB1', 115200, timeout=0)
    ph = SerialReqReplyHandler(ser,[1,2,1],[1,2,1])
    ph.dataOut = [49,50,51]
    
    def doCommands(handler):
        handler.dataOut = [ph.dataOut[1], ph.dataOut[2], ph.dataOut[0]]
        print handler.dataIn
    
    while(True):
        ph.handleSerialAndPerformOn('new data-in content', doCommands)
       
if __name__ == '__main__':    
   shiftLeftTest()

