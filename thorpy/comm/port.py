import serial
import select
import threading
import time
import queue
import weakref
import thorpy

class Port:
    #List to make "quasi-singletons"
    static_port_list = weakref.WeakValueDictionary()
    static_port_list_lock = threading.RLock()
    
    def __init__(self, port, sn=None):
        super().__init__()
        self._lock = threading.RLock()
        self._lock.acquire()
        self._buffer = b''
        self._unhandled_messages = queue.Queue()

        self._serial = serial.Serial(port,
                                     baudrate=115200,
                                     bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     #write_timeout = 0.1,#rajout
                                     
                                     rtscts=True)

        # The Thorlabs protocol description recommends toggeling the RTS pin and resetting the
        # input and output buffer. This makes sense, since the internal controller of the Thorlabs
        # device does not know what data has reached us of the FTDI RS232 converter.
        # Similarly, we do not know the state of the controller input buffer.
        # Be toggling the RTS pin, we let the controller know that it should flush its caches.
        self._serial.setRTS(0)
        time.sleep(0.05)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._serial.setRTS(1)
        time.sleep(0.05)
							  

        self._port = port
        self._debug = True
		
        from ..message import MGMSG_HW_NO_FLASH_PROGRAMMING, MGMSG_HW_REQ_INFO, MGMSG_HW_START_UPDATEMSGS, MGMSG_HW_STOP_UPDATEMSGS
        self.send_message(MGMSG_HW_NO_FLASH_PROGRAMMING(source = 0x01, dest = 0x50))

        # Now that the input buffer of the device is flushed, we can tell it to stop reporting updates and
        # then flush away any remaining messages.
        self.send_message(MGMSG_HW_STOP_UPDATEMSGS())
        time.sleep(0.5)
        self._serial.reset_input_buffer()

        self._info_message = None
        while self._info_message is None:
            self.send_message(MGMSG_HW_REQ_INFO())
            try:
                # print("while : try")
                message = self._recv_message(blocking = True)
                if isinstance(message,thorpy.message.systemcontrol.MGMSG_HW_GET_INFO) :
                    self._info_message = message
            except: # TODO: Be more specific on what we catch here
                print ("while : except")
                self._buffer = b''
                self._serial.flushInput()

        if sn is None:
            sn = self._info_message['serial_number']

        self._serial_number = int(sn)
        time.sleep(1)
        print("Port : send message 3")    
        
        #statut chaque 100ms :
        #self.send_message(MGMSG_HW_START_UPDATEMSGS(update_rate = 1))
            
        self._stages = weakref.WeakValueDictionary()
        
        self._lock.release()
        self.daemon = False
        # self.daemon = True
        print("Constructed: {0!r}".format(self))
        
        self._thread_main = threading.current_thread()
        self._thread_worker_initialized = threading.Event()
        self._thread_worker = threading.Thread(target = Port.run, args = (weakref.proxy(self), ))
        self._thread_worker.daemon = True
        self._thread_worker.start()
        
        # For USB devices, we need to keep the port alivew ith a ACK_DCSTATUSUPDATE message.
        if 1:#if "USB" in port:
            self._thread_worker2_initialized = threading.Event()
            self._thread_worker2 = threading.Thread(target = Port.keep_alive, args = (weakref.proxy(self), ))
            self._thread_worker2.daemon = True
            self._thread_worker2.start()


        self._thread_worker_initialized.wait()
        print("Port : fin init")
        

    def __del__(self):
        self._thread_worker2.join() 
        self._thread_worker.join()
        print("Destructed: {0!r}".format(self))


            
    def send_message(self, msg):
        with self._lock:
            if self._debug:
                print('> ', msg)
            self._serial.write(bytes(msg))
            
    @staticmethod
    def run(self):
        print("THREAD RUN") 
        try:
            self._continue = True
            timeout = 1
            self._thread_worker_initialized.set()
            while self._thread_main.is_alive():
                #Trick to avoid holding lock
                r, w, e = select.select([self._serial], [], [], timeout)
                msg = self._recv_message(False)
                if msg is not None:
                    message_handled = self._handle_message(msg)
                    if not message_handled:
                        print("Unhandled message", msg)
                        self._unhandled_messages.put(msg)
                        
            self._serial.close()
        except ReferenceError:
            print("EXCEPT : STOP RUN")
            pass  #Object deleted

    @staticmethod
    def keep_alive(self):
        from ..message import MGMSG_MOT_REQ_STATUSUPDATE
        print("KEEP ALIVE RUN") 
        while self._thread_main.is_alive():
            time.sleep(0.5)
            self.send_message(MGMSG_MOT_REQ_STATUSUPDATE(chan_ident=1))

    def _recv(self, l = 1, blocking = False):
        with self._lock:
            if not blocking:
                r, w, e = select.select([self._serial], [], [], 0)
                if len(r) == 0:
                    return 0
                
            new_data = self._serial.read(l)
            self._buffer += new_data
            return len(new_data)
        
        
    def fileno(self):
        with self._lock:
            return self._serial.fileno()
        
    def recv_message(self, block = True, timeout = None):
        try:    
            print("recv_message : try ")
            return self._unhandled_messages.get(block, timeout)
        except queue.Empty:
            print("recv_message except")
            return None
    
    def _recv_message(self, blocking = False, timeout = None):
        with self._lock:
            from ..message import Message, IncompleteMessageException
            msg = None
            start_time = time.time()
            while msg is None:
                try:
                    msg = Message.parse(self._buffer)
                except IncompleteMessageException:
                    # print("_recv_message : IncompleteExcept")
                    msg = None
                    length = self._recv(blocking = blocking)
                    
                    #We were not able to read data
                    if length == 0 and not blocking:
                        return None
                    
                    #Passed timeout...
                    if blocking and timeout is not None and timeout < time.time() - start_time:
                        print("timeout ",timeout,time.time() - start_time)
                        return None
                    
            
            self._buffer = self._buffer[len(msg):]
            
            if self._debug:
                print('< ', msg)
            return msg
        
    @property
    def serial_number(self):
        return self._serial_number
    
    @property
    def channel_count(self):
        #_info_message is immutable, no worries about lock
        return self._info_message['nchs']
    
    def _handle_message(self, msg):
        return False
    
    def __repr__(self):
        return '{0}({1!r},{2!r})'.format(self.__class__.__name__, self._port, self._serial_number)
    
    def get_stages(self, only_chan_idents = None):
        return {}
    
    @classmethod
    def create(cls, port, sn=None):
        print("Create: port, sn : ", port, sn)
        print("Port.static_port_list : ",list(Port.static_port_list))
        # print("Port.static_port_list[port] : ",Port.static_port_list[port])
        with Port.static_port_list_lock:
            print("Create : with")
            try:

                return Port.static_port_list[port]
            except KeyError as e:
                print("Create : Keyerror  : ", str(e))
                #Do we have a BSC103 or BBD10x? These are card slot controllers
                if sn != None and sn[:2] in ('70', '73', '94'):
                    print("Create : CardSlotPort")
                    p = CardSlotPort(port, sn)
                    
                else:
                    print("Create : SingleControllerPort")
                    p = SingleControllerPort(port, sn)
                    
                    print("Create : fin create")
                Port.static_port_list[port] = p
            
                return p


class CardSlotPort(Port):
    def __init__(self, port, sn = None):
        raise NotImplementedError("Card slot ports are not supported yet")

class SingleControllerPort(Port):
    def __init__(self, port, sn = None):
        print("SingleControllerPort init, port, sn :", port , sn)
        super().__init__(port, sn)
        
        if self.channel_count != 1:
            raise NotImplementedError("Multiple channel devices are not supported yet")

    
    def send_message(self, msg):
        msg.source = 0x01
        msg.dest = 0x50
        super().send_message(msg)
        
    def _recv_message(self, blocking = False):
        msg = super()._recv_message(blocking)
        if msg is None:
            return msg
        
        #assert msg.source == 0x50
        #assert msg.dest == 0x01
        return msg

    def _handle_message(self, msg):
        #Is it a channel message? In that case the stage object has to handle it
        if 'chan_ident' in msg:
            try:
                return self._stages[msg['chan_ident']]._handle_message(msg)
            except KeyError:
                #Keep messages to stages that don't exist
                return False
        
        #This is a system message, handle it ourselves
        
        #Not handled
        return False
    
    def get_stages(self, only_chan_idents = None):
        from thorpy.stages import stage_name_from_get_hw_info, GenericStage
        if only_chan_idents is None:
            only_chan_idents = [0x01]
            
        assert len(only_chan_idents) <= 1 
        assert all(x == 1 for x in only_chan_idents)
            
        ret = dict([(k, self._stages.get(k, None)) for k in only_chan_idents])
        for k in only_chan_idents:
            if ret[k] is None:
                ret[k] = GenericStage(self, 0x01, stage_name_from_get_hw_info(self._info_message))
                self._stages[k] = ret[k]
                
        return ret
