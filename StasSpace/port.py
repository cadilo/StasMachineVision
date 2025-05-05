import sys

from serial import Serial 
from serial.serialutil import SerialException 
from time import sleep

class State(Exception):
    operation = ''

    def __repr__(self):
        try:
            return f'State(lastcmd="{self.lastcmd}", exception="{self.exception}")'
        except:
            return f'State(lastcmd="{self.lastcmd}")'

class Ok(State):
    def __init__(self, lastcmd):
        self.operation = lastcmd

class Error(State):
    def __init__(self, lastcmd, exception):
        self.operation = lastcmd
        self.exception = exception

class Port(Serial):

    def __init__(self, port="/dev/ttyUSB0", outfile=sys.stderr, queue=None):
        self.__oufile = outfile
        self.__queue = queue
        self.__work_w_port = 0
        try:
            if port is not None:
                super().__init__(port, baudrate=9600)
                self.__work_w_port = 1
                sleep(1)
                print("Порт открыт!")                
            self.state = State()
        except SerialException as e:
            self.state = Error('', e)
            self.__log(self.state, file=self.__oufile, queue=self.__queue)
            print("-")
            raise e

    def __log(self, data, file=None, queue=None):
        if file is not None: 
            print(data, file=file)
        if queue is not None: 
            queue.put(data)
        if queue is None and file is None:
            print(data, file=sys.stderr)

    def G00(self, j1=0, j2=0, j3=0, j4=0, j5=0, j6=0):
        try:
            cmd = f'G00 {j1} {j2} {j3} {j4} {j5} {j6}'
            self.__log(f'>> {cmd.strip()}', file=self.__oufile, queue=self.__queue)
            if self.__work_w_port: self.write(bytes(cmd,'ascii'))
         
            self.state = Ok(cmd)
            self.state_pos = self.readline().decode('ascii').strip()
            if self.state_pos == "position complete":
                #time.sleep(1)
                print(self.state_pos)
            else:
                print(self.state_pos)
        except Exception as e:
            self.state = Error(cmd, e)

    def G01(self):
        try:
            cmd = f'G01'
            self.__log(f'>> {cmd.strip()}', file=self.__oufile, queue=self.__queue)
            #print(bytes(cmd,'ascii'))
            if self.__work_w_port: self.write(bytes(cmd,'ascii'))
            #print(self.readline().decode('ascii').strip())             
            #self.state = Ok(cmd)
            return self.readline().decode('ascii').strip()
          
        except Exception as e:
            self.state = Error(cmd, e)

    def G05(self, param):
        try:
            cmd = f'G05 {param}'
            self.write(bytes(cmd, 'ascii'))

        except Exception as e:
            self.state = Error(cmd, e)

    def set_speed(self, vp=50):
        try:
            cmd = f'G07 VP={vp}\r\n'
            self.__log(f'>> {cmd.strip()}', file=self.__oufile, queue=self.__queue)
            if self.__work_w_port: self.write(bytes(cmd,'ascii'))
            self.state = Ok(cmd)
            assert self.is_ready()
        except Exception as e:
            self.state = Error(cmd, e)

    def is_ready(self, timeout=1000):
        if self.__work_w_port: 
            try:
                n = 0
                while 1:
                    n += 1
                    assert n < timeout, f'The timeout of {timeout / 1000} seconds was exceeded when performing operation {self.state.operation}'
                    x = self.readline().decode('ascii').strip()
                    if x: self.__log(f'<< {x}', file=self.__oufile, queue=self.__queue)
                    if '%' in x:
                        break
                    sleep(0.001)
            except Exception as e:
                self.state = Error(self.state.operation, e)
                raise self.state
        else:
            self.__log('<< emulated!', file=self.__oufile, queue=self.__queue)

if __name__ == '__main__':
    from queue import Queue
    q = Queue()
    p = Port(port="/dev/ttyUSB0", outfile=open('/tmp/log','w'), queue=q)
    # sleep(1)
    # p.G00(0, 0, 0, 50, 0, 0)
    # print("1 - +")
    # sleep(5)
    # k=p.G01()
    # print(k)
    # sleep(2)
    # p.G00(0, 0, 0, 100, 0, 0)

    # k=p.G01()
    # print(k)
    # sleep(2)
    # p.G00(0, 0, 0, 150, 0, 0)

    # k=p.G01()
    # print(k)
    # sleep(2)
    # p.G00(0, 0, 0, 200, 0, 0)

    # sleep(2)
    # p.G00(0, 0, 0, 0, 0, 0)
    # print("2 - +")
    p.G05(1)
    sleep(3)
    p.G05(0)