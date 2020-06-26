import threading
import sys
import socket
import struct
import errno
import time

interface = 'can0'

class RecvThread(threading.Thread):
  def __init__(self):
      threading.Thread.__init__(self)
      print('Recv thread init')
  def run(self):
      print('Recv thread start')
      listen_cmd(interface)
      


class CANSocket(object):
  FORMAT = "<IB3x8s"
  FD_FORMAT = "<IB3x64s"
  CAN_RAW_FD_FRAMES = 5

  def __init__(self, interface=None):
    self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    if interface is not None:
      self.bind(interface)

  def bind(self, interface):
    self.sock.bind((interface,))
    self.sock.setsockopt(socket.SOL_CAN_RAW, self.CAN_RAW_FD_FRAMES, 1)

  def send(self, cob_id, data, flags=0):
    cob_id = cob_id | flags
    can_pkt = struct.pack(self.FORMAT, cob_id, len(data), data)
    self.sock.send(can_pkt)

  def recv(self, flags=0):
    can_pkt = self.sock.recv(72)

    if len(can_pkt) == 16:
      cob_id, length, data = struct.unpack(self.FORMAT, can_pkt)
    else:
      cob_id, length, data = struct.unpack(self.FD_FORMAT, can_pkt)

    cob_id &= socket.CAN_EFF_MASK
    return (cob_id, data[:length])


def format_data(data):
    return ''.join([hex(byte)[2:] for byte in data])


def generate_bytes(hex_string):
    if len(hex_string) % 2 != 0:
      hex_string = "0" + hex_string

    int_array = []
    for i in range(0, len(hex_string), 2):
        int_array.append(int(hex_string[i:i+2], 16))

    return bytes(int_array)


def send_cmd(interface, cob_id, body, extended_id):
    try:
      s = CANSocket(interface)
    except OSError as e:
      sys.stderr.write('Could not send on interface {0}\n'.format(interface))
      sys.exit(e.errno)

    try:
      cob_id = int(cob_id, 16)
    except ValueError:
      sys.stderr.write('Invalid cob-id {0}\n'.format(cob_id))
      sys.exit(errno.EINVAL)

    s.send(cob_id, generate_bytes(body), socket.CAN_EFF_FLAG if extended_id else 0)


def listen_cmd(interface):
    try:
      s = CANSocket(interface)
    except OSError as e:
      sys.stderr.write('Could not listen on interface {0}\n'.format(interface))
      sys.exit(e.errno)

    print('Listening on {0}'.format(interface))

    while True:
        cob_id, data = s.recv()
        print("cob_id: "+str(cob_id))
        print('%s %03x#%s' % (interface, cob_id, format_data(data)))
#        print('data: '+str(data))
#        print('format_data: '+str(format_data(data))[0])
        print('\n')

recvThread = RecvThread()
recvThread.start()

if __name__ == '__main__':
    cob_id = 100
    while(True):
        cob_id += 1
        body = '10001'+str(cob_id)[-3:]
        extended_id = False
        send_cmd(interface, str(cob_id)[-3:], body, extended_id)
        time.sleep(5)
