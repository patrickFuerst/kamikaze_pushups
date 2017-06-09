#!/usr/bin/env python


# The master collects all sensor data.
# Depending on the data we clean/tweak it and make it available over a
# socket interface.

import logging
import sys
import socket
import serial
from serial.threaded import LineReader, ReaderThread
import time
import io
from queue import *
from threading import Thread, Timer
import traceback
import struct
import glob
import math
import numpy

logger = logging.getLogger("SensorMaster")
logger.setLevel(logging.DEBUG)


# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> numpy.allclose(M, numpy.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
    n = numpy.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = numpy.outer(q, q)
    return numpy.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])



def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print(axes, "failed")

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.99810947, 0.06146124, 0, 0])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)



UDP_PORT = 7775
PUB_PORT = 7776

def process_orientation_data(data):

	values = {}

	# orientation data is in binary format
	if not len(data) == 63:
		logger.error("Invalid orientation data")
		return None

	unpacket_data = struct.unpack('=cdddddddb?f', data )
	header = unpacket_data[0]

	if header == b'o':
		values["x"] = unpacket_data[1]
		values["y"] = unpacket_data[2]
		values["z"] = unpacket_data[3]
		values["w"] = unpacket_data[4]
		values["ax"] = unpacket_data[5]
		values["ay"] = unpacket_data[6]
		values["az"] = unpacket_data[7]
		values["system_status"] = unpacket_data[8]
		values["calibrated"] = unpacket_data[9]
		values["battery_voltage"] = unpacket_data[10]
	else:
		logger.error("Invalid orientation data")

	return {"orientation_sensor" : values}

import zmq
class DataDistributer(Thread):


	def __init__(self):
		Thread.__init__(self)
		self.data_queue = Queue()
		self.ctx = zmq.Context()
		self.pub_socket = self.ctx.socket(zmq.PUB)
		self.pub_socket.bind("tcp://*:"+str(PUB_PORT))

	def run(self):
		logger.info("Running Data Distibuter")
		while True:
			datum = self.data_queue.get(block=True)
			print(datum)
			self.pub_socket.send_json(datum)
			if self.data_queue.qsize() > 10:
				logger.warning( "Data Queue size increasing")
			time.sleep(0.01)

	def get_queue(self):
		return self.data_queue

from pythonosc import osc_message_builder
from pythonosc import udp_client
import random

class DataDistributerOSC(Thread):


	def __init__(self):
		Thread.__init__(self)
		self.data_queue = Queue()
		self.osc_client = udp_client.SimpleUDPClient("localhost", 6648)
		self.osc_client_susi = udp_client.SimpleUDPClient("192.168.0.102", 6648)


	def run(self):
		logger.info("Running Data Distibuter OSC")
		print("sending osc")

		while True:
			datum = self.data_queue.get(block=True)
			#print("sending osc")

			msg = osc_message_builder.OscMessageBuilder(address = "/data")
			o_data = datum['orientation_sensor']
			quaternion = [o_data['w'], o_data['x'],o_data['y'],o_data['z']]
			x,y,z = euler_from_quaternion(quaternion)
			ax,ay,az = o_data['ax'],o_data['ay'],o_data['az']
			msg.add_arg(x)
			msg.add_arg(y)
			msg.add_arg(z)
			msg.add_arg(ax)
			msg.add_arg(ay)
			msg.add_arg(az)
			msg = msg.build()
			self.osc_client.send(msg)
			self.osc_client_susi.send(msg)


			msg = osc_message_builder.OscMessageBuilder(address = "/calibrated")
			o_data = datum['orientation_sensor']
			msg.add_arg(o_data["calibrated"])
			msg = msg.build()
			self.osc_client.send(msg)
			self.osc_client_susi.send(msg)

			if self.data_queue.qsize() > 10:
				logger.warning( "Data Queue size increasing")
			#time.sleep(0.01)


	def get_queue(self):
		return self.data_queue



class SocketReaderThread(Thread):

	def __call__(self):
		return self

	def __init__(self, socket, data_queue ):
		super(SocketReaderThread, self).__init__()
		self.socket = socket
		self.data_queue = data_queue



	def run(self):
		logger.info("Run Socket Reader")
		while True:
			data = self.socket.recv(64) # buffer size is a multiple of 2
			processed_data = process_orientation_data(data)
			#print("getdata")
			if processed_data:
				self.data_queue.put(processed_data)

			time.sleep(0.01)


class SerialProcessor(LineReader):

	def __call__(self):
		return self

	def __init__(self, data_queue, processingMethod):
		super(SerialProcessor, self).__init__()
		self.data_queue = data_queue
		self.processingMethod = processingMethod

	def connection_made(self, transport):
		super(SerialProcessor, self).connection_made(transport)
		logger.info('Serial Port opened')

	def handle_line(self, data):
		processed_data = self.processingMethod(data)
		if processed_data:
			self.data_queue.put(processed_data)

	def connection_lost(self, exc):
		if exc:
			traceback.print_exc(exc)
		logger.info('Serial Port closed')

import signal
class GracefulInterruptHandler(object):

    def __init__(self, signals=(signal.SIGINT, signal.SIGTERM)):
        self.signals = signals
        self.original_handlers = {}

    def __enter__(self):
        self.interrupted = False
        self.released = False

        for sig in self.signals:
            self.original_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, self.handler)

        return self

    def handler(self, signum, frame):
        self.release()
        self.interrupted = True

    def __exit__(self, type, value, tb):
        self.release()

    def release(self):
        if self.released:
            return False

        for sig in self.signals:
            signal.signal(sig, self.original_handlers[sig])

        self.released = True
        return True



if __name__ == '__main__':

	logging.basicConfig(level=logging.DEBUG,
			format='[%(name)s]: %(message)s',
			handlers=[logging.StreamHandler()])

	import argparse

	parser = argparse.ArgumentParser(
		description='Simple Serial to Network (TCP/IP) redirector.')

	data_distributer = DataDistributerOSC()
	data_queue = data_distributer.get_queue()

	data_distributer.daemon = True
	data_distributer.start()

	rot_sensor = None

	reconnection_timer = None


	udp_socket = socket.socket( socket.AF_INET, socket.SOCK_DGRAM)
	udp_socket.bind(('', UDP_PORT))

	udp_reader = SocketReaderThread( udp_socket, data_queue )
	udp_reader.daemon = True
	udp_reader.start()




	def exit_master():

		if reconnection_timer:
			logger.info("Cancel timer")
			reconnection_timer.cancel()

		udp_socket.close()
		logger.info("Exit SensorMaster")


	with GracefulInterruptHandler() as h:

		while True:
			# handle reconnects an other stuff
			time.sleep(1)
			if h.interrupted:
				exit_master();
				break

	# ser_to_net = SerialToNet()
	# serial_worker = serial.threaded.ReaderThread(rot_sensor, ser_to_net)
	# serial_worker.start()

	# if not args.client:
	#     srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	#     srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	#     srv.bind(('', args.localport))
	#     srv.listen(1)
	# try:
	#     intentional_exit = False
	#     while True:
	#         if args.client:
	#             host, port = args.client.split(':')
	#             sys.stderr.write("Opening connection to {}:{}...\n".format(host, port))
	#             client_socket = socket.socket()
	#             try:
	#                 client_socket.connect((host, int(port)))
	#             except socket.error as msg:
	#                 sys.stderr.write('WARNING: {}\n'.format(msg))
	#                 time.sleep(5)  # intentional delay on reconnection as client
	#                 continue
	#             sys.stderr.write('Connected\n')
	#             client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
	#             #~ client_socket.settimeout(5)
	#         else:
	#             sys.stderr.write('Waiting for connection on {}...\n'.format(args.localport))
	#             client_socket, addr = srv.accept()
	#             sys.stderr.write('Connected by {}\n'.format(addr))
	#             # More quickly detect bad clients who quit without closing the
	#             # connection: After 1 second of idle, start sending TCP keep-alive
	#             # packets every 1 second. If 3 consecutive keep-alive packets
	#             # fail, assume the client is gone and close the connection.
	#             client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
	#             client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
	#             client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
	#             client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
	#             client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
	#         try:
	#             ser_to_net.socket = client_socket
	#             # enter network <-> serial loop
	#             while True:
	#                 try:
	#                     data = client_socket.recv(1024)
	#                     if not data:
	#                         break
	#                     ser.write(data)                 # get a bunch of bytes and send them
	#                 except socket.error as msg:
	#                     if args.develop:
	#                         raise
	#                     sys.stderr.write('ERROR: {}\n'.format(msg))
	#                     # probably got disconnected
	#                     break
	#         except KeyboardInterrupt:
	#             intentional_exit = True
	#             raise
	#         except socket.error as msg:
	#             if args.develop:
	#                 raise
	#             sys.stderr.write('ERROR: {}\n'.format(msg))
	#         finally:
	#             ser_to_net.socket = None
	#             sys.stderr.write('Disconnected\n')
	#             client_socket.close()
	#             if args.client and not intentional_exit:
	#                 time.sleep(5)  # intentional delay on reconnection as client
	# except KeyboardInterrupt:
	#     pass

	# sys.stderr.write('\n--- exit ---\n')
	# serial_worker.stop()


