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
from Queue import *
from threading import Thread, Timer
import traceback
import struct
import glob

logger = logging.getLogger("SensorMaster")
logger.setLevel(logging.DEBUG)


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

	if header == 'o':
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
			print datum
			self.pub_socket.send_json(datum)
			if self.data_queue.qsize() > 10:
				logger.warning( "Data Queue size increasing")
			time.sleep(0.01)

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

	data_distributer = DataDistributer()
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
		clf.close()
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


