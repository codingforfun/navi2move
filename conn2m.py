#!/usr/bin/python3

""" This is a program to download data (tracks, route, pois) from
and upload (route, pois) data to a O-Synce navi2move GPS device.
"""

__version__ = "0.10"
__author__ = "Adrian Dempwolff <adrian.dempwolff@urz.uni-heidelberg.de>"
__copyright__ = "Copyright (c) 2015 Adrian Dempwolff"
__license__ = "GPLv2+"

############################################################
############################################################
# This program comes as-is, with absolutely no warranty.
# Although thorougly tested, it can't be excluded that using
# this software may cause permanent damage to your navi2move
# device or even render it unusable.
# 
# This piece of software is distributed under the terms of
# the GNU General Public License version 2
# <http://www.gnu.org/licenses/gpl-2.0.html> or, at your
# option, any later version.
############################################################
############################################################
# Version history:
# 0.10 (02.02.2015): initial release
############################################################
############################################################

import serial
import os
import sys
import copy
import time
import datetime
import struct
from osgeo import osr
from optparse import OptionParser, OptionGroup
from xml.dom import minidom
import crcmod

class Error(Exception):
	"""is the base error class of this program.
	"""
	pass

class EndOfTransmission(Error):
	"""is raised when an ascii 0x04 (end of transmission) byte is read
	"""
	pass

class BadResponse(Error):
	"""is raised whenever an unexpected reponse is encountered
	"""
	pass 

class BadChecksum(BadResponse):
	"""is raised when a bad NMEA checksum is encountered.
	"""

class UnsupportedValue(Error):
	"""is raised when an unsupported value is tried to set on the GPS
	device.
	"""
	pass

class TimeoutError(Error):
	"""is raised when a timeout is encountered in data sending.
	"""
	pass

def announce(string, top=True, sep="*"):
	if top:
		print(sep*60)
	print(string)
	print(sep*60)

def lenOfLongestStringIn(*args):
	maxLen = 0
	for seq in args:
		for c in seq:
			if len(c) > maxLen:
				maxLen = len(c)
	return maxLen

def timestamp2String(timestamp):
	"""takes a utc <timestamp> and returns it as string formatted using the local
	timezone.
	"""
	return time.strftime("%Y-%m-%dT%H:%M:%SZ",
		datetime.datetime.fromtimestamp(timestamp).timetuple())

def timestamp2Date(timestamp):
	"""takes a utc <timestamp> and returns the date as string using the local
	timezone.
	"""
	return time.strftime("%y%m%d",
		datetime.datetime.fromtimestamp(timestamp).timetuple())

def parseTimestring(string):
	"""parses a timestamp and returns it as struct_time object.
	"""
	return time.strptime(string, "%Y-%m-%dT%H:%M:%SZ")

def string2Timestamp(string):
	"""takes a time string and returns a timestamp as produced by time.mktime().
	"""
	return time.mktime(parseTimestring(string))

def reversedDict(inDict):
	return dict(((v, k) for (k, v) in inDict.items()))

# The glyph bitmaps were derived from freetype dejavusans, size 8 using pyglet.
# All standard characters look ok, special characters might be misplaced.
# The glyph bitmaps originally used by the manufacturer's software can in
# principle be dumped using the Route().writeCharDict() method.
charBitmapDict = {
	b'\x00\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
	b'\x20\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
	b'a\x00': b'\x00\x80\xd0\x50\x50\xf0\xe0\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x03\x00\x00\x00\x00',
	b'b\x00': b'\x00\xfe\xfe\x10\x10\xf0\xe0\x00\x00\x00\x00\x00\x03\x03\x02\x02\x03\x01\x00\x00\x00\x00',
	b'c\x00': b'\x00\xc0\xe0\x10\x10\x10\x00\x00\x00\x00\x00\x00\x00\x01\x02\x02\x02\x00\x00\x00\x00\x00',
	b'd\x00': b'\x00\xe0\xf0\x10\x10\xfe\xfe\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x03\x00\x00\x00\x00',
	b'e\x00': b'\x00\xc0\xe0\x50\x50\x70\x60\x00\x00\x00\x00\x00\x00\x01\x02\x02\x02\x02\x00\x00\x00\x00',
	b'f\x00': b'\x00\x10\xfc\xfe\x12\x12\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00',
	b'g\x00': b'\x00\xe0\xf0\x10\x10\xf0\xf0\x00\x00\x00\x00\x00\x01\x0b\x0a\x0a\x0f\x07\x00\x00\x00\x00',
	b'h\x00': b'\x00\xfe\xfe\x10\x10\xf0\xe0\x00\x00\x00\x00\x00\x03\x03\x00\x00\x03\x03\x00\x00\x00\x00',
	b'i\x00': b'\x00\xf6\xf6\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'j\x00': b'\x00\x00\xf6\xf6\x00\x00\x00\x00\x00\x00\x00\x00\x08\x0f\x07\x00\x00\x00\x00\x00\x00\x00',
	b'k\x00': b'\x00\xfe\xfe\xc0\xe0\x30\x10\x00\x00\x00\x00\x00\x03\x03\x00\x01\x03\x02\x00\x00\x00\x00',
	b'l\x00': b'\x00\xfe\xfe\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'm\x00': b'\x00\xf0\xf0\x10\x10\xf0\xf0\x10\x10\xf0\xe0\x00\x03\x03\x00\x00\x03\x03\x00\x00\x03\x03',
	b'n\x00': b'\x00\xf0\xf0\x10\x10\xf0\xe0\x00\x00\x00\x00\x00\x03\x03\x00\x00\x03\x03\x00\x00\x00\x00',
	b'o\x00': b'\x00\xe0\xf0\x10\x10\xf0\xe0\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x01\x00\x00\x00\x00',
	b'p\x00': b'\x00\xf0\xf0\x10\x10\xf0\xe0\x00\x00\x00\x00\x00\x0f\x0f\x02\x02\x03\x01\x00\x00\x00\x00',
	b'q\x00': b'\x00\xe0\xf0\x10\x10\xf0\xf0\x00\x00\x00\x00\x00\x01\x03\x02\x02\x0f\x0f\x00\x00\x00\x00',
	b'r\x00': b'\x00\xf0\xf0\x10\x10\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b's\x00': b'\x00\x60\xf0\xd0\x90\x90\x00\x00\x00\x00\x00\x00\x02\x02\x02\x03\x01\x00\x00\x00\x00\x00',
	b't\x00': b'\x00\x10\xfc\xfc\x10\x10\x00\x00\x00\x00\x00\x00\x00\x01\x03\x02\x02\x00\x00\x00\x00\x00',
	b'u\x00': b'\x00\xf0\xf0\x00\x00\xf0\xf0\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x03\x00\x00\x00\x00',
	b'v\x00': b'\x00\x00\x10\xf0\xc0\x00\xc0\xf0\x10\x00\x00\x00\x00\x00\x00\x03\x03\x03\x00\x00\x00\x00',
	b'w\x00': b'\x00\x30\xf0\x80\xe0\x70\xe0\x80\xf0\x30\x00\x00\x00\x03\x03\x00\x00\x00\x03\x03\x00\x00',
	b'x\x00': b'\x00\x00\x30\xe0\xc0\xe0\x30\x00\x00\x00\x00\x00\x00\x03\x01\x00\x01\x03\x00\x00\x00\x00',
	b'y\x00': b'\x00\x10\x70\xe0\x00\xc0\x70\x10\x00\x00\x00\x00\x00\x08\x0d\x07\x01\x00\x00\x00\x00\x00',
	b'z\x00': b'\x00\x10\x90\xd0\x70\x30\x00\x00\x00\x00\x00\x00\x03\x03\x02\x02\x02\x00\x00\x00\x00\x00',
	b'A\x00': b'\x00\x00\xc0\xf8\x9c\x9c\xf8\xc0\x00\x00\x00\x00\x02\x03\x01\x00\x00\x01\x03\x02\x00\x00',
	b'B\x00': b'\x00\xfc\xfc\x24\x24\x24\xfc\xd8\x00\x00\x00\x00\x03\x03\x02\x02\x02\x03\x01\x00\x00\x00',
	b'C\x00': b'\x00\xf0\xf8\x0c\x04\x04\x04\x0c\x00\x00\x00\x00\x00\x01\x03\x02\x02\x02\x03\x00\x00\x00',
	b'D\x00': b'\x00\xfc\xfc\x04\x04\x0c\xf8\xf0\x00\x00\x00\x00\x03\x03\x02\x02\x03\x01\x00\x00\x00\x00',
	b'E\x00': b'\x00\xfc\xfc\x24\x24\x24\x24\x00\x00\x00\x00\x00\x03\x03\x02\x02\x02\x02\x00\x00\x00\x00',
	b'F\x00': b'\x00\xfc\xfc\x24\x24\x24\x24\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'G\x00': b'\x00\xf0\xf8\x0c\x04\x44\xc4\xcc\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x03\x00\x00\x00',
	b'H\x00': b'\x00\xfc\xfc\x20\x20\x20\xfc\xfc\x00\x00\x00\x00\x03\x03\x00\x00\x00\x03\x03\x00\x00\x00',
	b'I\x00': b'\x00\xfc\xfc\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'J\x00': b'\x00\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x00\x02\x02\x03\x01\x00\x00\x00\x00\x00\x00',
	b'K\x00': b'\x00\xfc\xfc\x60\xf0\x98\x0c\x04\x00\x00\x00\x00\x03\x03\x00\x00\x01\x03\x02\x00\x00\x00',
	b'L\x00': b'\x00\xfc\xfc\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x02\x02\x02\x02\x00\x00\x00\x00',
	b'M\x00': b'\x00\xfc\xfc\x1c\x60\xc0\x70\x1c\xfc\xfc\x00\x00\x03\x03\x00\x00\x00\x00\x00\x03\x03\x00',
	b'N\x00': b'\x00\xfc\xfc\x18\x60\x80\xfc\xfc\x00\x00\x00\x00\x03\x03\x00\x00\x01\x03\x03\x00\x00\x00',
	b'O\x00': b'\x00\xf0\xf8\x0c\x04\x04\x0c\xf8\xf0\x00\x00\x00\x00\x01\x03\x02\x02\x03\x01\x00\x00\x00',
	b'P\x00': b'\x00\xfc\xfc\x44\x44\x44\x7c\x38\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'Q\x00': b'\x00\xf0\xf8\x0c\x04\x04\x0c\xf8\xf0\x00\x00\x00\x00\x01\x03\x02\x02\x0f\x09\x00\x00\x00',
	b'R\x00': b'\x00\xfc\xfc\x44\x44\xfc\xb8\x00\x00\x00\x00\x00\x03\x03\x00\x00\x01\x03\x02\x00\x00\x00',
	b'S\x00': b'\x00\x38\x3c\x64\x64\xc4\xc0\x00\x00\x00\x00\x00\x03\x02\x02\x02\x03\x01\x00\x00\x00\x00',
	b'T\x00': b'\x00\x04\x04\x04\xfc\xfc\x04\x04\x04\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00',
	b'U\x00': b'\x00\xfc\xfc\x00\x00\x00\xfc\xfc\x00\x00\x00\x00\x01\x01\x02\x02\x02\x01\x01\x00\x00\x00',
	b'V\x00': b'\x00\x04\x3c\xf0\x80\x80\xf0\x3c\x04\x00\x00\x00\x00\x00\x01\x03\x03\x01\x00\x00\x00\x00',
	b'W\x00': b'\x04\x7c\xe0\x80\x78\x1c\x78\x80\xe0\x7c\x04\x00\x00\x03\x03\x00\x00\x00\x03\x03\x00\x00',
	b'X\x00': b'\x00\x04\x8c\xf8\x60\x60\xf8\x8c\x04\x00\x00\x00\x02\x03\x01\x00\x00\x01\x03\x02\x00\x00',
	b'Y\x00': b'\x00\x04\x0c\x38\xe0\xe0\x38\x0c\x04\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00',
	b'Z\x00': b'\x00\x04\x84\xc4\x64\x34\x1c\x0c\x00\x00\x00\x00\x03\x03\x02\x02\x02\x02\x02\x00\x00\x00',
	b'1\x00': b'\x00\x04\x04\xfc\xfc\x00\x00\x00\x00\x00\x00\x00\x02\x02\x03\x03\x02\x02\x00\x00\x00\x00',
	b'2\x00': b'\x00\x04\x84\xc4\x64\x3c\x18\x00\x00\x00\x00\x00\x02\x03\x02\x02\x02\x02\x00\x00\x00\x00',
	b'3\x00': b'\x00\x08\x24\x24\x24\xfc\xd8\x00\x00\x00\x00\x00\x01\x02\x02\x02\x01\x01\x00\x00\x00\x00',
	b'4\x00': b'\x00\xc0\xa0\x98\x84\xfc\xfc\x80\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00',
	b'5\x00': b'\x00\x3c\x3c\x24\x24\xe4\xc0\x00\x00\x00\x00\x00\x02\x02\x02\x02\x03\x01\x00\x00\x00\x00',
	b'6\x00': b'\x00\xf0\xf8\x24\x24\xe4\xc0\x00\x00\x00\x00\x00\x00\x01\x02\x02\x03\x01\x00\x00\x00\x00',
	b'7\x00': b'\x00\x04\x04\xc4\xf4\x3c\x0c\x00\x00\x00\x00\x00\x00\x02\x03\x00\x00\x00\x00\x00\x00\x00',
	b'8\x00': b'\x00\xd8\xfc\x24\x24\xfc\xd8\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x01\x00\x00\x00\x00',
	b'9\x00': b'\x00\x38\x7c\x44\x44\xf8\xf0\x00\x00\x00\x00\x00\x00\x02\x02\x03\x01\x00\x00\x00\x00\x00',
	b'0\x00': b'\x00\xf0\xf8\x04\x04\xf8\xf0\x00\x00\x00\x00\x00\x00\x01\x02\x02\x01\x00\x00\x00\x00\x00',
	b'\xe4\x00': b'\x00\x80\xd4\x50\x54\xf0\xe0\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x03\x00\x00\x00\x00',
	b'\xf6\x00': b'\x00\xe0\xf4\x10\x10\xf4\xe0\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x01\x00\x00\x00\x00',
	b'\xfc\x00': b'\x00\xf0\xf4\x00\x00\xf4\xf0\x00\x00\x00\x00\x00\x01\x03\x02\x02\x03\x03\x00\x00\x00\x00',
	b'\xc4\x00': b'\x00\x00\xc0\xf9\x9c\x9c\xf9\xc0\x00\x00\x00\x00\x02\x03\x01\x00\x00\x01\x03\x02\x00\x00',
	b'\xd6\x00': b'\x00\xf0\xf8\x0d\x04\x04\x0d\xf8\xf0\x00\x00\x00\x00\x01\x03\x02\x02\x03\x01\x00\x00\x00',
	b'\xdc\x00': b'\x00\xfc\xfc\x01\x00\x01\xfc\xfc\x00\x00\x00\x00\x01\x01\x02\x02\x02\x01\x01\x00\x00\x00',
	b'\xdf\x00': b'\x00\xfc\xfe\x02\x72\xfe\xcc\x80\x00\x00\x00\x00\x03\x03\x00\x02\x02\x03\x01\x00\x00\x00',
	b'\xb0\x00': b'\x00\x80\x40\x40\x80\x00\x00\x00\x00\x00\x00\x00\x01\x02\x02\x01\x00\x00\x00\x00\x00\x00',
	b'^\x00': b'\x00\x00\x00\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x02\x01\x00\x01\x02\x00\x00\x00\x00',
	b'!\x00': b'\x00\x7c\x7c\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'"\x00': b'\x00\x80\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x03\x00\x03\x00\x00\x00\x00\x00\x00\x00',
	b'\xa7\x00': b'\x00\x64\xfe\xda\xf2\x70\x00\x00\x00\x00\x00\x00\x00\x02\x02\x03\x01\x00\x00\x00\x00\x00',
	b'$\x00': b'\x00\x8c\x9e\x9a\xff\xb2\xf2\x62\x00\x00\x00\x00\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00',
	b'%\x00': b'\x00\x18\x24\x24\x18\xc0\x30\x88\x44\x40\x80\x00\x00\x00\x02\x01\x00\x00\x01\x02\x02\x01',
	b'&\x00': b'\x00\xc0\xe8\x3c\x64\xc4\x84\xe0\x60\x00\x00\x00\x01\x01\x02\x02\x03\x01\x03\x00\x00\x00',
	b'/\x00': b'\x00\x00\xc0\x1c\x02\x00\x00\x00\x00\x00\x00\x00\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00',
	b'(\x00': b'\x00\xe0\xf8\x04\x00\x00\x00\x00\x00\x00\x00\x00\x01\x07\x08\x00\x00\x00\x00\x00\x00\x00',
	b')\x00': b'\x00\x04\xf8\xe0\x00\x00\x00\x00\x00\x00\x00\x00\x08\x07\x01\x00\x00\x00\x00\x00\x00\x00',
	b'=\x00': b'\x00\x80\x80\x80\x80\x80\x80\x80\x00\x00\x00\x00\x02\x02\x02\x02\x02\x02\x02\x00\x00\x00',
	b'?\x00': b'\x00\x04\x64\x74\x3c\x18\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00',
	b'`\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x00\x00\x00\x00\x00\x00\x00',
	b'\xb4\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00',
	b'\\\x00': b'\x00\x02\x1c\xc0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x00\x00\x00\x00\x00\x00',
	b'}\x00': b'\x00\x04\x04\xbc\xf8\x40\x40\x00\x00\x00\x00\x00\x08\x08\x0f\x07\x00\x00\x00\x00\x00\x00',
	b']\x00': b'\x00\x04\x04\xfc\xfc\x00\x00\x00\x00\x00\x00\x00\x08\x08\x0f\x0f\x00\x00\x00\x00\x00\x00',
	b'[\x00': b'\x00\xfc\xfc\x04\x04\x00\x00\x00\x00\x00\x00\x00\x0f\x0f\x08\x08\x00\x00\x00\x00\x00\x00',
	b'{\x00': b'\x00\x40\x40\xf8\xbc\x04\x04\x00\x00\x00\x00\x00\x00\x00\x07\x0f\x08\x08\x00\x00\x00\x00',
	b'\xb2\x00': b'\x00\x60\x20\x20\xc0\x00\x00\x00\x00\x00\x00\x00\x02\x03\x03\x02\x00\x00\x00\x00\x00\x00',
	b'\xb3\x00': b'\x00\x20\x60\x60\xa0\x00\x00\x00\x00\x00\x00\x00\x02\x02\x02\x01\x00\x00\x00\x00\x00\x00',
	b',\x00': b'\x00\x00\xc0\xc0\x00\x00\x00\x00\x00\x00\x00\x00\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00',
	b'.\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'-\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x03\x00\x00\x00\x00\x00\x00\x00',
	b';\x00': b'\x00\x00\xcc\xcc\x00\x00\x00\x00\x00\x00\x00\x00\x02\x01\x00\x00\x00\x00\x00\x00\x00\x00',
	b':\x00': b'\x00\x30\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x03\x00\x00\x00\x00\x00\x00\x00\x00',
	b'_\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x02\x02\x02\x02\x02\x00\x00\x00\x00',
	b'\xb5\x00': b'\x00\xf0\xf0\x00\x00\xf0\xf0\x00\x00\x00\x00\x00\x0f\x0f\x02\x02\x01\x03\x02\x00\x00\x00',
	b'|\x00': b'\x00\xfe\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0f\x00\x00\x00\x00\x00\x00\x00\x00\x00',
	b'<\x00': b'\x00\xc0\xc0\xc0\x20\x20\x20\x10\x00\x00\x00\x00\x00\x00\x00\x01\x01\x01\x02\x00\x00\x00',
	b'>\x00': b'\x00\x10\x20\x20\x20\xc0\xc0\xc0\x00\x00\x00\x00\x02\x01\x01\x01\x00\x00\x00\x00\x00\x00',
	b'#\x00': b'\x00\x80\xa0\xe0\xbc\xa0\xf0\xac\x20\x00\x00\x00\x00\x02\x01\x00\x03\x00\x00\x00\x00\x00',
	b"'\x00": b'\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00',
	b'+\x00': b'\x00\x40\x40\x40\xf8\x40\x40\x40\x00\x00\x00\x00\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00',
	b'*\x00': b'\x00\x40\x80\xe0\x80\x40\x00\x00\x00\x00\x00\x00\x01\x00\x03\x00\x01\x00\x00\x00\x00\x00',
	b'~\x00': b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x01\x00\x02\x02\x01\x00\x00\x00',
	b'@\x00': b'\x00\x78\x84\x32\x49\x85\x85\x49\xfe\x86\x38\x00\x00\x00\x01\x02\x02\x02\x02\x01\x00\x00',
	b'\xac ': b'\x00\xa0\xe0\xf8\xac\xa4\x24\x04\x00\x00\x00\x00\x00\x00\x01\x03\x02\x02\x02\x00\x00\x00',
}

def charToPic(char):
	from PIL import Image
	binData = charBitmapDict[char]
	lines = (binData[:11], binData[11:])
	imData = []
	for line in lines:
		mask = 0x01
		while mask <= 0x80:
			for c in line:
				if c&mask:
					imData.append(0xff)
				else:
					imData.append(0x00)
			mask = mask << 1
	i = Image.frombytes("L", (11, 16), bytes(imData), "raw", "L;I")
	try:
		os.mkdir("charPic")
	except:
		pass
	i.save("charPic/0x{:0>2x}{:0>2x}.png".format(char[0], char[1]))

gpxTemplate = ("""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n"""
	"""<gpx xmlns="http://www.topografix.com/GPX/1/1" version="1.1" """
	"""creator="conn2m.py" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" """
	"""xsi:schemaLocation="http://www.topografix.com/GPX/1/1"""
	""" http://www.topografix.com/GPX/1/1/gpx.xsd">\n"""
	"""\t<metadata>\n"""
	"""\t\t<name>{0[gpxType]:s} export from conn2m.py</name>\n\t</metadata>\n"""
	"""{0[innerXml]:s}"""
	"""</gpx>\n"""
)


class Gpx(object):
	"""is the class to handle local gpx xml files.
	"""
	def __init__(self, filename):
		self.filename = filename
		xmlDoc = minidom.parse(filename)
		xmlDoc.normalize()
		self.xml = xmlDoc.documentElement

	def parsePois(self):
		"""parses the pois and returns them as Pois instance.
		"""
		pois = []
		for wpt in self.xml.getElementsByTagName("wpt"):
			wptDict = {}
			wptDict["lon"] = int(float(wpt.getAttribute("lon"))*1e5+.5)
			wptDict["lat"] = int(float(wpt.getAttribute("lat"))*1e5+.5)
			wptDict["timeString"] = wpt.getElementsByTagName("time")[0].firstChild.data
			try:
				wptDict["symbol"] = int(wpt.getElementsByTagName("sym")[0].firstChild.data[3:])
			except:
				wptDict["symbol"] = 0
			pois.append(wptDict)
		return pois

	def parseRoutePoints(self):
		routePoints = []
		for ind, routePoint in enumerate(self.xml.getElementsByTagName("rtept")):
			routePointDict = {}
			routePointDict["lon"] = int(float(routePoint.getAttribute("lon"))*1e5+.5)
			routePointDict["lat"] = int(float(routePoint.getAttribute("lat"))*1e5+.5)
			try:
				routePointDict["name"] = routePoint.getElementsByTagName("name")[0].firstChild.data
			except:
				routePointDict["name"] = str(ind)
			try:
				routePointDict["symbol"] = routePoint.getElementsByTagName("sym")[0].firstChild.data
			except:
				routePointDict["symbol"] = "none"
			try:
				routePointDict["elevation"] = float(routePoint.getElementsByTagName("ele")[0].firstChild.data)
			except:
				routePointDict["elevation"] = 0
			routePoints.append(routePointDict)
		return routePoints


class TrackPoint(object):
	"""is the class to handle track points.
	"""
	# WGS84, geocentric, meters from earth center in x, y, z directions
	epsg4978 = osr.SpatialReference()
	epsg4978.ImportFromEPSG(4978)
	# WGS84, geographic 2D, lon, lat (, altitude); should actually be 4979,
	# geographic 3D, but 4326 generates height
	epsg4326 = osr.SpatialReference()
	epsg4326.ImportFromEPSG(4326)
	transform = osr.CoordinateTransformation(epsg4978, epsg4326)
	#timestampOffset = time.mktime(time.strptime("1980-01-05T23:59:45Z",
		#"%Y-%m-%dT%H:%M:%SZ"))
	# seems to have changed in the meanwhile
	timestampOffset = time.mktime(time.strptime("2014-01-14T14:36:50Z",
		"%Y-%m-%dT%H:%M:%SZ"))

	def __init__(self, line):
		self.line = line
		self.initData()

	def ecef2Wgs84(self, ecefX, ecefY, ecefZ):
		"""converts a WGS84 geocentric (EPSG:4978) triple <ecefX>, <ecefY>, <ecefZ>
		to WGS84 geographic 2D (EPSG:4326) coordinates; the elevation is also
		generated so that this should be actually named EPSG:4979 (WGS84 geographic
		3D) instead.
		"""
		return self.transform.TransformPoint(ecefX, ecefY, ecefZ)

	def initData(self):
		"""integers are given as sint32, little endian"""
		self.rawTimestamp = struct.unpack("<i", self.line[0:4])[0]
		self.timestamp = self.timestampOffset + self.rawTimestamp
		self.ecefX = float(struct.unpack("<i", self.line[4:8])[0])
		self.ecefY = float(struct.unpack("<i", self.line[8:12])[0])
		self.ecefZ = float(struct.unpack("<i", self.line[12:16])[0])
		self.lon, self.lat, self.ele = self.ecef2Wgs84(
			self.ecefX, self.ecefY, self.ecefZ)

	def toXml(self):
		xmlString = []
		xmlString.append("""\t\t\t<trkpt lat="{:.5f}" lon="{:.5f}">""".format(
			self.lat, self.lon))
		xmlString.append("""\t\t\t\t<ele>{:d}</ele>""".format(int(self.ele)))
		xmlString.append("""\t\t\t\t<time>{:s}</time>""".format(
			timestamp2String(self.timestamp)))
		xmlString.append("""\t\t\t</trkpt>""")
		return "\n".join(xmlString)

	def __repr__(self):
		return "time={:s}, lon={:.5f}, lat={:.5f}, elevation={:.1f}".format(
			timestamp2String(self.timestamp), self.lon, self.lat, self.ele)


class Track(object):
	"""is the class to handle tracks.
	"""
	innerXmlTemplate = (
		"""\t<trk>\n\t\t<name>{0[trackname]:s}</name>\n\t\t<trkseg>\n"""
		"""{0[trackpointXml]:s}\n"""
		"""\t\t</trkseg>\n\t</trk>\n"""
	)
	xmlTemplate = gpxTemplate.format({"gpxType": "Track", "innerXml":
		innerXmlTemplate})

	def __init__(self):
		self.points = []
		self.pointLines = []

	def fromLines(self, lines):
		self.pointLines = lines
		self.points = [TrackPoint(line) for line in self.pointLines]
		return self

	def fromPoints(self, points):
		self.points = points
		return self

	def split(self, minTimeDiff=3600):
		"""split the track at points seperated by at least <minTimeDiff> seconds.
		"""
		tracks = []
		curTrack = []
		for point in self.points:
			if not curTrack:
				# the first point in this track section
				curTrack.append(point)
			elif point.timestamp-curTrack[-1].timestamp < minTimeDiff:
				# this point belongs to the current track section
				curTrack.append(point)
			else:
				# end of track section
				tracks.append(curTrack)
				curTrack = [point, ]
		else:
			tracks.append(curTrack)
		return [Track().fromPoints(trackPoints) for trackPoints in tracks]

	def toGpx(self):
		"""returns the track as gpx representation.
		"""
		trackpointXml = "\n".join([point.toXml() for point in self.points])
		trackname = timestamp2String(self.points[0].timestamp)
		return self.xmlTemplate.format(
			{"trackpointXml": trackpointXml, "trackname": trackname})

	def makeDatePrefix(self):
		startDate = timestamp2Date(self.points[0].timestamp)
		endDate = timestamp2Date(self.points[-1].timestamp)
		if startDate == endDate:
			return startDate
		else:
			return startDate+"-"+endDate

	def __iter__(self):
		for i in self.points:
			yield i


class Poi(object):
	"""is the class to handle single points of interest.
	"""
	def __init__(self):
		pass

	def fromBin(self, line):
		self.raw = line
		self.lat, self.lon = (i/1e5 for i in struct.unpack("<ii", line[8:16]))
		self.symbol = line[6]
		year = 1900 + line[0]
		month, day, hour, minute, second = line[1:6]
		# the time is received as utc, so convert it to the local timezone
		rawTimeString = "{:4d}-{:0>2d}-{:0>2d}T{:0>2d}:{:0>2d}:{:0>2d}Z".format(
			year, month, day, hour, minute, second)
		self.datetime = datetime.datetime.strptime(rawTimeString,
			"%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=datetime.timezone.utc).astimezone(
			tz=None)
		self.timeString = time.strftime("%Y-%m-%dT%H:%M:%SZ",
			self.datetime.timetuple())
		return self

	def fromGpxValues(self, poiDict):
		for k in poiDict:
			setattr(self, k, poiDict[k])
		return self

	def toBin(self):
		lat = struct.pack("<i", self.lat)
		lon = struct.pack("<i", self.lon)
		parsedTime = parseTimestring(self.timeString)
		year = parsedTime[0] - 1900
		month, day, hour, minute, second = parsedTime[1:6]
		time = struct.pack("<6B", year, month, day, hour, minute, second)
		symbol = struct.pack("<B", self.symbol)
		self.raw = time + symbol + b"\xa0" + lat + lon
		return self.raw

	def toXml(self):
		xmlString = []
		xmlString.append("""\t<wpt lat="{:.5f}" lon="{:.5f}">""".format(
			self.lat, self.lon))
		xmlString.append("""\t\t<name>{:s}</name>""".format(self.timeString))
		xmlString.append("""\t\t<time>{:s}</time>""".format(self.timeString))
		xmlString.append("""\t\t<sym>POI{:d}</sym>""".format(self.symbol))
		xmlString.append("""\t</wpt>""")
		return "\n".join(xmlString)


class Pois(object):
	"""is the class to handle multiple pois, either stored on the device or in a
	gpx file.
	"""
	innerXmlTemplate = "{0[poiXml]:s}\n"
	xmlTemplate = gpxTemplate.format({"gpxType": "POI", "innerXml":
		innerXmlTemplate})

	def __init__(self):
		self.poiLines = []
		self.pois = []

	def fromLines(self, lines):
		self.poiLines = lines
		self.pois = [Poi().fromBin(line) for line in self.poiLines]
		return self

	def fromGpx(self, filename):
		pois = Gpx(filename).parsePois()
		self.pois = [Poi().fromGpxValues(poi) for poi in pois]
		return self

	def toGpx(self):
		poiXml = "\n".join([poi.toXml() for poi in self.pois])
		return self.xmlTemplate.format({"poiXml": poiXml})

	def toBin(self):
		return b"".join([poi.toBin() for poi in self.pois])

	def makeDatePrefix(self):
		startDate = timestamp2Date(string2Timestamp(self.pois[0].timeString))
		endDate = timestamp2Date(string2Timestamp(self.pois[-1].timeString))
		if startDate == endDate:
			return startDate
		else:
			return startDate+"-"+endDate


class RoutePoint(object):
	"""is the class to handle route points.
	"""
	symbolDict = {
		0: "none",
		1: "straight ahead",
		2: "keep left",
		3: "keep right",
		4: "left",
		5: "backward",
		6: "right",
		7: "left rearward",
		8: "right rearward",
	}

	def __init__(self):
		self.binData = b""

	def fromBin(self, data, chartable):
		self.binData = data
		chartableInds = struct.unpack("<32H", data[:64])
		self.name = "".join([chartable[ind] for ind in chartableInds if not ind==0])
		self.symbol = struct.unpack("<H", data[64:66])[0]
		self.lat, self.lon = (i*1e-5 for i in struct.unpack("<2i", data[68:76]))
		self.elevation = 0
		return self

	def fromGpxValues(self, routePointDict):
		for k in routePointDict:
			setattr(self, k, routePointDict[k])
		try:
			self.symbol = reversedDict(self.symbolDict)[self.symbol]
		except KeyError:
			self.symbol = reversedDict(self.symbolDict)["none"]
		# chop the name at a length of 32
		self.name = self.name[:32]
		return self

	def toBin(self, chartable):
		lat = struct.pack("<i", self.lat)
		lon = struct.pack("<i", self.lon)
		symbol = struct.pack("<I", self.symbol)
		name = b"".join([struct.pack("<H", chartable.index(c)) for c in self.name])
		name += b"\x00"*(64-len(name))
		self.binData = name + symbol + lat + lon
		return self.binData

	def toXml(self):
		xmlString = []
		xmlString.append("""\t\t<rtept lat="{:.5f}" lon="{:.5f}">""".format(
			self.lat, self.lon))
		xmlString.append("""\t\t\t<name>{:s}</name>""".format(self.name))
		xmlString.append("""\t\t\t<ele>{:d}</ele>""".format(self.elevation))
		try:
			xmlString.append("""\t\t\t<sym>{:s}</sym>""".format(self.symbolDict[self.symbol]))
		except KeyError:
			xmlString.append("""\t\t\t<sym>unknown symbol</sym>""")
		xmlString.append("""\t\t</rtept>""")
		return "\n".join(xmlString)


class Route(object):
	"""is the class to handle routes.
	"""
	innerXmlTemplate = "\t<rte>\n{0[routeXml]:s}\n\t</rte>\n"
	xmlTemplate = gpxTemplate.format({"gpxType": "Route", "innerXml":
		innerXmlTemplate})

	def __init__(self):
		self.binData = b""
		self.points = []

	def readCharDict(self, filename):
		"""reads char bitmap data generated by self.writeCharDict() and returns them
		as dict.
		"""
		try:
			charBitmapsData = open(filename, "rb").read()
		except:
			return {}
		choppedCharData = [charBitmapsData[i:i+24] for i in range(0, len(charBitmapsData), 24)]
		charDict = dict([
			(charLine[:2].decode("utf-16-le"), charLine[2:]) for charLine
			in choppedCharData])
		return charDict

	def writeCharDict(self, charDict, filename):
		"""writes character bitmap data from char bitmap data section to a file
		with <filename>.
		"""
		charDictFile = open(filename, "wb")
		for c in sorted(charDict):
			charDictFile.write(c.encode("utf-16-le") + charDict[c])
		charDictFile.close()

	def makeInCharTable(self, data, charBitmapsData, useCharDataFile=False):
		"""generates self.chartable from data read from the device.
		<charBitmapsData> is the stuff in the last binary route format section,
		<data> is the chartable section
		"""
		choppedCharTable = [data[i:i+2] for i in range(0, len(data), 2)]
		chartable = [c.decode("utf-16-le") for c in choppedCharTable]
		if useCharDataFile:
			choppedCharData = [charBitmapsData[i:i+22] for i in range(0, len(charBitmapsData), 22)]
			charDict = self.readCharDict("chars")
			for ind, (c, charString) in enumerate(zip(chartable, choppedCharData)):
				if not c in charDict:
					charDict[c] = charString
				elif charDict[c] != charString:
					print("Non-matching char string for char {!s}".format(c))
			self.writeCharDict(charDict, "chars")
		return chartable

	def makeOutCharTable(self):
		"""generates a chartable and char data as needed to write data to the
		device.
		"""
		chartable = ["\x00", ]
		for point in self.points:
			for c in point.name:
				if not c in chartable:
					chartable.append(c)
		charBitmaps = []
		for c in chartable:
			if c.encode("utf-16-le") in charBitmapDict:
				charBitmaps.append(charBitmapDict[c.encode("utf-16-le")])
			else:
				charBitmaps.append(charBitmapDict["?".encode("utf-16-le")])
		charBitmapsData = b"".join(charBitmaps)
		return chartable, charBitmapsData

	def parseBin(self, dumpRaw):
		"""parses the binary data in self.binData.

		The data format is the following:
		* a header of 20 bytes, therein 
			- 2 bytes: unknown, maybe consecutive route number, up to now only seen as
			  little-endian (signed or unsigned short) 1
			- 2 bytes little-endian (unsigned) short: number of route points
			- 4 bytes little-endian (unsigned) int: offset of the chartable
			- 4 bytes little-endian (unsigned) int: offset of char bitmap data
			- 4 bytes little-endian (unsigned) int: offset of EOF
			- 2 bytes 0x0000
			- 2 bytes little-endian signed short: minus of the sum16 of the first 18
			  header bytes
		* the point data, number of route points times 76 bytes, each point
			consisting of:
			- 64 bytes string data: 32*2 bytes little-endian unsigned short, position of
			  character in the chartable
			- 4 bytes of which the first one demarks the direction symbol, 3 * 0x00
			- 4 bytes little-endian signed int: latitude in fixed-point notation,
			  lat * 1e5
			- 4 bytes little-endian signed int: longitude, dto.
		* a chartable:
			- starts always with 0x0000, may be seen as element 0 of the chartable
			  which has up to now not been seen referenced by the route points
			- 2 bytes per referenced character, probably utf-16-le encoded.
		* char bitmap data: for each character in the chartable 22 bytes bitmap
		  data for displaying the character on the device resulting in a black and
			white image of 11x16 pixles.  For details on the bitmap encoding, see code
			in charToPic().
		"""
		header = self.binData[:20]
		self.headerStart, self.numOfPoints = struct.unpack("<2H", header[:4])
		(self.charTableOffset, self.charBitmapsDataOffset,
			self.eofOffset) = struct.unpack("<3I", header[4:16])
		headerChecksum = header[18:20]
		expectedHeaderChecksum = struct.pack("<h", -(sum(header[:18])&0xffff))
		if headerChecksum != expectedHeaderChecksum:
			raise BadChecksum("Bad route data header checksum: expected {!s}, got"
				" {!s}.".format(expectedHeaderChecksum, headerChecksum))
		pointData = [self.binData[20:self.charTableOffset][i:i+76] for i in
			range(0, self.numOfPoints*76, 76)]
		charTableData = self.binData[self.charTableOffset:self.charBitmapsDataOffset]
		charBitmapsData = self.binData[self.charBitmapsDataOffset:self.eofOffset]
		choppedCharData = [charBitmapsData[i:i+22] for i in
			range(0, len(charBitmapsData), 22)]
		self.chartable = self.makeInCharTable(charTableData, charBitmapsData)
		if dumpRaw:
			print("Writing raw route data to {!s}.".format(dumpRaw))
			f = open(dumpRaw, "w")
			f.write(" ".join(["{:0>2x}".format(i) for i in header])+"\n\n")
			for p in pointData:
				f.write(" ".join(["{:0>2x}".format(i) for i in p])+"\n\n")
			f.write(" ".join(["{:0>2x}".format(i) for i in charTableData])+"\n\n")
			for c in choppedCharData:
				f.write(" ".join(["{:0>2x}".format(i) for i in c])+"\n")
			f.close()
		self.points = [RoutePoint().fromBin(data, self.chartable) for data in
			pointData]

	def fromGpx(self, filename):
		"""reads route data from a gpx file <filename> and returns a Route object
		holding the information read from this file.
		"""
		routePoints = Gpx(filename).parseRoutePoints()
		self.points = [RoutePoint().fromGpxValues(point) for point in routePoints]
		return self

	def fromBin(self, binData, dumpRaw):
		"""handles parsing of binary input data <binData> and returns a Route object
		holding the information parsed.
		"""
		self.binData = binData
		self.parseBin(dumpRaw)
		return self

	def toGpx(self):
		"""generates a gpx xml route file including all the data currently holded by
		this Route object.  The xml data is returned as string suitable for writing to a
		file.
		"""
		routeXml = "\n".join([routePoint.toXml() for routePoint in self.points])
		return self.xmlTemplate.format({"routeXml": routeXml})

	def toBin(self):
		chartable, charBitmapsDataBin = self.makeOutCharTable()
		pointDataBin = b"".join([routePoint.toBin(chartable) for routePoint in self.points])
		charTableBin = b"".join([c.encode("utf-16-le") for c in chartable])
		headerLen = 20
		numOfPointsBin = struct.pack("<H", len(self.points))
		charTableOffset = struct.pack("<I", headerLen+len(pointDataBin))
		charBitmapsDataOffset = struct.pack("<I", headerLen+len(pointDataBin)+len(charTableBin))
		eofOffset = struct.pack("<I",
			headerLen+len(pointDataBin)+len(charTableBin)+len(charBitmapsDataBin))
		unknownNumBin = struct.pack("<H", 0x01)
		headerPre = (unknownNumBin + numOfPointsBin + charTableOffset + charBitmapsDataOffset +
			eofOffset + b"\x00\x00")
		headerChecksum = struct.pack("<h", -(sum(headerPre)&0xffff))
		header = headerPre + headerChecksum
		self.binData = header + pointDataBin + charTableBin + charBitmapsDataBin
		return self.binData


class DeviceConfig(object):
	"""handles setting/getting of all kinds of device configuration acessible
	through the serial interface of the device.
	"""
	languageDict = {
		0: "en",
		1: "fr",
		2: "de",
		3: "nl",
		4: "it",
		5: "es",
	}
	turnRadiusDict = {
		5: "5 m", 10: "10 m", 20: "20 m", 30: "30 m", 50: "50 m",
	}
	lightDurationDict = {
		0: "OFF",
		10: "10 s",
		30: "30 s",
		60: "60 s",
		255: "ON",
	}
	autoOffDict = {
		0: "OFF",
		10: "10 min",
		30: "30 min",
		60: "60 min",
	}
	unitsDict = {
		0: "km, m",
		1: "mile, ft",
	}
	deviceConfigItems = [
		"language",
		"turnRadius",
		"lightDuration",
		"autoOff",
		"homeTz",
		"currentTz",
		"units",
	]
	recordConfigItems = [
		"timeInterval",
		"unknown_1", # mostly 00000, sometimes seen as 03600
		"distanceInterval",
		"unknown_2",
		"speedInterval",
		"unknown_3",
		"unknown_4",
	]

	def __init__(self, connectionObject):
		self.con = connectionObject
		self.getDeviceConfig()
		self.getRecordConfig()

	def getRecordConfig(self):
		message = b'$POEM06'
		print("Getting recording config.")
		self.con.sendCommand(message)
		resp = self.con.getResponse(expectedResponse=b'$POEM101')
		self.recordConfig, self.diskUse = self.parseRecordConfig(resp)

	def parseRecordConfig(self, string):
		rawRecordConfig = string.strip().split(b"*")[0].split(b",")[1:]
		recordConfig = {}
		for ind, item in enumerate(self.recordConfigItems):
			recordConfig[item] = int(rawRecordConfig[ind])
		diskUse = float(rawRecordConfig[-1])
		return recordConfig, diskUse

	def getDeviceConfig(self):
		message = b'$POEM14,0'
		print("Getting general device config.")
		self.con.sendCommand(message)
		resp = self.con.getResponse(expectedResponse=b"$POEM103")
		self.deviceConfig = self.parseDeviceConfig(resp)

	def parseDeviceConfig(self, string):
		# strip off the header and the trailing checksum
		rawDeviceConfig = string.strip().split(b"*")[0].split(b",")[1:]
		deviceConfig = {}
		for ind, item in enumerate(self.deviceConfigItems):
			deviceConfig[item] = int(rawDeviceConfig[ind])
		return deviceConfig

	def setDeviceConfig(self):
		"""sends the config actually stored in self.deviceConfig to the device.
		"""
		setConfig = []
		setConfig.append(b'$POEM14,1')
		for item in self.deviceConfigItems:
			setConfig.append(str(self.deviceConfig[item]).encode("ascii"))
		message = b",".join(setConfig)
		print("Setting general device config with message {!s}".format(message))
		self.con.sendCommand(message)
		self.con.getResponse(expectedResponse=b"$POEM200,14")

	def setRecordConfig(self):
		"""sends the track recording config actually stored in self.recordConfig
		to the device.
		"""
		recordConfig = []
		recordConfig.append(b'$POEM02')
		for item in self.recordConfigItems:
			recordConfig.append(str(self.recordConfig[item]).encode("ascii"))
		message = b",".join(recordConfig)
		print("Setting record config with message {!s}".format(message))
		self.con.sendCommand(message)
		self.con.getResponse(expectedResponse=b"$PMST200,02")

	def set_lightDuration(self, duration, send=True):
		"""sets the light duration of the device to <duration>.

		<duration> is one of the keys of self.lightDurationDict or "off"/"on".
		"""
		if duration in self.lightDurationDict:
			self.deviceConfig["lightDuration"] = duration
		elif type(duration)==type(str()) and duration.lower() == "off":
			self.deviceConfig["lightDuration"] = 0
		elif type(duration)==type(str()) and duration.lower() == "on":
			self.deviceConfig["lightDuration"] = 255
		else:
			raise UnsupportedValue("Unsupported light duration: {!s}".format(duration))
		if send:
			self.setDeviceConfig()

	def set_turnRadius(self, turnRadius, send=True):
		"""sets the turn radius in meters.

		<turnRadius> is an integer as specified in self.turnRadiusDict.
		"""
		if not turnRadius in self.turnRadiusDict:
			raise UnsupportedValue("Unsupported turn radius value: {!s}".format(turnRadius))
		else:
			self.deviceConfig["turnRadius"] = turnRadius
		if send:
			self.setDeviceConfig()

	def set_units(self, units, send=True):
		"""sets the units used by the device when displaying information.

		<units> may be 'km' or 'mile'.
		"""
		if not units.lower() in ('km', 'mile'):
			raise UnsupportedValue("Unsupported units: {!s}".format(units))
		else:
			self.deviceConfig["units"] = {'km': 0, 'mile': 1}[units.lower()]
		if send:
			self.setDeviceConfig()

	def _set_timezone(self, timezone, tzType, send=True):
		"""sets a timezone.

		<timezone> must be in range(-12, 13), <tzType> is either 'home' or 'current'.
		"""
		if not timezone in range(-12, 13):
			raise UnsupportedValue("Unsupported timezone: {!s}".format(timezone))
		elif tzType == 'home':
			self.deviceConfig["homeTz"] = timezone
		elif tzType == 'current':
			self.deviceConfig["currentTz"] = timezone
		if send:
			self.setDeviceConfig()

	def set_homeTz(self, timezone, send=True):
		"""sets the home timezone.  Don't know what this is for.
		"""
		self._set_timezone(timezone, 'home', send=send)
			
	def set_currentTz(self, timezone, send=True):
		"""sets the current timezone, i. e. the timezone used to display time on the
		device.
		"""
		self._set_timezone(timezone, 'current', send=send)

	def set_language(self, language, send=True):
		"""sets the interface language of the device.

		<language> is a string of length 2, supported languages are
		en, fr, de, nl, it and es.
		"""
		if language not in self.languageDict.values():
			raise UnsupportedValue("Unsupported language: {!s}".format(language))
		else:
			self.deviceConfig["language"] = reversedDict(self.languageDict)[language]
		if send:
			self.setDeviceConfig()

	def set_autoOff(self, autoOff, send=True):
		"""sets the idle time in minutes after which the device automatically
		turns off.

		<autoOff> is an integer and may take values of 10, 30 and 60 and 0 which
		means disabling of the auto-off functionality.
		"""
		if autoOff not in self.autoOffDict:
			raise UnsupportedValue("Unsupported auto-off value: {!s}".format(autoOff))
		else:
			self.deviceConfig["autoOff"] = autoOff
		if send:
			self.setDeviceConfig()

	def set_timeInterval(self, interval, send=True):
		"""sets the maximum time interval between two track points in seconds.

		A value of 0 disables the condition.
		"""
		try:
			interval = int(interval)
		except:
			raise UnsupportedValue("Unsupported time interval: {!s}".format(interval))
		self.recordConfig["timeInterval"] = interval
		if send:
			self.setRecordConfig()

	def set_distanceInterval(self, interval, send=True):
		"""sets the maximum distance between two track points in meters.

		A value of 0 disables the condition.
		"""
		try:
			interval = int(interval)
		except:
			raise UnsupportedValue("Unsupported distance: {!s}".format(interval))
		self.recordConfig["distanceInterval"] = interval
		if send:
			self.setRecordConfig()

	def set_speedInterval(self, interval, send=True):
		"""sets the maximum speed change between two track points in meters/second.

		A value of 0 disables the condition.
		"""
		try:
			interval = int(interval)
		except:
			raise UnsupportedValue("Unsupported speed change: {!s}".format(interval))
		self.recordConfig["speedInterval"] = interval
		if send:
			self.setRecordConfig()

	def printableDeviceConfig(self):
		deviceConfig = copy.copy(self.deviceConfig)
		deviceConfig["units"] = self.unitsDict[deviceConfig["units"]]
		deviceConfig["language"] = self.languageDict[deviceConfig["language"]]
		deviceConfig["lightDuration"] = self.lightDurationDict[deviceConfig["lightDuration"]]
		deviceConfig["turnRadius"] = self.turnRadiusDict[deviceConfig["turnRadius"]]
		deviceConfig["autoOff"] = self.autoOffDict[deviceConfig["autoOff"]]
		homeTzSign = ""
		if deviceConfig["homeTz"] > 0:
			homeTzSign = "+"
		elif deviceConfig["homeTz"] < 0:
			homeZtSign = "-"
		deviceConfig["homeTz"] = homeTzSign+"{:0>2d}".format(deviceConfig["homeTz"])
		currentTzSign = ""
		if deviceConfig["currentTz"] > 0:
			currentTzSign = "+"
		elif deviceConfig["currentTz"] < 0:
			currentZtSign = "-"
		deviceConfig["currentTz"] = currentTzSign+"{:0>2d}".format(deviceConfig["currentTz"])
		return deviceConfig

	def printableRecordConfig(self):
		recordConfig = {}
		recordConfig["timeInterval"] = "{:d} s".format(
			self.recordConfig["timeInterval"])
		recordConfig["distanceInterval"] = "{:d} m".format(
			self.recordConfig["distanceInterval"])
		recordConfig["speedInterval"] = "{:d} m/s".format(
			self.recordConfig["speedInterval"])
		return recordConfig

	def __repr__(self):
		deviceConfig = self.printableDeviceConfig()
		return "track recording settings: {!s}\ndevice settings: {!s}\ndisk use: {:.2f} %".format(
			self.recordConfig, deviceConfig, self.diskUse)


class NaviConnection(object):
	"""is the class which handles all device related stuff.
	"""
	#message = b'$POEM12,1'  # returns nothing but three bytes 0x02, 0x01,
	                         # 0xfe and a bunch of 0xff's
	#message = b'$POEM12,3'  # returns only 0x15's -> sending

	def __init__(self, portname="/dev/ttyUSB0", timeout=1.0, quiet=False):
		self.portname = portname
		self.timeout = timeout
		self.quiet = quiet
		self.port = serial.serial_for_url(self.portname, 9600, timeout=self.timeout)
		self.port.setInterCharTimeout(.1)
		self.port.setParity("N")
		self.port.setByteSize(8)
		self.port.setStopbits(1)
		self.port.setDsrDtr(False)
		self.port.setXonXoff(False)
		self.port.setRtsCts(False)
		self.flushBuffers()
		self.config = DeviceConfig(self)

	def calcNMEAChecksum(self, string, stripDollar=True):
		"""<string> is a bytes object to compute the checksum from.
		If <stripDollar> is set to False, a leading b"$" is not stripped off.
	
		The resulting checksum is returned as bytes object of length 2 (the
		hexadecimal representation of the checksum). 
		"""
		if stripDollar and string.startswith(b"$"):
			string = string[1:]
		checksum = 0
		for c in string:
			checksum = checksum ^ c
		return "{:0>2x}".format(checksum).encode("ascii").upper()

	def checkNMEAChecksum(self, respString):
		"""Checks if the NMEA message <respString> has a correct checksum.
		<respString> is a bytes object.
		"""
		# strip of EOL characters
		respString = respString.strip()
		# strip of a leading '$'
		if respString.startswith(b'$'):
			respString = respString[1:]
		# check if checksum is correctly attached to the message
		if not respString[-3] == ord('*'):
			raise BadChecksum("Can't handle message: {!s}".format(respString))
		# split respString into message and checksum
		message, checksum = respString[:-3], respString[-2:]
		checksum = checksum.upper()
		expectedChecksum = self.calcNMEAChecksum(message, stripDollar=False).upper()
		if checksum != expectedChecksum:
			raise BadChecksum(
				"Bad checksum for message {!s}: Expected {!s}, read {!s}".format(
				message, expectedChecksum, checksum))

	def calcCrcChecksum(self, string):
		"""returns a crc-16 (xmodem) value of string.

		<string> is a bytes object.
		"""
		c = crcmod.predefined.PredefinedCrc("xmodem")
		c.update(string)
		return c.digest()

	def checkCrcChecksum(self, string, checksum):
		"""checks the crc-16 (xmodem) checksum of a given <string> to be
		equal to a given <checksum>.

		<string> is a bytes object, <checksum> is a bytes object of length 2.
		"""
		expectedChecksum = self.calcCrcChecksum(string)
		if expectedChecksum != checksum:
			raise BadChecksum("Bad CRC checksum for string {!s}: Expected {!s}, "
				"read {!s}".format(string, expectedChecksum, checksum))

	def readBytes(self, length=1):
		byteString = self.port.read(length)
		return byteString

	def readline(self):
		return self.port.readline()

	def write(self, string):
		self.port.write(string)
		self.port.flush()

	def flushBuffers(self):
		self.port.flushInput()
		self.port.flushOutput()

	def close(self):
		self.__del__()

	def sendBreak(self, length=0.01):
		"""sends a break to the device and sleeps for a while.
		"""
		self.port.sendBreak(length)
		time.sleep(length+.1)

	def setBaudrate(self, baudrate):
		self.sendBreak()
		self.port.setBaudrate(baudrate)
		print("Set baudrate to {:d}".format(self.port.getBaudrate()))

	def sendRawCommand(self, command, sleep=True):
		"""send a raw <command> (without adding anything (checksum, eol))
		to self.port.
		<command> is a bytes object or a sequence of integers

		If <sleep> is True, wait 0.2 seconds before returning
		"""
		byteString = bytes(command)
		self.write(byteString)
		if sleep:
			time.sleep(0.2)

	def sendCommand(self, command, sleep=True):
		"""<command> a bytes object or sequence of integers.  It is expected to
		already start with a b'$' and have no EOL characters at the end.

		The command is formatted in a NMEA message-like fashion:
			* a checksum is appended to the message
			* an EOL \r\n is appended

		If <sleep> is True, wait 0.2 seconds before returning
		"""
		byteString = bytes(command)
		byteString = byteString + b'*' + self.calcNMEAChecksum(byteString)
		byteString = byteString + b'\r\n'
		self.sendRawCommand(byteString, sleep)

	def getResponse(self, checkChecksum=True, expectedResponse=None):
		"""reads one line from the device and compares it with <expectedResponse>.

		If <expectedResponse> is None, the response is returned unchecked.
		If <checkChecksum> is True, a checksum as appended to a NMEA sentence
		is generated from the response and compared to that one read with the
		response message.
		"""
		resp = self.readline()
		if not self.quiet:
			print("Received response: {!s}".format(resp), flush=True)
		if checkChecksum:
			# check for a correct checksum of the response message
			self.checkNMEAChecksum(resp)
		if expectedResponse and not resp.startswith(expectedResponse):
			raise BadResponse("Unexpected response: Expected {!s}, read: {!s}".format(
				expectedResponse, resp[:len(expectedResponse)]))
		return resp

	def __del__(self):
		self.port.__exit__()

	def parseChunks(self, chunks, pointLen, dumpRaw=False):
		"""returns a list of byte lines, each corresponding to a track point,
		poi or whatsoever
		"""
		pointLines = []
		for chunk in chunks:
			if pointLines and len(pointLines[-1]) != pointLen:
				# incomplete line from last chunk
				numOfStripBytes = pointLen - len(pointLines[-1])
				chunkPre, chunkPost = chunk[:numOfStripBytes], chunk[numOfStripBytes:]
				pointLines[-1] = pointLines[-1] + chunkPre
				chunk = chunkPost
			lines = [chunk[i:i+pointLen] for i in range(0, len(chunk), pointLen)]
			for l in lines:
				if len(l) <= pointLen and set(l) == set([0xff, ]):
					# this is the end of the chunk
					break
				pointLines.append(l)
		if dumpRaw:
			print("Writing raw data to {!s}.".format(dumpRaw))
			open(dumpRaw, "w").write("\n".join([
				" ".join(["{:0>2x}".format(el) for el in l]) for l in pointLines])+"\n")
		return pointLines

	def getDataChunk(self):
		"""reads a chunk of a track or other data like pois from the device.

		If data is available, the chunk starts with 0x02 (start of text).
		Otherwise, it starts with 0x04 (end ot transmission).

		If data is available, the size of the chunk always seems to be 1029
		consisting of
		* 3 start bytes:
			* 0x02: ascii start of text
			* 1 byte consecutive chunk number (integer)
			* 1 byte: 0xff - chunk number
		* 1024 data bytes consisting of (possibly cut at start/end)
		  track points of 20 byte size each or pois of 16 byte size each or ...
    * 2 end bytes: probably crc-16 xmodem checksum
		"""
		time.sleep(.5)
		resp = self.readBytes(self.port.inWaiting())
		if not resp:
			return b""
		elif resp[0] == 0x04:
			raise EndOfTransmission
		elif resp[0] == 0x02:
			start, resp, crc = resp[:3], resp[3:-2], resp[-2:]
			self.checkCrcChecksum(resp, crc)
		else:
			raise Error("Don't know what to do with this data chunk.  Read first"
				" byte 0x{:>2x}".format(resp[0]))
		return resp

	def getData(self, message):
		continueMessage = b'C'
		acknowledgeMessage = b'\x06'
		# send the message to initiate tracks transfer
		self.sendCommand(message)
		resp = self.getResponse(expectedResponse=b'$POEM200,12')
		# prepare the serial interface for bulk data track communication
		# comminucation won't work without this
		self.setBaudrate(115200)
		# configuration done, signalize the device that we want to continue now
		self.sendRawCommand(continueMessage)
		# get the track chunks until an EndOfTransmission is encountered
		dataChunks = []
		print("Getting data ...", end="", flush=True)
		while True:
			try:
				chunk = self.getDataChunk()
				print(".", end="", flush=True)
				self.sendRawCommand(acknowledgeMessage)
				dataChunks.append(chunk)
			except EndOfTransmission:
				print(" complete.", flush=True)
				break
		# set the baudrate back to the initial value
		self.setBaudrate(9600)
		return dataChunks

	def getTracks(self, dumpRaw=False):
		# message needed here
		message = b'$POEM12,14' # returns tracks
		print("Getting tracks.", end=" ", flush=True)
		trackChunks = self.getData(message)
		trackPointLines = self.parseChunks(trackChunks, pointLen=20,
			dumpRaw=dumpRaw)
		return Track().fromLines(trackPointLines).split()

	def getPois(self, dumpRaw=False):
		# message needed here
		message = b'$POEM12,2' # returns poi data
		print("Getting poi data.", end=" ", flush=True)
		poiChunks = self.getData(message)
		poiLines = self.parseChunks(poiChunks, pointLen=16, dumpRaw=dumpRaw)
		return Pois().fromLines(poiLines)

	def getRoute(self, dumpRaw=False):
		# message needed here
		message = b'$POEM12,12' # returns route data
		print("Getting route data.", end=" ", flush=True)
		routeChunks = self.getData(message)
		routeData = b"".join(routeChunks)
		return Route().fromBin(routeData, dumpRaw)

	def waitForAcknowledge(self, timeout=10.0):
		"""waits until an acknowledge (0x06) or negative acknowledge (0x15) byte was
		sent from the device.

		<timeout> is a timeout in seconds.  If no acknowledge byte is received
		during this time, a TimeoutError is raised.
		"""
		startTime = time.time()
		try:
			while True:
				resp = self.readBytes(1)
				if len(resp) > 0 and resp[0] == 0x15:
					return False
				elif len(resp) > 0 and resp[0] == 0x06:
					return True
				elif time.time()-startTime > timeout:
					raise TimeoutError("Didn't receive a positve or negative acknowledge"
						" byte within {:.1f} seconds.".format(timeout))
				else:
					time.sleep(.05)
		except TimeoutError as msg:
			sys.stderr.write(msg+"\n")
			sys.stderr.write("A timeout error occured.  This usually means that the"
				"\ndevice is hung or so.  Please replug the device and try again.\n")
			sys.exit(1)

	def makeChunks(self, data):
		"""chops data into 1024 byte chunks, filling the last up with 0xff's.

		Puts a header at the start and a sum8 checksum at the end.
		It's strange that different checksum algorithms are used for sending
		and receiving but the method used here seems to work.
		"""
		chunks = [data[i:i+1024] for i in range(0, len(data), 1024)]
		fillFFs = b"\xff"*(1024-len(chunks[-1]))
		chunks[-1] = chunks[-1]+fillFFs
		formattedChunks = []
		for ind, chunk in enumerate(chunks):
			header = bytes((0x02, ind+1, 0xff-ind-1))
			checksum = bytes((sum(chunk)&0xff, ))
			formattedChunks.append(header+chunk+checksum)
		return formattedChunks

	def sendData(self, initMessage, data, checkChecksum=True):
		"""sends <data> in chunks to the device, initiating data transfer using
		<initMessage>.

		<data>: a binary string.  Will be converted to appropriate chunks here.
		"""
		# send the message to initiate tracks transfer
		self.sendCommand(initMessage)
		resp = self.getResponse(checkChecksum=checkChecksum,
			expectedResponse=b'$POEM200,12')
		# prepare the serial interface for bulk data track communication
		# comminucation won't work without this
		self.setBaudrate(115200)
		# send data chunks, then 0x04 (end of transmission)
		dataChunks = self.makeChunks(data)
		print("Sending data ...", end="", flush=True)
		# get the initial negative acknowledge
		resp = self.waitForAcknowledge()
		for chunk in dataChunks:
			resp = False
			while resp is False:
				print(".", end="", flush=True)
				self.sendRawCommand(chunk)
				# resp will get True if the data transfer is positively acknowledged by
				# the device
				resp = self.waitForAcknowledge()
		# all chunks sent, send an end of transmission byte
		self.sendRawCommand(b"\x04")
		try:
			assert self.waitForAcknowledge()==True
		except AssertionError:
			raise Error("End of transfer was not acknowledged by the device.")
		print(" complete.", flush=True)
		# set the baudrate back to the initial value
		self.setBaudrate(9600)

	def sendPois(self, filename):
		"""sends pois in gpx file <filename> to the device.  The device answers the
		initial message with a $POEM200,12,004 with a wrong checksum, so we turn off
		checksum checking here.
		"""
		print("Sending pois from gpx file {!s} to the device.".format(filename))
		sendPoiMessage = b"$POEM12,4"
		poiData = Pois().fromGpx(filename).toBin()
		self.sendData(sendPoiMessage, poiData, checkChecksum=False)

	def sendRoute(self, filename):
		"""sends a route saved in a gpx file <filename> to the device.
		"""
		print("Sending route from gpx file {!s} to the device.".format(filename))
		sendRouteMessage = b"$POEM12,11"
		routeData = Route().fromGpx(filename).toBin()
		self.sendData(sendRouteMessage, routeData, checkChecksum=False)


############################################################
############################################################
############################################################

modes = {
	"get-tracks": "get tracks from the device",
	"print-config": "print the device configuration",
	"set-config": "set device configuration items",
	"get-pois": "get pois from the device",
	"send-pois": "send pois to the device",
	"get-route": "download route from device",
	"send-route": "send a route to the device",
}

def makeModeStrings():
	modeStrings = []
	maxLen = lenOfLongestStringIn(modes)
	for mode in sorted(modes):
		desc = modes[mode]
		modeStrings.append("\n\t* "+"{{: <{:d}s}}".format(maxLen).format(mode)
			+" - {:s}".format(desc))
	return "".join(modeStrings)

def parseCommandline():
	parser = OptionParser(usage="%prog <mode> [options]"
		"\n\nis a program to interact with a O-Synce navi2move GPS device."
		"\nAt least a <mode> must be specified.  Possible modes"
		" are:"+makeModeStrings(), version="%prog {:s}".format(__version__))
	# device options
	devOptGroup = OptionGroup(parser, "Device related options")
	devOptGroup.add_option("-p", "--port", help="the usb serial interface,"
		"\ne. g. /dev/ttyUSB0.  The default is 'auto' which means scanning"
		"\nfor an appropriate device.", action="store", default="auto",
		dest="port")
	parser.add_option_group(devOptGroup)
	# options belonging to several modes
	miscOptGroup = OptionGroup(parser, "Options related to modes producing output",
		"that is 'get-tracks', 'get-pois' and 'get-route'")
	miscOptGroup.add_option("-o", "--output-prefix", metavar="PREFIX",
		help="set a PREFIX for the output gpx files.  The default prefix is 'track'"
		"\nin 'get-tracks' mode, 'pois' in 'get-pois' mode and 'route' in"
		"\n'get-route' mode", dest="outputPrefix", action="store", default=None)
	miscOptGroup.add_option("--no-date-prefixes", help="filenames of output"
		"\ngpx track and POI files are prefixed with a date representing the"
		"\nrecording date of the written data by default.  Use this option to"
		"\nturn this behaviour off.", dest="useDatePrefix", action="store_false",
		default="True")
	parser.add_option_group(miscOptGroup)
	# options belonging to send-* modes
	sendOptGroup = OptionGroup(parser, "Options related to 'send-pois' and "
		"'send-route' modes")
	sendOptGroup.add_option("--use-gpx", help="send data read from the gpx file"
		"\nFILENAME to the device.", action="store", default=None, dest="gpxFile",
		metavar="FILENAME")
	parser.add_option_group(sendOptGroup)
	# set-config options
	setConfigOptGroup = OptionGroup(parser, "Options related to 'set-config' mode",
		"use these to set the device configuration.")
	setConfigOptGroup.add_option("--language", help="set the device language.",
		dest="set-config_dev_language", action="store", default=None, metavar="LANGUAGE")
	setConfigOptGroup.add_option("--light-duration", help="set light duraction to"
		"\nso many seconds.  Possible values are 10, 30, 60, 0 (for always off) and"
		"\n255 (for always on)", type="int", dest="set-config_dev_lightDuration",
		action="store", default=None, metavar="DURATION")
	setConfigOptGroup.add_option("--auto-off", help="auto switch off the device"
		"\nwhen idle for TIME minutes.  Possible values are 10, 30 and 60.  Saying"
		"\n0 here means disabling of the auto-off feature of the device.",
		dest="set-config_dev_autoOff", action="store", type="int", default=None,
		metavar="TIME")
	setConfigOptGroup.add_option("--turn-radius", help="set the turn radius in"
		"\nmeters.  The turn radius controls when the device switches between"
		"\nwaypoints when routing.  Possible values are 5, 10, 20, 30 and 50.",
		action="store", type="int", default=None, dest="set-config_dev_turnRadius",
		metavar="TURN-RADIUS")
	setConfigOptGroup.add_option("--units", help="set the units used by the device"
		"\nto display data.  Possible values are 'km' which means kph/km/m and"
		"\n'mile' which means mph/mile/feet.", action="store", default=None,
		dest="set-config_dev_units", metavar="UNIT")
	setConfigOptGroup.add_option("--home-timezone", help="set the home timezone."
		"\n Possible values are between -12 and +12.", type="int", default=None,
		action="store", metavar="TIMEZONE", dest="set-config_dev_homeTz")
	setConfigOptGroup.add_option("--current-timezone", help="set the current timezone."
		"\n Possible values are between -12 and +12.", type="int", default=None,
		action="store", metavar="TIMEZONE", dest="set-config_dev_currentTz")
	parser.add_option_group(setConfigOptGroup)
	# track recording options
	recOptGroup = OptionGroup(parser, "Options related to set-config mode",
		"use these to set the track recording behaviour of the device.")
	recOptGroup.add_option("--time-interval", help="maximum time between to"
		"\ntrack points in seconds.  A value of 0 disables this condition.",
		action="store", type="int", default=None, metavar="INTERVAL",
		dest="set-config_rec_timeInterval")
	recOptGroup.add_option("--distance-interval", help="maximum distance between to"
		"\ntrack points in meters.  A value of 0 disables this condition.",
		action="store", type="int", default=None, metavar="INTERVAL",
		dest="set-config_rec_distanceInterval")
	recOptGroup.add_option("--speed-interval", help="maximum speed change between to"
		"\ntrack points in meters per second.  A value of 0 disables this condition.",
		action="store", type="int", default=None, metavar="INTERVAL",
		dest="set-config_rec_speedInterval")
	parser.add_option_group(recOptGroup)
	# debug options
	debugOptGroup = OptionGroup(parser, "Debug options")
	debugOptGroup.add_option("--dump-raw", help="dump raw data downloaded"
		"\nfrom the device to FILENAME.", dest="dumpRaw", action="store",
		default=None, metavar="FILENAME")
	parser.add_option_group(debugOptGroup)
	opts, args = parser.parse_args()
	if len(args) != 1:
		sys.stderr.write("Please specify exactly one mode\n\n")
		parser.print_help()
		sys.exit(1)
	elif args[0] not in modes:
		sys.stderr.write("No such mode available: {!s}\n\n".format(args[0]))
		parser.print_help()
		sys.exit(0)
	else:
		mode = args[0]
	if mode == "get-tracks" and opts.outputPrefix is None:
		opts.outputPrefix = "track"
	elif mode == "get-pois" and opts.outputPrefix is None:
		opts.outputPrefix = "pois"
	elif mode == "get-route" and opts.outputPrefix is None:
		opts.outputPrefix = "route"
	return mode, opts


def writeTrackGpx(track, filename):
	announce("Writing track file {!s}".format(filename), False, "-")
	open(filename, "w").write(track.toGpx())

def writeTracks(tracks, filenamePrefix, useDatePrefix):
	announce("Writing tracks.")
	for ind, track in enumerate(tracks):
		filename = "{:s}{:0>3d}.gpx".format(filenamePrefix, ind)
		if useDatePrefix:
			filename = track.makeDatePrefix()+"_"+filename
		writeTrackGpx(track, filename=filename)

def writePois(pois, filenamePrefix, useDatePrefix):
	filename = filenamePrefix+".gpx"
	if useDatePrefix:
		filename = pois.makeDatePrefix()+"_"+filename
	announce("Writing poi file {!s}".format(filename))
	open(filename, "w").write(pois.toGpx())

def writeRoute(route, filenamePrefix):
	filename = filenamePrefix+".gpx"
	announce("Writing route file {!s}".format(filename))
	open(filename, "w").write(route.toGpx())

def printConfig(p):
	devConfig = p.config.printableDeviceConfig()
	recConfig = p.config.printableRecordConfig()
	diskUse = p.config.diskUse
	announce("Device configuration", True)
	maxLen = lenOfLongestStringIn(devConfig, recConfig)
	announce("* General device configuration:", False, "-")
	for c in sorted(devConfig):
		print("\t{{: <{:d}s}}: ".format(maxLen).format(c), end="")
		print(devConfig[c])
	announce("* Track recording configuration:", True, "-")
	for c in sorted(recConfig):
		print("\t{{: <{:d}s}}: ".format(maxLen).format(c), end="")
		print(recConfig[c])
	announce("* Disk usage: {:.2f} %".format(diskUse), True, "-")
	print()

def setConfig(p, opts):
	devConfigItems = []
	recConfigItems = []
	for o in dir(opts):
		if o.startswith("set-config_dev_") and getattr(opts, o) != None:
			devConfigItems.append(("set_"+o.split("_")[2], getattr(opts, o)))
		elif o.startswith("set-config_rec_") and getattr(opts, o) != None:
			recConfigItems.append(("set_"+o.split("_")[2], getattr(opts, o)))
	for ind, (configItem, value) in enumerate(devConfigItems):
		if ind == len(devConfigItems)-1:
			getattr(p.config, configItem)(value, send=True)
		else:
			getattr(p.config, configItem)(value, send=False)
	for ind, (configItem, value) in enumerate(recConfigItems):
		if ind == len(recConfigItems)-1:
			getattr(p.config, configItem)(value, send=True)
		else:
			getattr(p.config, configItem)(value, send=False)
	p.config.getDeviceConfig()
	p.config.getRecordConfig()
	printConfig(p)

def checkGpx(filename):
	if not filename:
		sys.stderr.write("No gpx file specified.  Use the --use-gpx option.\n")
		sys.exit(1)
	try:
		os.stat(filename)
		if not os.path.isfile(filename):
			raise Error("Not a regular file: {!s}".format(filename))
	except:
		sys.stderr.write("No such file: {!s}\n".format(filename))
		sys.exit(0)

def sendPois(p, opts):
	checkGpx(opts.gpxFile)
	p.sendPois(opts.gpxFile)

def sendRoute(p, opts):
	checkGpx(opts.gpxFile)
	p.sendRoute(opts.gpxFile)

def scanForDevice():
	cp210xDir = "/sys/bus/usb/drivers/cp210x"
	try:
		os.stat(cp210xDir)
	except:
		raise Error("No device connected.")
	for i in os.listdir(cp210xDir):
		if i[0].isnumeric():
			try:
				lines = open(os.path.join(cp210xDir, i,
					"modalias")).read().split("\n")
				for l in lines:
					l = l.lower()
					if l.startswith("usb:") and l[4:14]=="v10c4pea60":
						dev = [f for f in os.listdir(os.path.join(cp210xDir, i)) if
							f.startswith("ttyUSB")][0]
						raise StopIteration
			except StopIteration:
				break
			except Exception:
				pass
	else:
		raise Error("Couldn't find any matching device.")
	return "/dev/"+dev


def main():
	mode, opts = parseCommandline()
	if opts.port == "auto":
		opts.port = scanForDevice()
		announce("Selected {:s} as device interface port.".format(opts.port), True)
	p = NaviConnection(portname=opts.port)
	if mode == "get-tracks":
		tracks = p.getTracks(opts.dumpRaw)
		writeTracks(tracks, opts.outputPrefix, opts.useDatePrefix)
	elif mode == "print-config":
		printConfig(p)
	elif mode == "set-config":
		setConfig(p, opts)
	elif mode == "get-pois":
		pois = p.getPois(opts.dumpRaw)
		writePois(pois, opts.outputPrefix, opts.useDatePrefix)
	elif mode == "send-pois":
		sendPois(p, opts)
	elif mode == "get-route":
		route = p.getRoute(opts.dumpRaw)
		writeRoute(route, opts.outputPrefix)
	elif mode == "send-route":
		sendRoute(p, opts)
	p.close()

if __name__ == "__main__":
	main()

