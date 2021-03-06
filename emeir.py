#!/usr/bin/python -u
#
# emeir.py
# 
# Program to read the electrical meter using a reflective light sensor
# This is the data recording part running on a Raspberry Pi.
# It retrieves data from the Arduino over USB serial and stores
# counter and consumption values into a round robin database.

# Copyright 2015 Martin Kompf
# Copyright 2015 Manuel Muehlig for changes related to multiple sensors and mqtt
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import serial
import time
import sys
import os
import re
import argparse
import rrdtool

# number of sensors
num_sensors = 2

# Serial port of arduino
port = '/dev/ttyUSB0'

# Revolutions per kWh
rev_per_kWh = 75

# Path to RRD with counter values
count_rrd_template = "%s/emeir_" % (os.path.dirname(os.path.abspath(__file__)))



# Create the Round Robin Database
def create_rrd(sensor, initial_value = 0):
  count_rrd = count_rrd_template + str(sensor) + ".rrd"
  print 'Creating RRD: ' + count_rrd
  # Create RRD to store counter and consumption:
  # 1 trigger cycle matches consumption of 1/revs_per_kWh
  # Counter is GAUGE (kWh)
  # Consumption is ABSOLUTE (W)
  # 1 value per minute for 3 days
  # 1 value per day for 30 days
  # 1 value per week for 10 years
  # Consolidation LAST for counter
  # Consolidation AVERAGE for consumption
  try:
    rrdtool.create(count_rrd, 
      '--no-overwrite',
      '--step', '60',
      'DS:counter:GAUGE:86400:0:1000000',
      'DS:consum:ABSOLUTE:86400:0:1000000',
      'RRA:LAST:0.5:1:4320',
      'RRA:AVERAGE:0.5:1:4320',
      'RRA:LAST:0.5:1440:30',
      'RRA:AVERAGE:0.5:1440:30',
      'RRA:LAST:0.5:10080:520',
      'RRA:AVERAGE:0.5:10080:520')
  except Exception as e:
    print 'Error ' + str(e)
  
  if initial_value != 0:
    update = "N:%.2f:%.0f" % (initial_value, 0)
    rrdtool.update(count_rrd, update)

# Get the last counter value from the rrd database
def last_rrd_count(sensor):
  val = 0.0
  handle = os.popen("rrdtool lastupdate " + count_rrd_template + str(sensor) + ".rrd")
  for line in handle:
    m = re.match(r"^[0-9]*: ([0-9.]*) [0-9.]*", line)
    if m:
      val = float(m.group(1))
      break
  handle.close()
  return val

# Main
def main():
  # Check command args
  parser = argparse.ArgumentParser(description='Program to read the electrical meter using a reflective light sensor.')
  parser.add_argument('-c', '--create', action='store_true', default=False, help='Create rrd database if necessary')
  parser.add_argument('-i','--initial', nargs=num_sensors, help='Provides a list with initial values when creating rrd', required=False)
  args = parser.parse_args()

  if args.create:
    for i in range(0,num_sensors):
        if args.initial:
          create_rrd(i, float(args.initial[i]))
        else:
          create_rrd(i)

  # Open serial line
  ser = serial.Serial(port, 9600)
  if not ser.isOpen():
    print "Unable to open serial port %s" % port
    sys.exit(1)

  trigger_state = []
  counter = []
  for i in range(0, num_sensors):
    trigger_state.append(0)
    counter.append(last_rrd_count(i))
    print "restoring counter for sensor %d to %f" % (i, counter[i])

  trigger_step = 1.0 / rev_per_kWh
  while(True):
    # Read line from arduino and convert to trigger value
    line = ser.readline()
    line = line.strip()
    elements = line.split()
    if len(elements) != 2:
        continue

    sensor = int(elements[0])
    old_state = trigger_state[sensor]
    if elements[1] == '1':
      trigger_state[sensor] = 1
    elif elements[1] == '0':
      trigger_state[sensor] = 0
    if old_state == 1 and trigger_state[sensor] == 0:
      # trigger active -> update count rrd
      counter[sensor] += trigger_step
      update = "N:%.2f:%.0f" % (counter[sensor], trigger_step*3600000.0)
      #print update
      count_rrd = count_rrd_template + str(sensor) + ".rrd"
      rrdtool.update(count_rrd, update)


if __name__ == '__main__':
  main()
