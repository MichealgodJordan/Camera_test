#!/usr/bin/env python3
"""Plot /tmp/yolo_metrics.csv using matplotlib. One plot per metric (no seaborn; single plot per figure).
"""
import matplotlib.pyplot as plt
import csv
import os
import datetime

path = '/tmp/yolo_metrics.csv'
if not os.path.exists(path):
    print('No metrics file at', path)
    exit(1)

times, fps, cpu, mem = [], [], [], []
with open(path,'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        ts = float(row['timestamp'])
        times.append(datetime.datetime.fromtimestamp(ts))
        fps.append(float(row['fps']))
        cpu.append(float(row['cpu_percent']))
        mem.append(float(row['mem_percent']))

plt.figure()
plt.plot(times, fps)
plt.title('FPS over time')
plt.xlabel('time')
plt.ylabel('FPS')
plt.gcf().autofmt_xdate()
plt.show()

plt.figure()
plt.plot(times, cpu)
plt.title('CPU percent over time')
plt.xlabel('time')
plt.ylabel('CPU %')
plt.gcf().autofmt_xdate()
plt.show()

plt.figure()
plt.plot(times, mem)
plt.title('Memory percent over time')
plt.xlabel('time')
plt.ylabel('Memory %')
plt.gcf().autofmt_xdate()
plt.show()
