import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
from numpy import max as np_max
import sys

if len(sys.argv) < 2:
	print("USAGE: \npython test_animation.py <file>")
	sys.exit(0)

path = sys.argv[1]

data = pd.read_csv(path, header=None)
N_esp = np_max(data[0]) + 1

fig = plt.figure()
ax = fig.add_subplot(1, 2, 1)
bx = fig.add_subplot(1, 2, 2)

xs = []
ys = []		# first Y axis
ys2 = []	# second Y axis
for i in range(0, N_esp):
	xs.append([])
	ys.append([])
	ys2.append([])

graph_data = open(path, 'r').read()
lines = graph_data.split('\n')
for line in lines:
	if len(line) > 1:
		id, t1, t2, x, v = line.split(',')
		id = int(id)
		t2 = float(t1)
		x = float(x)	# velocity
		v = float(v)	# position
		xs[id].append(t2)
		ys[id].append(x)
		ys2[id].append(v)

for i in range(0, N_esp):
	ax.plot(xs[i], ys[i], linewidth=0.8, label='Position: ESP '+str(i))
	bx.plot(xs[i], ys2[i], linewidth=0.8, label='Velocity: ESP '+str(i))

ax.grid()
ax.set_ylabel('Signal x(t)')
ax.set_xlabel('Time [ms]')
ax.set_title('Position')
ax.legend()

bx.grid()
bx.set_ylabel('Signal v(t)')
bx.set_xlabel('Time [ms]')
bx.set_title('Velocity')
bx.legend()

plt.show()
