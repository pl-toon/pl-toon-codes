from cgitb import enable
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

#reference: https://thepoorengineer.com/en/arduino-python-plot/#python

max_x = 1000

if len(sys.argv) < 2:
	print("USAGE: \npython test_animation.py <file>")
	sys.exit(0)

path = sys.argv[1]

fig = plt.figure(figsize=(9, 6))
ax = fig.add_subplot(1, 1, 1)
ax.set_xlim([0, max_x])
#ax.set_ylim([-2, 2])

ax.grid()
ax.set_ylabel('Signal d(t)')
ax.set_xlabel('Time [ms]')
ax.set_title('ESPNOW Received Data')
ax.margins(y=.25)

lines_plot = []
lineValueText = []

for i in range(0,4):
	lines_plot.append(ax.plot([], [], label='ID_'+str(i))[0])
	lineValueText.append(ax.text(0.82, 0.95-i*0.05, '', transform=ax.transAxes))

def animate(i):
	N_esp = 1
	lines = open(path, 'r').readlines()[-5000:]
	#lines = graph_data.split('\n')
	xs = []
	ys = []
	ys2 = []
	for k in range(0, 10):
		xs.append([])
		ys.append([])
		ys2.append([])

	for line in lines:
		if len(line) > 1:
			id, t, d, v = line.split(',')
			id = int(id)
			if id + 1 > N_esp:
				N_esp = id + 1
			t = float(t)
			d = float(d)
			v = float(v)
			xs[id].append(t)
			ys[id].append(v)
			ys2[id].append(v)


	for k in range(0, N_esp):
		L = len(ys[k])
		M = min(max_x, L)
		lines_plot[k].set_data(range(0, M), ys[k][-M:])
		lineValueText[k].set_text('[ID_' + str(k) + '] = ' + '{:.3f}'.format(ys[k][-1]))
		ax.relim()
		ax.autoscale_view()

	return lines_plot

ani = animation.FuncAnimation(fig, animate, interval=20, blit=False)

plt.legend(loc='upper left')
plt.show()
