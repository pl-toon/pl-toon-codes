import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

if len(sys.argv) < 2:
	print("USAGE: \npython test_animation.py <file>")
	sys.exit(0)

path = sys.argv[1]
#n_esp = int(sys.argv[2])
N_esp = 1

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

def animate(i):
	global N_esp
	graph_data = open(path, 'r').read()
	lines = graph_data.split('\n')
	xs = []
	ys = []
	ys2 = []
	for i in range(0, 10):
		xs.append([])
		ys.append([])
		ys2.append([])

	for line in lines:
		if len(line) > 1:
			id, t1, t2, x, v = line.split(',')
			id = int(id)
			if id + 1 > N_esp:
				N_esp = id + 1
			t2 = float(t1)
			x = float(x)
			v = float(v)
			xs[id].append(t2)
			ys[id].append(x)
			ys2[id].append(v)

	ax.clear()
	for i in range(0, N_esp):
		ax.plot(xs[i], ys[i], linewidth=0.8)
		ax.plot(xs[i], ys2[i], linestyle='--', linewidth=0.8)

	ax.grid()
	ax.set_ylabel('Signal x(t)')
	ax.set_xlabel('Time [s]')
	ax.set_title('ESPNOW Received Data')
	#ax.axis([0, 11, 0, 1.8])

ani = animation.FuncAnimation(fig, animate, interval=50)
plt.show()
