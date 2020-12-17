import time
import psutil
import matplotlib.pyplot as plt


fig = plt.figure()
ax = fig.add_subplot(111)
fig.show()

i = 0
x, y = [], []
end = 0

while True:
    elapse_time = time.time() - end
    end = time.time()
    x.append(i)
    y.append(psutil.cpu_percent())
    
    ax.plot(x, y, color='b')
    
    fig.canvas.draw()
    
    ax.set_xlim(left=max(0, i-50), right=i+50)
    
    time.sleep(0.1)
    i += 1
    print(elapse_time)