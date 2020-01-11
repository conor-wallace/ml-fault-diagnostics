import multiprocessing
import time
import math
import numpy as np

start = time.perf_counter()

def do_something(seconds):
    print('Sleeping %s second(s)' % seconds)
    time.sleep(seconds)
    print('Done sleeping')

processes = []
for _ in range(10):
    p = multiprocessing.Process(target=do_something, args=[1])
    p.start()
    processes.append(p)

for process in processes:
    process.join()

finish = time.perf_counter()

print('Finished in %s second(s)' % round((finish-start), 3))

print(10 + 2*(math.factorial(10)/(math.factorial(10 - 2)*math.factorial(2))))

roots = np.roots([1,1,-2*210])
print(roots[roots>0])
