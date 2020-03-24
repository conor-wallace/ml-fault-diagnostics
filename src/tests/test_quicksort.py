import numpy as np

def swap(x, a, b):
    temp = x[a]
    x[a] = x[b]
    x[b] = temp

def partition(x, start, end):
    pivot_index = start
    pivot_value = x[end]
    for i in range(start, end+1):
        if x[i] < pivot_value:
            swap(x, i, pivot_index)
            pivot_index = pivot_index + 1
    swap(x, pivot_index, end)

    return pivot_index

def quickSort(x, start, end):
    if(start >= end):
        return
    index = partition(x, start, end)
    quickSort(x, start, index-1)
    quickSort(x, index + 1, end)

if __name__ == '__main__':
    x = np.random.randint(low=0, high=30, size=30)
    print(x)
    quickSort(x, 0, x.shape[0]-1)
    print(x)
