def quickSort(x, start, end):
    if(start >= end):
        return
    index = partition(x, start, end)
    quickSort(x, start, index-1)
    quickSort(x, index, start)

def partition(x, start, end):
    pivot_index = start
    pivot_value = x[end]
    for i in range(start, end+1):
        if x[i] < pivot_value:
            swap(x, i, pivot_index)
            pivot_index = pivot_index + 1
    swap(x, pivot_index, end)

    return pivot_index

def swap(x, a, b):
    temp = x[a]
    x[a] = x[b]
    x[b] = temp

x = [9, 3, 4, 6, 5]
quickSort(x, 0, len(x)-1)

print(x)
