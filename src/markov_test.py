import numpy as np

X = np.random.randint(2, size=10)
print(X)
Y = np.random.randint(2, size=10)
print(Y)

def calc_mean(data, state):
    total = 0
    for i in range(len(data)):
        if data[i] == state:
            total += 1
    return float(total)/float(len(data))

PX_loss = calc_mean(X, 1)
PX_keep = calc_mean(X, 0)
PY_loss = calc_mean(Y, 1)
PY_keep = calc_mean(Y, 0)

print("P(X=loss) = %s" % PX_loss)
print("P(X=keep) = %s" % PX_keep)
print("P(Y=loss) = %s" % PY_loss)
print("P(Y=keep) = %s" % PY_keep)
