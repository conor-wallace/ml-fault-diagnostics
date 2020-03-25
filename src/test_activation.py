from scipy.special import softmax
import numpy as np

# highest theoretical number of votes: 5
# lowest theoretical number of votes: 0
# five cluster criterion

x = np.array([[np.random.uniform(0, 5), np.random.uniform(0, 5), np.random.uniform(0, 5)],
              [np.random.uniform(0, 5), np.random.uniform(0, 5), np.random.uniform(0, 5)],
              [np.random.uniform(0, 5), np.random.uniform(0, 5), np.random.uniform(0, 5)],
              [np.random.uniform(0, 5), np.random.uniform(0, 5), np.random.uniform(0, 5)],
              [np.random.uniform(0, 5), np.random.uniform(0, 5), np.random.uniform(0, 5)]])

# first axis so that the rows reflect the voting outcome from each criterion
y = softmax(x, axis=1)
print("Activation Matrix: Row vectors contain probabilities that each index is the currect number of clusters.")
print(y*100.0)
