# lstm autoencoder to recreate a timeseries
import numpy as np
from keras.models import Sequential
from keras.layers import LSTM
from keras.layers import Dense
from keras.layers import RepeatVector
from keras.layers import TimeDistributed
'''
A UDF to convert input data into 3-D
array as required for LSTM network.
'''

def temporalize(X, y, lookback):
    output_X = []
    output_y = []
    for i in range(len(X)-lookback-1):
        t = []
        for j in range(1,lookback+1):
            # Gather past records upto the lookback period
            t.append(X[[(i+j+1)], :])
        output_X.append(t)
        output_y.append(y[i+lookback+1])
    return output_X, output_y

# define input timeseries
timeseries = np.array([[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9],
                       [0.1**3, 0.2**3, 0.3**3, 0.4**3, 0.5**3, 0.6**3, 0.7**3, 0.8**3, 0.9**3]]).transpose()

timesteps = timeseries.shape[0]
n_features = timeseries.shape[1]

timesteps = 3
X, y = temporalize(X = timeseries, y = np.zeros(len(timeseries)), lookback = timesteps)

n_features = 2
X = np.array(X)
X = X.reshape(X.shape[0], timesteps, n_features)

# define model
model = Sequential()
#encoder
#return_sequences connects the matrix input at LSTM cell t to the LSTM cell t of the next layer
model.add(LSTM(128, activation='relu', input_shape=(timesteps,n_features), return_sequences=True))
model.add(LSTM(64, activation='relu', return_sequences=False))
model.add(RepeatVector(timesteps))
# The output here is the encoded data, hence the return_sequences=False and repeat vector
# Here the model is mirrored, e.g. 128 cells -> 64 cells -> 64 cells -> 128 cells
# This is the decoder and the idea is to try to reconstruct the input vector
model.add(LSTM(64, activation='relu', return_sequences=True))
model.add(LSTM(128, activation='relu', return_sequences=True))
# Output of layer five is a matrix of shape 3x128 (number of timesteps x number of LSTM cells) = U
# Layer 6 converts the input to a matrix of shape 128x2 (number of inputs x number of features) = V
model.add(TimeDistributed(Dense(n_features)))
# The output of this is the matrix multiplication of U and V (3x128) * (128x2) = (3x2) output.
# The objective function is difference between this output and the input
model.compile(optimizer='adam', loss='mse')
model.summary()

# fit model
model.fit(X, X, epochs=300, batch_size=5, verbose=0)
# demonstrate reconstruction
yhat = model.predict(X, verbose=0)
print('---Predicted---')
print(np.round(yhat,3))
print('---Actual---')
print(np.round(X, 3))
