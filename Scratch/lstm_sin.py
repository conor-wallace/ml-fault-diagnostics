import numpy as np
import matplotlib.pyplot as plt
import math
import tensorflow as tf
from keras import optimizers, Sequential
from keras.models import Model
from keras.utils import plot_model
from keras.layers import Dense, LSTM, RepeatVector, TimeDistributed
from keras.callbacks import ModelCheckpoint, TensorBoard
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.metrics import mean_squared_error

# hyperparameters
lookback = 10
num_epochs = 100
batch_size = 32
num_features = 2
num_samples = 400

# helper functions
def flatten(x, y):
    flat_x = []
    flat_y = []
    for i in range(x.shape[0]-1):
        flat_x.append(x[i, 0, :])
    for j in range(3):
        flat_x.append(x[i+1, j, :])

    for i in range(y.shape[0]):
        flat_y.append(y[i, 0])

    flat_y = np.array(flat_y)
    flat_x = np.array(flat_x)

    for i in range(lookback-1):
        flat_x = np.insert(flat_x, -1, np.nan, axis=0)

    for i in range(lookback+1):
        flat_y = np.insert(flat_y, 0, np.nan, axis=0)
    flat_y = np.reshape(flat_y, (-1, 1))

    return flat_x, flat_y

def array_to_series(data_normal, data_fault):
    series_x = []
    series_y = []
    for i in range(data_normal.shape[0]-lookback-1):
        normal_sample = data_normal[i:i+lookback, :]
        series_x.append(normal_sample)

    for i in range(data_fault.shape[0]-lookback-1):
        fault_sample = data_fault[i:i+lookback, :]
        series_y.append(fault_sample)

    return np.array(series_x), np.array(series_y)

def temporalize(data_normal, data_fault):
    global lookback
    sequence_x = []
    sequence_y = []
    for i in range((data_normal.shape[0]-lookback-1)):
        normal_sample = data_normal[i:i+lookback, :]
        normal_target = data_normal[i+lookback, :]
        sequence_x.append(normal_sample)
        sequence_y.append(normal_target)

    for i in range((data_fault.shape[0]-lookback-1)):
        fault_sample = data_fault[i:i+lookback, :]
        fault_target = data_fault[i+lookback, :]
        sequence_x.append(fault_sample)
        sequence_y.append(fault_target)

    return np.array(sequence_x), np.array(sequence_y)

def create_sequence_data(data_normal, data_fault):
    global num_features, lookback

    sequence_x, sequence_y = temporalize(data_normal, data_fault)
    sequence_x = np.reshape(sequence_x, (-1, lookback, num_features))
    sequence_y = np.reshape(sequence_y, (-1, num_features))

    return sequence_x, sequence_y

def train_test_split(x, y, seed):
    split = int(x.shape[0] * seed)
    train_x = x[0:split, :, :]
    train_y = y[0:split, :]
    test_x = x[split:, :, :]
    test_y = y[split:, :]
    print(train_x.shape)

    return train_x, train_y, test_x, test_y

# create data
x_n = np.array([math.sin(x) for x in np.arange(num_samples)])
x_f = np.array([math.sin(x) for x in np.arange(num_samples)])
t = np.arange(num_samples)
x_n = np.concatenate((np.reshape(x_n, [num_samples, 1]), np.reshape(t, [num_samples, 1])), axis=1)
x_f = np.concatenate((np.reshape(x_f, [num_samples, 1]), np.reshape(t, [num_samples, 1])), axis=1)
print(x_n.shape)
print(x_f.shape)
sigma = 0.1
for i in range(x_f.shape[0]):
    x_n[i, 0] += x_n[i, 0] * sigma
    x_f[i, 0] += x_f[i, 0] * sigma

x_n = np.reshape(x_n, [-1, num_features])
x_f = np.reshape(x_f, [-1, num_features])
scaler = MinMaxScaler()
normal_data = scaler.fit_transform(x_n)
fault_data = scaler.fit_transform(x_f)

sequence_x, sequence_y = create_sequence_data(normal_data, fault_data)
normal_series, fault_series = array_to_series(normal_data, fault_data)
train_x, train_y, test_x, test_y = train_test_split(sequence_x, sequence_y, 0.7)

# define model
model = Sequential()
model.add(LSTM(128, activation='relu', input_shape=(lookback, num_features), return_sequences=True))
model.add(LSTM(64, activation='relu'))
model.add(Dense(num_features))
model.compile(optimizer='adam', loss='mse')

# fit model
model.fit(x=train_x, y=train_y, epochs=num_epochs, batch_size=batch_size, verbose=1, shuffle=True)

# demonstrate reconstruction
# y_validate = model.predict(test_x, verbose=0)
yhat = model.predict(test_x, verbose=1)
normal_flattened, y_flattened = flatten(test_x, yhat)

t = np.arange(normal_flattened.shape[0])
plt.plot(t[:], normal_flattened[:, 0], 'b', label='Target')
plt.plot(t[:], y_flattened[:, 0], 'r', label='Predictions')
plt.title('validation')
plt.legend()
plt.show()
