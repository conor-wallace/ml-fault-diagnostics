import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import pandas as pd
import tensorflow as tf
from keras import optimizers, Sequential
from keras.models import Model
from keras.utils import plot_model
from keras.layers import Dense, LSTM, Dropout, RepeatVector, TimeDistributed
from keras.callbacks import ModelCheckpoint, TensorBoard
from sklearn.preprocessing import StandardScaler, MinMaxScaler, OneHotEncoder
from sklearn.metrics import mean_squared_error

# hyperparameters
lookback = 10
num_epochs = 100
batch_size = 32
num_features = 7
num_labels = 3
num_samples = 6750

# helper functions
def temporalize(data):
    global lookback
    X = []
    Y = []

    samples = data.shape[0] - lookback

    for i in range(samples - lookback):
        X.append(data[i:i+lookback, :-1])
        Y.append(data[i+lookback, -1])

    X = np.array(X)
    X = np.expand_dims(X, axis=2)

    Y = np.array(Y)
    Y = np.expand_dims(Y, axis=1)

    return X, Y

def create_sequence_data(data):
    global num_features, lookback
    healthy_data = data[0:2250]
    left_data = data[2250:4500]
    right_data = data[4500:-1]

    healthy_data = np.concatenate((healthy_data, np.reshape(np.repeat(0, healthy_data.shape[0]).T, (healthy_data.shape[0], 1))), axis=1)
    left_data = np.concatenate((left_data, np.reshape(np.repeat(1, left_data.shape[0]).T, (left_data.shape[0], 1))), axis=1)
    right_data = np.concatenate((right_data, np.reshape(np.repeat(2, right_data.shape[0]).T, (right_data.shape[0], 1))), axis=1)

    sequence_x, sequence_y = temporalize(healthy_data)
    sequence_x = np.reshape(sequence_x, (-1, lookback, num_features))
    sequence_y = np.reshape(sequence_y, (-1, 1))

    temp_x, temp_y = temporalize(left_data)
    temp_x = np.reshape(temp_x, (-1, lookback, num_features))
    temp_y = np.reshape(temp_y, (-1, 1))

    sequence_x = np.concatenate((sequence_x, temp_x), axis=0)
    sequence_y = np.concatenate((sequence_y, temp_y), axis=0)

    temp_x, temp_y = temporalize(right_data)
    temp_x = np.reshape(temp_x, (-1, lookback, num_features))
    temp_y = np.reshape(temp_y, (-1, 1))

    sequence_x = np.concatenate((sequence_x, temp_x), axis=0)
    sequence_y = np.concatenate((sequence_y, temp_y), axis=0)

    for i in range(sequence_y.shape[0]):
        sequence_y[i] = int(sequence_y[i])

    encoder = OneHotEncoder(sparse=False)
    sequence_y = encoder.fit_transform(sequence_y)

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
df = pd.read_csv('~/catkin_ws/src/network_faults/data/lilbot_data.csv', sep=",", header=0)
print(df.head(5))
data = df.to_numpy()
print(data.shape)

scaler = StandardScaler()
normalized_data = scaler.fit_transform(data)
sequence_x, sequence_y = create_sequence_data(normalized_data)
print(sequence_x.shape)
print(sequence_y.shape)
train_x, train_y, test_x, test_y = train_test_split(sequence_x, sequence_y, 1.0)
print(test_x.shape)

# define model
model = Sequential()
model.add(LSTM(128, activation='tanh', input_shape=(lookback, num_features), return_sequences=True))
# model.add(Dropout(0.2))
model.add(LSTM(128, activation='tanh', return_sequences=True))
# model.add(Dropout(0.2))
model.add(LSTM(64, activation='tanh'))
model.add(Dense(num_labels, activation='sigmoid'))
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

# fit model
model.fit(x=train_x, y=train_y, epochs=num_epochs, batch_size=batch_size, verbose=1, shuffle=True)

# demonstrate reconstruction
# y_validate = model.predict(test_x, verbose=0)
# yhat = model.predict(test_x, verbose=1)
# count = 0.0
# for i in range(yhat.shape[0]):
#     print("---Predicted---")
#     print(yhat[i])
#     print("---Actual---")
#     print(test_y[i])
#     if yhat[i] == test_y[i]:
#         count += 1.0
# accuracy = count / yhat.shape[0]
# print(accuracy)
# t = np.arange(normal_flattened.shape[0])
# plt.plot(t[:], normal_flattened[:, 0], 'b', label='Target')
# plt.plot(t[:], y_flattened[:, 0], 'r', label='Predictions')
# plt.title('validation')
# plt.legend()
# plt.show()
