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
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.metrics import mean_squared_error

# hyperparameters
lookback = 20
num_epochs = 10
batch_size = 128
num_features = 3
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

def array_to_series(data):
    series_x = []
    series_y = []
    for i in range(data.shape[0]-lookback-1):
        normal_sample = data[i:i+lookback, :]
        series_x.append(normal_sample)

    return np.array(series_x), np.array(series_y)

def temporalize(data):
    global lookback
    sequence_x = []
    sequence_y = []
    for i in range((data.shape[0]-lookback-1)):
        normal_sample = data[i:i+lookback, 0:-1]
        normal_target = data[i+lookback, -1]
        sequence_x.append(normal_sample)
        sequence_y.append(normal_target)

    return np.array(sequence_x), np.array(sequence_y)

def create_sequence_data(data):
    global num_features, lookback
    straight = []
    left = []
    right = []

    for sample in data:
        if sample[-1] == 0.0:
            straight.append(sample)
        elif sample[-1] == 0.5:
            sample[-1] += 0.5
            left.append(sample)
        elif sample[-1] == 1.0:
            sample[-1] += 1.0
            right.append(sample)

    straight = np.array(straight)
    left = np.array(left)
    right = np.array(right)

    sequence_x, sequence_y = temporalize(straight)
    sequence_x = np.reshape(sequence_x, (-1, lookback, num_features))
    sequence_y = np.reshape(sequence_y, (-1))

    temp_x, temp_y = temporalize(left)
    temp_x = np.reshape(temp_x, (-1, lookback, num_features))
    temp_y = np.reshape(temp_y, (-1))

    sequence_x = np.concatenate((sequence_x, temp_x), axis=0)
    sequence_y = np.concatenate((sequence_y, temp_y), axis=0)

    temp_x, temp_y = temporalize(right)
    temp_x = np.reshape(temp_x, (-1, lookback, num_features))
    temp_y = np.reshape(temp_y, (-1))

    sequence_x = np.concatenate((sequence_x, temp_x), axis=0)
    sequence_y = np.concatenate((sequence_y, temp_y), axis=0)

    for i in range(sequence_y.shape[0]):
        sequence_y[i] = int(sequence_y[i])

    print(sequence_y[15000:18000])
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
df = pd.read_csv('~/catkin_ws/src/network_faults/data/training_data.csv', sep=",", header=0)
print(df.head(5))
data_df = df[["time", "x", "y", "class"]]
data = data_df.to_numpy()
print(data.shape)

scaler = MinMaxScaler()
normalized_data = scaler.fit_transform(data)
print(normalized_data[15000:18000, -1])
sequence_x, sequence_y = create_sequence_data(normalized_data)
# normal_series = array_to_series(normalized_data)
train_x, train_y, test_x, test_y = train_test_split(sequence_x, sequence_y, 0.7)
print(test_x.shape)

# define model
model = Sequential()
model.add(LSTM(128, activation='relu', input_shape=(lookback, num_features), return_sequences=True))
model.add(Dropout(0.2))
model.add(LSTM(64, activation='relu'))
model.add(Dropout(0.2))
model.add(Dense(1, activation='sigmoid'))
model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])

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
