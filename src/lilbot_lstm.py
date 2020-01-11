import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import random
import pandas as pd
import tensorflow as tf
from keras import optimizers, Sequential
from keras.models import Model
from keras.utils import plot_model
from keras.layers import Dense, LSTM, Dropout, RepeatVector, TimeDistributed, Flatten, Input, ConvLSTM2D
from keras.layers.convolutional import Conv1D, MaxPooling1D, Conv2D, MaxPooling2D
from keras.layers.merge import concatenate
from keras.callbacks import ModelCheckpoint, TensorBoard
from sklearn.preprocessing import StandardScaler, MinMaxScaler, OneHotEncoder
from sklearn.metrics import mean_squared_error

# hyperparameters
lookback = 10
num_epochs = 1000
batch_size = 64
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
    healthy_data = data[0:1800]
    left_data = data[1800:3600]
    right_data = data[3600:-1]

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
    global lookback, num_features, num_labels
    split = int(x.shape[0]*seed)
    split_data = random.sample(range(x.shape[0]), split)
    print(split_data)
    train_x = np.empty(shape=(split, lookback, num_features))
    train_y = np.empty(shape=(split, num_labels))
    test_x = np.empty(shape=(x.shape[0]-split, lookback, num_features))
    test_y = np.empty(shape=(x.shape[0]-split, num_labels))
    train_count = 0
    test_count = 0
    for i in range(x.shape[0]):
        if i in split_data:
            train_x[train_count] = x[i]
            train_y[train_count] = y[i]
            train_count += 1
        else:
            test_x[test_count] = x[i]
            test_y[test_count] = y[i]
            test_count += 1

    return train_x, train_y, test_x, test_y

# create data
df = pd.read_csv('~/catkin_ws/src/network_faults/data/lilbot_data.csv', sep=",", header=0)
data = df.to_numpy()

scaler = StandardScaler()
normalized_data = scaler.fit_transform(data)
sequence_x, sequence_y = create_sequence_data(normalized_data)
print(sequence_x.shape)
print(sequence_y.shape)
train_x, train_y, test_x, test_y = train_test_split(sequence_x, sequence_y, 0.75)
print(train_x.shape)
print(train_y.shape)
print(test_x.shape)
print(test_y.shape)

'''
                    %tacc1,%vacc1,%tloss1,%vloss1,%tacc2,%vacc2,%tloss2,%vloss2,%tacc3,%vacc3,%tloss3,%vloss3,%tacc4,%vacc4,%tloss4,%vloss4,%tacc5,%vacc5,%tloss5,%vloss5
LSTM             :
Multichannel CNN :
Multiheaded CNN  :
CNN-LSTM         :
ConvLSTM         :
'''

# LSTM Model
model = Sequential()
model.add(LSTM(128, activation='tanh', input_shape=(lookback, num_features), return_sequences=True))
model.add(LSTM(128, activation='tanh', return_sequences=True))
model.add(LSTM(128, activation='tanh'))
model.add(Dense(3, activation='sigmoid'))

# # Multichannel CNN Model
# model = Sequential()
# model.add(Conv1D(filters=128, kernel_size=2, activation='tanh', input_shape=(lookback,num_features)))
# model.add(Conv1D(filters=128, kernel_size=2, activation='tanh'))
# model.add(MaxPooling1D(pool_size=2))
# model.add(Flatten())
# model.add(Dense(128, activation='tanh'))
# model.add(Dense(3, activation='sigmoid'))

# # Multiheaded CNN Model
# in_layers, out_layers = list(), list()
# for i in range(num_features):
# 	inputs = Input(shape=(lookback,1))
# 	conv1 = Conv1D(filters=128, kernel_size=2, activation='tanh')(inputs)
# 	conv2 = Conv1D(filters=128, kernel_size=2, activation='tanh')(conv1)
# 	pool1 = MaxPooling1D(pool_size=2)(conv2)
# 	flat = Flatten()(pool1)
# 	# store layers
# 	in_layers.append(inputs)
# 	out_layers.append(flat)
# # merge heads
# merged = concatenate(out_layers)
# # interpretation
# dense1 = Dense(128, activation='tanh')(merged)
# outputs = Dense(3, activation='sigmoid')(dense1)
# model = Model(inputs=in_layers, outputs=outputs)
# train_x = [train_x[:,:,i].reshape((train_x.shape[0],lookback,1)) for i in range(num_features)]
# test_x = [test_x[:,:,i].reshape((test_x.shape[0],lookback,1)) for i in range(num_features)]

# # CNN-LSTM Model
# # reshape data into time steps of sub-sequences
# n_steps, n_length = 2, 5
# train_x = train_x.reshape((train_x.shape[0], n_steps, n_length, num_features))
# test_x = test_x.reshape((test_x.shape[0], n_steps, n_length, num_features))
# print(train_x.shape)
#
# model = Sequential()
# model.add(TimeDistributed(Conv1D(filters=128, kernel_size=2, activation='tanh'), input_shape=(None,n_length,num_features)))
# model.add(TimeDistributed(MaxPooling1D(pool_size=2)))
# model.add(TimeDistributed(Flatten()))
# model.add(LSTM(128, activation='tanh', return_sequences=True))
# model.add(LSTM(128, activation='tanh'))
# model.add(Dense(3, activation='sigmoid'))

# # ConvLSTM Model
# # reshape into subsequences (samples, time steps, rows, cols, channels)
# n_steps, n_length = 2, 5
# train_x = train_x.reshape((train_x.shape[0], n_steps, 1, n_length, num_features))
# test_x = test_x.reshape((test_x.shape[0], n_steps, 1, n_length, num_features))
# # define model
# model = Sequential()
# model.add(ConvLSTM2D(filters=128, kernel_size=(1,2), activation='tanh', input_shape=(n_steps, 1, n_length, num_features), return_sequences=True))
# model.add(ConvLSTM2D(filters=128, kernel_size=(1,2), activation='tanh', return_sequences=True))
# model.add(ConvLSTM2D(filters=128, kernel_size=(1,2), activation='tanh'))
# model.add(Flatten())
# model.add(Dense(3, activation='sigmoid'))

model.compile(optimizer='Adagrad', loss='categorical_crossentropy', metrics=['accuracy'])

# fit model
history = model.fit(x=train_x, y=train_y, epochs=num_epochs, batch_size=batch_size, verbose=1, validation_data=(test_x, test_y), shuffle=True)

# summarize history for accuracy
plt.plot(history.history['accuracy'])
plt.plot(history.history['val_accuracy'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()
# summarize history for loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()

# summarize history for loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()
