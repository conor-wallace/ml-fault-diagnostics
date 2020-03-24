import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import random
import pandas as pd
from sklearn.preprocessing import StandardScaler, MinMaxScaler, OneHotEncoder
from sklearn.metrics import mean_squared_error, confusion_matrix
from scipy.stats import kurtosis, skew
import timeit
import multiprocessing
import tensorflow as tf
from tensorflow.keras import backend
from keras import optimizers, Sequential
from keras.models import Model
from keras.layers import Dense, CuDNNLSTM, Dropout

class FaultClassifier():
    def __init__(self, num_features, num_labels, lookback, num_epochs, batch_size):
        self.lookback = lookback
        self.num_epochs = num_epochs
        self.batch_size = batch_size
        self.num_features = num_features
        self.num_labels = num_labels
        self.seed = 0.7
        self.train_x = None
        self.train_y = None
        self.test_x = None
        self.test_y = None
        self.model = None
        self.scaler = None
        self.encoder = None

    # helper functions
    def temporalize(self, x, y):
        X = []
        Y = []

        samples = x.shape[1] - self.lookback

        for j in range(x.shape[0]):
            for i in range(samples - self.lookback):
                X.append(x[j, i:i+self.lookback, :])
                Y.append(y[j, :])

        return np.array(X), np.reshape(np.array(Y), (np.array(Y).shape[0], -1))

    def train_test_split(self, x, y):
        shuffled_a = np.empty(x.shape, dtype=x.dtype)
        shuffled_b = np.empty(y.shape, dtype=y.dtype)
        permutation = np.random.permutation(len(x))
        for old_index, new_index in enumerate(permutation):
            shuffled_a[new_index] = x[old_index]
            shuffled_b[new_index] = y[old_index]

        split = int(shuffled_a.shape[0]*self.seed)
        self.train_x = shuffled_a[0:split]
        self.train_y = shuffled_b[0:split]
        self.test_x = shuffled_a[split:]
        self.test_y = shuffled_b[split:]

        print(self.train_x.shape)
        print(self.train_y.shape)
        print(self.test_x.shape)
        print(self.test_y.shape)

    def loadData(self, noise_data):
        noise_df = pd.read_csv(noise_data)
        noise_dataset = noise_df[['%x','%y','%theta']]
        noise_dataset = noise_dataset.to_numpy()

        noise_dataset = np.reshape(noise_dataset, (-1, 270, 3))
        statistics_dataset = np.empty((noise_dataset.shape[0], 2))

        for i in range(noise_dataset.shape[0]):
            statistics_dataset[i, :] = [kurtosis(noise_dataset[i, :, 1]), skew(noise_dataset[i, :, 1])]

        print(statistics_dataset)
        #
        # print(data.shape)
        # labels = np.reshape(dataset[:, -1], (-1, 1))
        #
        # for i in range(data.shape[0]):
        #     x_noise = np.random.normal(0.0, 0.004, 1)
        #     y_noise = np.random.normal(0.0, 0.004, 1)
        #     theta_noise = np.random.normal(0.0, 0.004, 1)
        #     data[i, 2] += x_noise
        #     data[i, 3] += y_noise
        #     data[i, 4] += theta_noise
        #
        # self.scaler = MinMaxScaler(feature_range=(-1, 1))
        # self.encoder = OneHotEncoder(sparse=False)
        # normalized_data = self.scaler.fit_transform(data)
        # onehot_labels = self.encoder.fit_transform(labels)
        sequence_x, sequence_y = self.temporalize(noise_dataset, statistics_dataset)
        print(sequence_x.shape)
        print(sequence_y.shape)
        self.train_test_split(sequence_x, sequence_y)

    def trainModel(self, dropout):
        # LSTM Model
        self.model = Sequential()
        self.model.add(CuDNNLSTM(128, input_shape=(self.lookback,self.num_features), return_sequences=True))
        self.model.add(Dropout(dropout))
        self.model.add(CuDNNLSTM(128))
        self.model.add(Dropout(dropout))
        self.model.add(Dense(self.num_labels, activation='sigmoid'))

        # # CNN Model
        # self.model = Sequential()
        # self.model.add(Conv1D(filters=128, kernel_size=3, activation='tanh', input_shape=(self.lookback,self.num_features)))
        # self.model.add(MaxPooling1D(pool_size=2))
        # self.model.add(Conv1D(filters=128, kernel_size=3, activation='tanh', input_shape=(self.lookback,self.num_features)))
        # self.model.add(MaxPooling1D(pool_size=2))
        # self.model.add(Flatten())
        # self.model.add(Dense(self.num_labels, activation='softmax'))

        self.model.compile(optimizer='Adam', loss='mse')

        self.model.fit(x=self.train_x, y=self.train_y, validation_data=(self.test_x, self.test_y), epochs=self.num_epochs, batch_size=self.batch_size, verbose=1)

        # # serialize model to YAML
        # model_path = model_path #'/home/ace/catkin_ws/src/unity_controller/data/model.yaml'
        # weights_path = weights_path #'/home/ace/catkin_ws/src/unity_controller/data/model.h5'
        # model_yaml = self.model.to_yaml()
        # with open(model_path, "w") as yaml_file:
        #     yaml_file.write(model_yaml)
        # # serialize weights to HDF5
        # self.model.save_weights(weights_path)
        # print("Saved model to disk")

    def predict(self):
        start_time1 = timeit.default_timer()

        x = np.reshape(self.test_x[0, :, :], (1, self.lookback, self.num_features))
        lstm_y_hat_test = self.lstm_model.model.predict(x=x, batch_size=self.batch_size, verbose=1)
        elapsed1 = timeit.default_timer() - start_time1

        print('Finished in %s second(s)' % round((elapsed1), 3))

        lstm_y_hat_eval = self.lstm_model.model.evaluate(x=self.test_x, y=self.test_y, batch_size=self.batch_size, verbose=1)
        print(lstm_y_hat_eval)

        lstm_y_hat = self.lstm_model.model.predict(x=self.test_x, batch_size=self.batch_size, verbose=1)

        lstm_y_pred = np.zeros((lstm_y_hat.shape[0], self.num_labels))
        for i in range(lstm_y_hat.shape[0]):
            if np.argmax(lstm_y_hat[i, :]) == 0:
                lstm_y_pred[i, :] = [1, 0, 0]
            elif np.argmax(lstm_y_hat[i, :]) == 1:
                lstm_y_pred[i, :] = [0, 1, 0]
            else:
                lstm_y_pred[i, :] = [0, 0, 1]
        lstm_y_pred = self.encoder.inverse_transform(lstm_y_pred)
        y_true = self.encoder.inverse_transform(self.test_y)
        print(y_true.shape)
        lstm_conf_matrix = confusion_matrix(y_true, lstm_y_pred)
        print("LSTM Confusion Matrix")
        print(lstm_conf_matrix)

if __name__ == '__main__':
    path_data = '/home/conor/catkin_ws/src/network_faults/data/path_data.csv'
    noise_data = '/home/conor/catkin_ws/src/network_faults/data/noise_data.csv'
    # model_path1 = '/home/ace/catkin_ws/src/unity_controller/data/model1.yaml'
    # weights_path1 = '/home/ace/catkin_ws/src/unity_controller/data/model1.h5'
    classifier1 = FaultClassifier(num_features=3, num_labels=2, lookback=10, num_epochs=1000, batch_size=128)
    classifier1.loadData(noise_data)
    classifier1.trainModel(0.1)
    # classifier1.predict()
