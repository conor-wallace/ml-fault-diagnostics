import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import random
import pandas as pd
import tensorflow as tf
from keras import optimizers, Sequential
from keras.models import Model
from keras.callbacks import ModelCheckpoint
from keras.utils import plot_model
from keras.layers import Dense, CuDNNLSTM, Dropout
from keras.layers.convolutional import Conv1D, MaxPooling1D, Conv2D, MaxPooling2D
from keras.layers.merge import concatenate
from keras.callbacks import ModelCheckpoint, TensorBoard
from sklearn.preprocessing import StandardScaler, MinMaxScaler, OneHotEncoder
from sklearn.metrics import mean_squared_error

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

    # helper functions
    def temporalize(self, x, y):
        X = []
        Y = []

        samples = x.shape[0] - self.lookback

        for i in range(samples - self.lookback):
            X.append(x[i:i+self.lookback, :])
            Y.append(y[i+self.lookback])

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

    def loadData(self, path):
        df = pd.read_csv(path)
        dataset = df[['%velocity','%steering','%x','%y','%theta','%iteration','%time','%label']]
        dataset = dataset.to_numpy()
        data = dataset[:, :-1]
        print(data.shape)
        labels = np.reshape(dataset[:, -1], (-1, 1))

        scaler = MinMaxScaler(feature_range=(-1, 1))
        encoder = OneHotEncoder(sparse=False)
        normalized_data = scaler.fit_transform(data)
        onehot_labels = encoder.fit_transform(labels)
        sequence_x, sequence_y = self.temporalize(normalized_data, onehot_labels)
        self.train_test_split(sequence_x, sequence_y)

    def trainModel(self):
        # LSTM Model
        model = Sequential()
        model.add(CuDNNLSTM(128, input_shape=(self.lookback,self.num_features), return_sequences=True))
        model.add(Dropout(0.3))
        model.add(CuDNNLSTM(128))
        model.add(Dropout(0.3))
        model.add(Dense(self.num_labels, activation='softmax'))

        model.compile(optimizer='Adam', loss='categorical_crossentropy', metrics=['accuracy'])

        # checkpoint
        filepath="/home/conor/catkin_ws/src/network_faults/data/weights.best.hdf5"
        checkpoint = ModelCheckpoint(filepath, monitor='val_acc', verbose=1, save_best_only=True, mode='max')
        callbacks_list = [checkpoint]

        # fit model
        history = model.fit(self.train_x, self.train_y, validation_data=(self.test_x, self.test_y), epochs=self.num_epochs, batch_size=self.batch_size, callbacks=callbacks_list)

        plot_model(model, to_file='model.png')

        # summarize history for accuracy
        plt.plot(history.history['acc'])
        plt.plot(history.history['val_acc'])
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

if __name__ == '__main__':
    path = '/home/conor/catkin_ws/src/unity_controller/data/sim_data.csv'
    predictor1 = FaultClassifier(num_features=7, num_labels=3, lookback=10, num_epochs=1000, batch_size=512)
    predictor1.loadData(path)
    predictor1.trainModel()

    # predictor2 = FaultClassifier(num_features=5, num_labels=3, lookback=10, num_epochs=500, batch_size=32)
    # predictor2.loadData(path)
    # predictor2.trainModel()
    #
    # predictor3 = FaultClassifier(num_features=5, num_labels=3, lookback=10, num_epochs=500, batch_size=64)
    # predictor3.loadData(path)
    # predictor3.trainModel()
    #
    # predictor4 = FaultClassifier(num_features=5, num_labels=3, lookback=10, num_epochs=500, batch_size=128)
    # predictor4.loadData(path)
    # predictor4.trainModel()
