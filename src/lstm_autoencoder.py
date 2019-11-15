# lstm autoencoder to recreate a timeseries
import matplotlib.pyplot as plt
import seaborn as sns

import pandas as pd
import numpy as np
from pylab import rcParams

import tensorflow as tf
from keras import optimizers, Sequential
from keras.models import Model
from keras.utils import plot_model
from keras.layers import Dense, LSTM, RepeatVector, TimeDistributed
from keras.callbacks import ModelCheckpoint, TensorBoard

from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, precision_recall_curve
from sklearn.metrics import recall_score, classification_report, auc, roc_curve
from sklearn.metrics import precision_recall_fscore_support, f1_score
from sklearn.model_selection import train_test_split

# TODO: Gather Data from GA simulation
# TODO: Train seq2seq LSTM to match the healthy UGV path data
# TODO: measure residuals to order to determine how bad the fault is
# TODO: create fuzzy logic controller to decide what level of noise to optimize
#       for based on the measured residual

def create_dataset(df, lookback):
    #Drop useless data
    df = df.drop(['%packet_loss', '%time_delay'], axis=1)

    #Split data into positive and negative data
    df = df.to_numpy()
    scaler = MinMaxScaler(feature_range=(0, 1))
    df = scaler.fit_transform(df)
    normal_df = df[0:401]
    fault_df = df[400:-1]

    #LSTM Requires shape of n_samples, lookback, n_features
    normalX = []
    normalY = []
    for i in range(normal_df.shape[0] - 2*lookback):
        sample = normal_df[i:i+lookback]
        target = normal_df[i+lookback:i+lookback+lookback]
        normalX.append(sample)
        normalY.append(target)

    faultX = []
    faultY = []
    for i in range(fault_df.shape[0] - 2*lookback):
        sample = fault_df[i:i+lookback]
        target = normal_df[i+lookback:i+lookback+lookback]
        faultX.append(sample)
        faultY.append(target)

    normalX = np.array(normalX)
    faultX = np.array(faultX)
    normalY = np.array(normalY)
    faultY = np.array(faultY)

    dataX = np.concatenate((normalX, faultX), axis=0)
    dataY = np.concatenate((normalY, faultY), axis=0)

    return dataX, dataY


df = pd.read_csv("~/catkin_ws/src/network_faults/data/path_data.csv")

timesteps = 10
X,Y = create_dataset(df, timesteps)
trainX, testX, trainY, testY = train_test_split(X, Y)

# define model
model = Sequential()
model.add(LSTM(128, activation='relu', input_shape=(timesteps,X.shape[2]), return_sequences=True))
model.add(LSTM(64, activation='relu', return_sequences=False))
model.add(RepeatVector(timesteps))
# The output here is the encoded data, hence the return_sequences=False and repeat vector
# Here the model is mirrored, e.g. 128 cells -> 64 cells -> 64 cells -> 128 cells
# This is the decoder and the idea is to try to reconstruct the input vector
model.add(LSTM(64, activation='relu', return_sequences=True))
model.add(LSTM(128, activation='relu', return_sequences=True))
# Output of layer five is a matrix of shape 3x128 (number of timesteps x number of LSTM cells) = U
# Layer 6 converts the input to a matrix of shape 128x2 (number of inputs x number of features) = V
model.add(TimeDistributed(Dense(Y.shape[2])))
# The output of this is the matrix multiplication of U and V (3x128) * (128x2) = (3x2) output.
# The objective function is difference between this output and the input
model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])
model.summary()

# fit model
model.fit(x=X, y=Y, epochs=300, batch_size=5, verbose=1, use_multiprocessing=True)
