import numpy as np
import matplotlib.pyplot as plt
import math
import csv
import random
import pandas as pd
from sklearn.preprocessing import StandardScaler, MinMaxScaler, OneHotEncoder
from sklearn.metrics import mean_squared_error, confusion_matrix
from sklearn.feature_selection import RFE, SelectKBest, chi2
import seaborn as sns
# from nn import NN
import timeit
import multiprocessing

def temporalize(x, y, lookback):
    X = []
    Y = []

    samples = x.shape[0] - lookback

    for i in range(samples - lookback):
        X.append(x[i:i+lookback, :])
        Y.append(y[i+lookback])

    return np.array(X), np.reshape(np.array(Y), (np.array(Y).shape[0], -1))

def train_test_split(x, y):
    shuffled_a = np.empty(x.shape, dtype=x.dtype)
    shuffled_b = np.empty(y.shape, dtype=y.dtype)
    permutation = np.random.permutation(len(x))
    for old_index, new_index in enumerate(permutation):
        shuffled_a[new_index] = x[old_index]
        shuffled_b[new_index] = y[old_index]

    split = int(shuffled_a.shape[0]*0.7)
    train_x = shuffled_a[0:split]
    train_y = shuffled_b[0:split]
    test_x = shuffled_a[split:]
    test_y = shuffled_b[split:]

    return train_x, train_y, test_x, test_y

def loadData(path, threshold):
    df = pd.read_csv(path)
    dataset = df[['%x_residual','%y_residual','%theta_residual','%x','%y','%theta','%x_error','%y_error','%theta_error','%velocity','%steering','%time','%label']]
    # dataset = df[['%x','%y','%theta','%time','%label']]
    # dataset = dataset.to_numpy()
    # data = dataset[:, :-1]
    # print(data.shape)
    # labels = np.reshape(dataset[:, -1], (-1, 1))

    X = dataset.iloc[:,0:-1]  #independent columns
    y = dataset.iloc[:,-1]    #target column i.e price range
    #get correlations of each features in dataset
    corrmat = dataset.corr()
    top_corr_features = corrmat.index
    print(top_corr_features)
    print(corrmat)
    correlation_matrix = corrmat.to_numpy()
    print(correlation_matrix.shape)
    data = dataset.iloc[:, np.fabs(correlation_matrix[-1, :]) >= threshold]
    data = data.to_numpy()
    features = data[:, :-1]
    print(np.fabs(correlation_matrix[-1, :]) >= threshold)
    print(data)
    labels = np.reshape(data[:, -1], (-1, 1))
    print(features.shape)
    plt.figure(figsize=(20,20))
    #plot heat map
    g=sns.heatmap(dataset[top_corr_features].corr(),annot=True,cmap="RdYlGn")
    plt.show()
    # for i in range(data.shape[0]):
    #     x_noise = np.random.normal(0.0, 0.004, 1)
    #     y_noise = np.random.normal(0.0, 0.004, 1)
    #     theta_noise = np.random.normal(0.0, 0.004, 1)
    #     data[i, 2] += x_noise
    #     data[i, 3] += y_noise
    #     data[i, 4] += theta_noise

    return features, labels

# def trainModel(train_x, train_y, test_x, test_y, lookback, num_epochs, batch_size, dropout):
#     # LSTM Model
#     lstm_model = NN(train_x, train_y, test_x, test_y,
#                             num_features=train_x.shape[2], num_labels=train_y.shape[1],
#                             lookback=lookback, num_epochs=num_epochs, batch_size=batch_size)
#     history = lstm_model.buildModel(None, None, dropout)
#
#     print(history.history['acc'])
#     train_acc = np.array(history.history['acc'])
#     train_acc = np.reshape(train_acc, (-1))
#     print(train_acc.shape)
#     print(train_acc[-1])
#
#     print(history.history['val_acc'])
#     test_acc = np.array(history.history['val_acc'])
#     test_acc = np.reshape(test_acc, (-1))
#     print(test_acc.shape)
#     print(test_acc[-1])
#
#     score = (train_acc[-1] + test_acc[-1])/2.0
#
#     print("score: %s" % score)
#
#     return lstm_model

# def predict(test_x, test_y):
#     start_time1 = timeit.default_timer()
#
#     x = np.reshape(self.test_x[0, :, :], (1, self.lookback, self.num_features))
#     lstm_y_hat_test = self.lstm_model.model.predict(x=x, batch_size=self.batch_size, verbose=1)
#     elapsed1 = timeit.default_timer() - start_time1
#
#     print('Finished in %s second(s)' % round((elapsed1), 3))
#
#     lstm_y_hat_eval = self.lstm_model.model.evaluate(x=self.test_x, y=self.test_y, batch_size=self.batch_size, verbose=1)
#     print(lstm_y_hat_eval)
#
#     lstm_y_hat = self.lstm_model.model.predict(x=self.test_x, batch_size=self.batch_size, verbose=1)
#
#     lstm_y_pred = np.zeros((lstm_y_hat.shape[0], self.num_labels))
#     for i in range(lstm_y_hat.shape[0]):
#         if np.argmax(lstm_y_hat[i, :]) == 0:
#             lstm_y_pred[i, :] = [1, 0, 0]
#         elif np.argmax(lstm_y_hat[i, :]) == 1:
#             lstm_y_pred[i, :] = [0, 1, 0]
#         else:
#             lstm_y_pred[i, :] = [0, 0, 1]
#     lstm_y_pred = self.encoder.inverse_transform(lstm_y_pred)
#     y_true = self.encoder.inverse_transform(self.test_y)
#     print(y_true.shape)
#     lstm_conf_matrix = confusion_matrix(y_true, lstm_y_pred)
#     print("LSTM Confusion Matrix")
#     print(lstm_conf_matrix)

if __name__ == '__main__':
    path = '/home/conor/catkin_ws/src/network_faults/data/path.csv'
    lookback = 10
    num_epochs = 25
    batch_size = 64
    dropout = 0.2
    threshold = 0.0

    X, Y = loadData(path, threshold)
    scaler = MinMaxScaler(feature_range=(-1, 1))
    encoder = OneHotEncoder(sparse=False)
    normalized_data = scaler.fit_transform(X)
    onehot_labels = encoder.fit_transform(Y)
    sequence_x, sequence_y = temporalize(normalized_data, onehot_labels, lookback)
    train_x, train_y, test_x, test_y = train_test_split(sequence_x, sequence_y)
    model = trainModel(train_x, train_y, test_x, test_y, lookback, num_epochs, batch_size, dropout)
    # predict(model)
