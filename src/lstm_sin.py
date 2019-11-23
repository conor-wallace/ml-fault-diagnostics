# LSTM for international airline passengers problem with regression framing
import numpy
import matplotlib.pyplot as plt
import math
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import mean_squared_error
# convert an array of values into a dataset matrix
# def create_dataset(dataset, look_back=1):
	# # dataX, dataY = [], []
	# # for i in range(len(dataset)-look_back-1):
	# # 	a = dataset[i:(i+look_back), 0]
	# # 	dataX.append(a)
	# # 	dataY.append(dataset[i + look_back, 0])
    # #segment data into features and labels
    # X = []
    # Y = []
    #
    # sequence_length = 5*look_back
    # number_samples = len(dataset) - sequence_length
    #
    # for i in range(number_samples):
    #     #for i = 0, X = sin_wave[0:50], Y = sin_wave[50], i = 0, 1, ..., n
    #     X.append(dataset[i:i+look_back])
    #     Y.append(dataset[i+look_back])
    #
    # X = numpy.array(X)
    # X = numpy.expand_dims(X, axis=2)
    #
    # Y = numpy.array(Y)
    # Y = numpy.expand_dims(Y, axis=1)
    #
    # X_val = []
    # Y_val = []
    #
    # for i in range(number_samples-look_back, len(dataset)-look_back):
    #     X_val.append(dataset[i:i+look_back])
    #     Y_val.append(dataset[i+look_back])
    #
    # X_val = numpy.array(X_val)
    # X_val = numpy.expand_dims(X_val, axis=2)
    #
    # Y_val = numpy.array(Y_val)
    # Y_val = numpy.expand_dims(Y_val, axis=1)
    # return X, Y, X_val, Y_val

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
    for i in range(normal_df.shape[0] - lookback):
        sample = normal_df[i:i+lookback]
        normalX.append(sample)

    faultX = []
    for i in range(fault_df.shape[0] - lookback):
        sample = fault_df[i:i+lookback]
        faultX.append(sample)

    normalX = np.array(normalX)
    faultX = np.array(faultX)
    normalY = np.zeros(normalX.shape[0])
    faultY = np.ones(faultX.shape[0])

    dataX = np.concatenate((normalX, faultX), axis=0)
    dataY = np.concatenate((normalY, faultY), axis=0)

    return dataX, dataY


df = pd.read_csv("~/catkin_ws/src/network_faults/data/path_data.csv")

# X,Y = create_dataset(df, 3)
# trainX, testX, trainY, testY = train_test_split(X, Y)

# fix random seed for reproducibility
numpy.random.seed(7)
# load the dataset
dataset = df.to_numpy()
# normalize the dataset
scaler = MinMaxScaler(feature_range=(0, 1))
dataset = scaler.fit_transform(dataset)
# reshape into X=t and Y=t+1
look_back = 10
trainX, trainY, testX, testY = create_dataset(dataset, look_back)
# reshape input to be [samples, time steps, features]
trainX = numpy.reshape(trainX, (trainX.shape[0], trainX.shape[1], 1))
trainY = numpy.reshape(trainY, (trainY.shape[0], trainY.shape[1]))
testX = numpy.reshape(testX, (testX.shape[0], testX.shape[1], 1))
testY = numpy.reshape(testY, (testY.shape[0], testY.shape[1]))
print(trainX.shape, trainY.shape)
print(testX.shape, testY.shape)
# create and fit the LSTM network
hidden_dim = 100
nepoch = 40

model = Sequential()
model.add(LSTM(hidden_dim, input_shape=(look_back, 1)))
model.add(Dense(1))
model.compile(loss='mean_squared_error', optimizer='adam')
model.fit(trainX, trainY, epochs=nepoch, batch_size=1, verbose=2)
# make predictions
trainPredict = model.predict(trainX)
testPredict = model.predict(testX)
# invert predictions
trainPredict = scaler.inverse_transform(trainPredict)
trainY = scaler.inverse_transform(trainY)
testPredict = scaler.inverse_transform(testPredict)
testY = scaler.inverse_transform(testY)
# calculate root mean squared error
# trainScore = math.sqrt(mean_squared_error(trainY[0], trainPredict[:,0]))
# print('Train Score: %.2f RMSE' % (trainScore))
# testScore = math.sqrt(mean_squared_error(testY[0], testPredict[:,0]))
# print('Test Score: %.2f RMSE' % (testScore))
# shift train predictions for plotting
trainPredictPlot = numpy.empty_like(dataset)
trainPredictPlot[:, :] = numpy.nan
trainPredictPlot[0:len(trainPredict), :] = trainPredict
# shift test predictions for plotting
testPredictPlot = numpy.empty_like(dataset)
print(testPredictPlot.shape)
testPredictPlot[:, :] = numpy.nan
testPredictPlot[len(testPredictPlot)-5*look_back:len(testPredictPlot), :] = testPredict
# plot baseline and predictions
plt.plot(scaler.inverse_transform(dataset))
plt.plot(trainPredictPlot)
plt.plot(testPredictPlot)
plt.show()
