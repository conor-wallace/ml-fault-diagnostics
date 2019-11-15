#INPUT SHAPE: [number_of_smaples, length_of_sequence, number_of_features]
#OUTPUT SHAPE: [number_of_samples, number_of_features]
import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

#create sine wave data
sin_wave = np.array([math.sin(x) for x in np.arange(200)])
plt.plot(sin_wave[:50])
# plt.show()

#segment data into features and labels
X = []
Y = []

sequence_length = 50
number_samples = len(sin_wave) - sequence_length

for i in range(number_samples):
    #for i = 0, X = sin_wave[0:50], Y = sin_wave[50], i = 0, 1, ..., n
    X.append(sin_wave[i:i+sequence_length])
    Y.append(sin_wave[i+sequence_length])

X = np.array(X)
X = np.expand_dims(X, axis=2)

Y = np.array(Y)
Y = np.expand_dims(Y, axis=1)

X_val = []
Y_val = []

for i in range(number_samples - sequence_length, number_samples):
    X_val.append(sin_wave[i:i+sequence_length])
    Y_val.append(sin_wave[i+sequence_length])

X_val = np.array(X_val)
X_val = np.expand_dims(X_val, axis=2)

Y_val = np.array(Y_val)
Y_val = np.expand_dims(Y_val, axis=1)

print(X.shape, Y.shape)
print(X_val.shape, Y_val.shape)

#architecture of RNN model:
#input layer: 50 units for the input sequence
#hidden layer: 100 units
#output layer: 1 unit

learning_rate = 0.0001
nepoch = 40
T = 50
hidden_dim = 100
output_dim = 1

bptt_truncate = 5
min_clip_value = -10
max_clip_value = 10

#W_i: weight matrix for weights between the input and hidden layer
#W_h: weight matrix for shared weights in the hidden layer (RNN layer)
#W_o: weight matrix for weights between the hidden and the output layer
U = np.random.uniform(0, 1, (hidden_dim, T))
W = np.random.uniform(0, 1, (hidden_dim, hidden_dim))
V = np.random.uniform(0, 1, (output_dim, hidden_dim))

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

# Training:
#     1.1 check loss on training data
#         1.1.1 forward pass
#         1.1.2 calculate error
#     1.2 check loss on validation data
#         1.2.1 forward pass
#         1.2.2 calculate error
#     1.3 start actual training
#         1.3.1 forward pass
#         1.3.2 backpropagate error
#         1.3.3 update weights

for epoch in range(nepoch):
    # 1.1 check loss on training data
    loss = 0.0
    # 1.1.1 forward pass
    for i in range(Y.shape[0]):
        x, y = X[i], Y[i]

        #prev_state is the value of the activation function at time t-1 initialized to zeros
        prev_state = np.zeros((hidden_dim, 1))
        for t in range(T):
            new_input = np.zeros(x.shape)
            #forward pass for every sample in this sequence
            new_input[t] = x[t]
            mulu = np.dot(U, new_input)
            mulw = np.dot(W, prev_state)
            add = mulu + mulw
            s = sigmoid(add)
            mulv = np.dot(V, s)
            prev_state = s

        # 1.1.2 calculate error
        loss_per_sample = (y - mulv)**2/2
        loss += loss_per_sample
    loss = loss / float(y.shape[0])

    # 1.2 check loss on validation data
    val_loss = 0.0
    # 1.2.1 forward pass
    for i in range(Y_val.shape[0]):
        x, y = X_val[i], Y_val[i]
        prev_state = np.zeros((hidden_dim, 1))
        for t in range(T):
            new_input = np.zeros(x.shape)
            new_input[t] = x[t]
            mulu = np.dot(U, new_input)
            mulw = np.dot(W, prev_state)
            add = mulu + mulw
            s = sigmoid(add)
            mulv = np.dot(V, s)
            prev_state = s

        # 1.2.2 calculate error
        val_loss_per_sample = (y - mulv)**2 / 2
        val_loss += val_loss_per_sample
    val_loss = val_loss / float(y.shape[0])

    print('Epoch: ', epoch + 1, ', Loss: ', loss, ', Val Loss: ', val_loss)

    # 1.3 start actual training
    for i in range(Y.shape[0]):
        x, y = X[i], Y[i]

        #prev_state is the value of the activation function at time t-1 initialized to zeros
        prev_state = np.zeros((hidden_dim, 1))
        layers = []
        dU = np.zeros(U.shape)
        dW = np.zeros(W.shape)
        dV = np.zeros(V.shape)

        dU_t = np.zeros(U.shape)
        dW_t = np.zeros(W.shape)
        dV_t = np.zeros(V.shape)

        dU_i = np.zeros(U.shape)
        dW_i = np.zeros(W.shape)

        # 1.3.1 forward pass
        for t in range(T):
            new_input = np.zeros(x.shape)
            new_input[t] = x[t]
            mulu = np.dot(U, new_input)
            mulw = np.dot(W, prev_state)
            add = mulu + mulw
            s = sigmoid(add)
            mulv = np.dot(V, s)
            layers.append({'s':s, 'prev_state':prev_state})
            prev_state = s

        #derivative of prediction
        dmulv = (mulv - y)

        # 1.3.2 backpropagate error
        for t in range(T):
            dV_t = np.dot(dmulv, np.transpose(layers[t]['s']))
            dsv = np.dot(np.transpose(V), dmulv)

            ds = dsv
            dadd = add * (1 - add) * ds
            dmulw = dadd * np.ones_like(mulw)
            dprev_state = np.dot(np.transpose(W), dmulw)

            for i in range(t-1, max(-1, t-bptt_truncate-1), -1):
                ds = dsv + dprev_state
                dadd = add * (1 - add) * ds

                dmulw = dadd * np.ones_like(mulw)
                dmulu = dadd * np.ones_like(mulu)

                dW_i = np.dot(W, layers[t]['s'])
                dprev_state = np.dot(np.transpose(W), dmulw)

                new_input = np.zeros(x.shape)
                new_input[t] = x[t]
                dU_i = np.dot(U, new_input)
                dx = np.dot(np.transpose(U), dmulu)

                dU_t += dU_i
                dW_t += dW_i

            dV += dV_t
            dW += dW_t
            dU += dU_t

            if dU.max() > max_clip_value:
                dU[dU > max_clip_value] = max_clip_value
            if dV.max() > max_clip_value:
                dV[dV > max_clip_value] = max_clip_value
            if dW.max() > max_clip_value:
                dW[dW > max_clip_value] = max_clip_value

            if dU.min() < min_clip_value:
                dU[dU < min_clip_value] = min_clip_value
            if dV.min() < min_clip_value:
                dV[dV < min_clip_value] = min_clip_value
            if dW.min() < min_clip_value:
                dW[dW < min_clip_value] = min_clip_value

        # 1.3.3 update weights
        U -= learning_rate * dU
        W -= learning_rate * dW
        V -= learning_rate * dV

preds = []
for i in range(Y.shape[0]):
    x, y = X[i], Y[i]
    prev_s = np.zeros((hidden_dim, 1))
    # Forward pass
    for t in range(T):
        mulu = np.dot(U, x)
        mulw = np.dot(W, prev_s)
        add = mulw + mulu
        s = sigmoid(add)
        mulv = np.dot(V, s)
        prev_s = s

    preds.append(mulv)

preds = np.array(preds)
print(preds.shape)
plt.plot(preds[:, 0, 0], 'b', label='Predictions')
plt.plot(Y[:, 0], 'r', label='Labels')
plt.title('training')
plt.legend()
plt.show()

preds = []
for i in range(Y_val.shape[0]):
    x, y = X_val[i], Y_val[i]
    prev_s = np.zeros((hidden_dim, 1))
    # For each time step...
    for t in range(T):
        mulu = np.dot(U, x)
        mulw = np.dot(W, prev_s)
        add = mulw + mulu
        s = sigmoid(add)
        mulv = np.dot(V, s)
        prev_s = s

    preds.append(mulv)

preds = np.array(preds)
print(preds.shape)
plt.plot(preds[:, 0, 0], 'b', label='Predictions')
plt.plot(Y_val[:, 0], 'r', label='Labels')
plt.title('validation')
plt.legend()
plt.show()

rmse = math.sqrt(mean_squared_error(Y_val[:, 0], preds[:, 0, 0]))
print(rmse)
