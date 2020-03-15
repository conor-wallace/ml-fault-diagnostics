#!/usr/bin/env python

import rospy
import math
import sys
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from bicycle import Bicycle
from unity_controller.msg import AgentVelocity, AgentPose, Velocity
import subprocess, re
from kafka import KafkaProducer
from json import dumps
from time import sleep
import pandas as pd
from sklearn.preprocessing import MinMaxScaler
from sklearn import svm
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis, QuadraticDiscriminantAnalysis
from sklearn.model_selection import train_test_split
import tensorflow as tf
from keras import optimizers, Sequential
from keras.models import Model
from keras.layers import Dense, CuDNNLSTM
from keras.models import model_from_yaml
from pid import PID
import timeit

# TODO:
# a.) convert this to ROS and Unity
# b.) get network data node working
# c.) record new data set
# d.) connect simulation to cloud

ideal_pose = None
faulty_pose = None
residual = np.empty(3)
target = np.empty(2)
fault = 0

def trainAlternativeModel():
    path = '../data/path_data.csv'
    df = pd.read_csv(path)
    dataset = df[['%x', '%y', '%theta', '%velocity', '%steering', '%iteration', '%label']]
    dataset = dataset.to_numpy()
    data = dataset[:, :-1]
    print(data.shape)
    labels = np.reshape(dataset[:, -1], (-1, 1))

    scaler = MinMaxScaler(feature_range=(-1, 1))
    normalized_data = scaler.fit_transform(data)
    X_train, X_test, y_train, y_test = train_test_split(normalized_data, labels, test_size=0.33, random_state=42)

    model = QuadraticDiscriminantAnalysis()
    # model = LinearDiscriminantAnalysis()
    # model = KNeighborsClassifier()
    # model = LogisticRegression()
    # model = svm.SVC()
    model.fit(X_train, y_train)

    print("Trained Model Ready")

    return model

def loadScaler():
    path = '../data/path_data.csv'
    df = pd.read_csv(path)
    dataset = df[['%x','%y','%theta','%velocity','%steering','%iteration']]
    dataset = dataset.to_numpy()

    scaler = MinMaxScaler(feature_range=(-1, 1))
    dataset = scaler.fit_transform(dataset)

    return scaler

def loadModel(model_path, weights_path):
    yaml_file = open(model_path, 'r')
    loaded_model_yaml = yaml_file.read()
    yaml_file.close()
    model = model_from_yaml(loaded_model_yaml)
    # load weights into new model
    model.load_weights(weights_path)
    print("Loaded model from disk")

    # evaluate loaded model on test data
    model.compile(optimizer='Adam', loss='categorical_crossentropy', metrics=['accuracy'])

    return model

def poseCallback(msg):
    global ideal_pose, faulty_pose, residual, fault, target
    # print(msg)
    ideal_pose = msg.agents[0]
    faulty_pose = msg.agents[1]
    fault = faulty_pose.orientation.x
    residual[0] = faulty_pose.position.x
    residual[1] = faulty_pose.position.y
    residual[2] = faulty_pose.orientation.z
    target[0] = faulty_pose.orientation.y
    target[1] = faulty_pose.orientation.w

def processData(pid, scaler, model, model_class):
    global ideal_pose, faulty_pose, fault, target
    # initialize node
    rospy.init_node('fault_data', anonymous = True)

    labels = ["ideal", "no fault", "left fault", "right fault"]
    rospy.Subscriber('agent_poses', AgentPose, poseCallback, queue_size=1, tcp_nodelay=True)
    velocity_publisher = rospy.Publisher('agent_velocities', AgentVelocity, queue_size=1, tcp_nodelay=True)
    rate = rospy.Rate(10) # 10hz
    msg = AgentVelocity()
    ideal_velocity = Velocity()
    faulty_velocity = Velocity()
    msg.agent_velocities = [ideal_velocity, faulty_velocity]

    previous_time = rospy.get_time()
    time = 0.0
    start_time = 0.0
    iteration = 0
    num_features = 7
    feature_vector = np.empty(num_features)
    training_data = []
    timesteps = 10
    num_features = 6
    data = np.empty((timesteps, num_features))
    sample_count = 0
    true_positive = 0
    runtimes = []

    while not rospy.is_shutdown():
        if ideal_pose is not None:
            if ideal_pose.position.x == 123456789:
                ideal_velocity.velocity, ideal_velocity.steering = 0.0, 0.0
                faulty_velocity.velocity, faulty_velocity.steering = 0.0, 0.0
                velocity_publisher.publish(msg)
                rate.sleep()
                runtimes = np.array(runtimes)
                # f = open('../data/runtime.csv', 'a')
                # np.savetxt(f, runtimes, delimiter=",")
                sys.exit(1)
            # Compute PID
            time = rospy.get_time() - start_time
            dt = rospy.get_time() - previous_time

            ideal_distance, ideal_heading = pid.calculateError(target, ideal_pose.position.x, ideal_pose.position.y, ideal_pose.orientation.z)
            pid.calculatePID(ideal_distance, ideal_heading, dt)
            ideal_velocity.velocity, ideal_velocity.steering = pid.velocity, pid.steering
            faulty_distance, faulty_heading = pid.calculateError(target, faulty_pose.position.x, faulty_pose.position.y, faulty_pose.orientation.z)
            pid.calculatePID(faulty_distance, faulty_heading, dt)
            faulty_velocity.velocity, faulty_velocity.steering = pid.velocity, pid.steering

            feature_vector = np.array([float(residual[0]), float(residual[1]), float(residual[2]), float(faulty_velocity.velocity), float(faulty_velocity.steering), float(iteration)])
            feature_vector = np.reshape(feature_vector, (1, num_features))

            features = scaler.transform(feature_vector)
            label = float(faulty_pose.orientation.x)

            if model_class != 0:
                start_time1 = timeit.default_timer()
                y_hat = model.predict(features)
                elapsed1 = timeit.default_timer() - start_time1
                elapsed1 = round((elapsed1*1000.0), 5)
                print('Finished in %s second(s)' % elapsed1)
                runtimes.append([elapsed1, model_class])
                sample_count += 1
                # print(y_hat)
                if np.argmax(y_hat) == 0:
                    print("Predicted: Healthy")
                    if label == 0.0:
                        true_positive += 1
                elif np.argmax(y_hat) == 1:
                    print("Predicted: Left Fault")
                    if label == 1.0:
                        true_positive += 1
                else:
                    print("Predicted: Right Fault")
                    if label == 2.0:
                        true_positive += 1

                if true_positive > 0:
                    accuracy = float(true_positive)/float(sample_count) * 100
                    print("Prediction Accuracy: %s" % accuracy)
            else:
                if np.isnan(np.sum(data)):
                    data = np.delete(data, 0, 0)
                    data = np.append(data, features, axis=0)
                else:
                    data = np.delete(data, 0, 0)
                    data = np.append(data, features, axis=0)
                    #make predictions
                    start_time1 = timeit.default_timer()
                    y_hat = model.predict(x=np.reshape(data, (1, timesteps, num_features)))
                    elapsed1 = timeit.default_timer() - start_time1
                    elapsed1 = round((elapsed1*1000.0), 5)
                    print('Finished in %s second(s)' % elapsed1)
                    runtimes.append([elapsed1, model_class])
                    sample_count += 1
                    # print(y_hat)
                    if np.argmax(y_hat) == 0:
                        print("Predicted: Healthy")
                        if label == 0.0:
                            true_positive += 1
                    elif np.argmax(y_hat) == 1:
                        print("Predicted: Left Fault")
                        if label == 1.0:
                            true_positive += 1
                    else:
                        print("Predicted: Right Fault")
                        if label == 2.0:
                            true_positive += 1

                    if true_positive > 0:
                        accuracy = float(true_positive)/float(sample_count) * 100
                        print("Prediction Accuracy: %s" % accuracy)

            previous_time = rospy.get_time()
            iteration += 1
        else:
            start_time = rospy.get_time()

        velocity_publisher.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    # Create a MinMaxScaler
    scaler = loadScaler()

    # # Load LSTM Model
    # model_path = '../data/model1.yaml'
    # weights_path = '../data/model1.h5'
    # model = loadModel(model_path, weights_path)

    # Alternative Model
    model = trainAlternativeModel()

    # Get PID models
    fault = 0
    pid = PID(fault)

    model_class = 5
    # Process Data
    processData(pid, scaler, model, model_class)

    # data = np.concatenate((path1_error, path2_error, path3_error), axis=0)
    # # print(data)
    # # f = open('/home/conor/catkin_ws/src/unity_controller/data/sim_data.csv', 'a')
    # # np.savetxt(f, data, delimiter=",")
