import random
from collections import deque

import numpy as np
import pylab
from keras.layers import LSTM, Dense
from keras.models import Sequential
from keras.optimizers import Adam

import tensorflow_model_optimization as tfmot
import tensorflow as tf

# DQN Agent for the Cartpole
# it uses Neural Network to approximate q function
# and replay memory & target q network
class DQNAgent_quant:
    def __init__(self, model_path='./cartpole_dqn.tflite'):
        # if you want to see Cartpole learning, then change to True
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

    # get action from model using epsilon-greedy policy
    def get_action(self, state):
        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        # process the input state for tflite interpreter
        input_shape = input_details[0]['shape']
        input_data = np.array(state, dtype=np.float32)

        self.interpreter.set_tensor(input_details[0]['index'], input_data)
        self.interpreter.invoke()

        output_data = self.interpreter.get_tensor(output_details[0]['index'])
        return np.argmax(output_data[0])

class DoubleDQNAgent_quant:
    def __init__(self, model_path='./cartpole_ddqn.tflite'):
        # if you want to see Cartpole learning, then change to True
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

    # get action from model using epsilon-greedy policy
    def get_action(self, state):
        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        # process the input state for tflite interpreter
        input_shape = input_details[0]['shape']
        input_data = np.array(state, dtype=np.float32)

        self.interpreter.set_tensor(input_details[0]['index'], input_data)
        self.interpreter.invoke()

        output_data = self.interpreter.get_tensor(output_details[0]['index'])
        return np.argmax(output_data[0])

