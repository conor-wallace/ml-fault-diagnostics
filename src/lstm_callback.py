from __future__ import absolute_import, division, print_function, unicode_literals

import tensorflow as tf

import datetime

class CustomCallback(tf.keras.callbacks.Callback):
  def on_train_batch_begin(self, batch, logs=None):
    print('working')
  #
  # def on_train_batch_end(self, batch, logs=None):
  #   print('Training: batch {} ends at {}'.format(batch, datetime.datetime.now().time()))
  #
  # def on_test_batch_begin(self, batch, logs=None):
  #   print('Evaluating: batch {} begins at {}'.format(batch, datetime.datetime.now().time()))
  #
  # def on_test_batch_end(self, batch, logs=None):
  #   print('Evaluating: batch {} ends at {}'.format(batch, datetime.datetime.now().time()))