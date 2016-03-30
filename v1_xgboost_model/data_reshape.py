#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Reshape extracted data files to format acceptable by xgboost
"""

import os

# [1, 501) acc_y.txt.train
# [601, 700) acc_y.txt.test

dp_start = 601
dp_end = 700
fname = 'acc_y.txt.test'

if __name__ == '__main__':
  training_examples_dir = '../../contact_dynamics_old/training_examples/'
  if not os.path.exists('./training_data'):
    os.mkdir('./training_data')
  data_i = dp_start
  data_points_end = dp_end
  with open(os.path.join('./training_data', fname), 'w') as w:
    while os.path.exists(os.path.join(training_examples_dir, 'ex%d.txt' % data_i)):
      with open(os.path.join(training_examples_dir, 'ex%d.txt' % data_i), 'r') as f:
        for l in f:
          if not l.strip():
            continue
          data_list = l.split(', ')[:-1]
          w.write(' '.join([str(data_list[-2])] + ['%d:%s' % (i + 1, data_list[i]) for i in xrange(0, 21)]) + '\n')
      data_i += 1
      if data_i >= data_points_end:
        break
