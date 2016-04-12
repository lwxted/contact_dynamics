#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Reshape extracted data files to format acceptable by xgboost
"""

import os

# [1, 400] contact_mode_label.train
# [401, 800] contact_mode_label.test

dp_start = 800
dp_end = 801
fname = 'contact_mode_label.predict'

if __name__ == '__main__':
  training_examples_dir = '../training_examples/'
  if not os.path.exists('./training_data'):
    os.mkdir('./training_data')
  data_i = dp_start
  data_points_end = dp_end
  with open(os.path.join('./training_data', fname), 'w') as w:
    while os.path.exists(os.path.join(training_examples_dir, 'ex%d.txt' % data_i)):
      with open(os.path.join(training_examples_dir, 'ex%d.txt' % data_i), 'r') as f:
        print os.path.join(training_examples_dir, 'ex%d.txt' % data_i)
        for l in f:
          if not l.strip():
            continue
          data_list = l.strip().split(', ')
          w.write(' '.join(['%d:%s' % (i + 1, data_list[i]) for i in xrange(0, 21)]) + '\n')
      data_i += 1
      if data_i >= data_points_end:
        break
