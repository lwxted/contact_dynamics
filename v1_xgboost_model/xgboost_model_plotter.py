#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import xgboost as xgb
import matplotlib.pyplot as plt

base_path = os.path.dirname(os.path.realpath(__file__))

# bst = xgb.Booster({'nthread' : 4})
# bst.load_model('./models_label/5-0.1-50-label.model')
# xgb.plot_tree(bst, num_trees=2)

# with open(os.path.join(base_path, '../training_examples', 'ex800.txt'), 'r') as f:
#   with open(os.path.join(base_path, 'predict_std.txt'), 'w') as w:
#     for l in f:
#       if not l.strip():
#         continue
#       data_list = l.strip().split(', ')
#       w.write(str(float(data_list[-1])))
#       w.write('\n')

std = []
res = []

with open(os.path.join(base_path, 'predict_std.txt'), 'r') as f1:
  for l in f1:
    if not l.strip():
      continue
    std.append(float(l.strip()))

with open(os.path.join(base_path, 'predict_res.txt'), 'r') as f2:
  for l in f2:
    if not l.strip():
      continue
    res.append(float(l.strip()))

plt.plot(std[3300:3500], color='blue')
plt.plot(res[3300:3500], color='red')
plt.show()
