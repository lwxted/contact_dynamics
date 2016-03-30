#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Basic xgboost model that attempts to predict values of acceleration
"""

import os
import xgboost as xgb

base_path = os.path.dirname(os.path.realpath(__file__))
print base_path

train_file = os.path.join(base_path, 'training_data', 'acc_y.txt.train')
test_file = os.path.join(base_path, 'training_data', 'acc_y.txt.test')

dtrain = xgb.DMatrix(train_file)
dtest = xgb.DMatrix(test_file)

param = {'max_depth' : 100, 'eta' : 0.5, 'objective' : 'reg:linear'}

watchlist = [(dtest, 'eval'), (dtrain, 'train')]
num_rounds = 20
bst = xgb.train(param, dtrain, num_rounds, watchlist)

preds = bst.predict(dtest)
labels = dtest.get_label()

bst.save_model('0001.model')
# dump model
bst.dump_model('dump.raw.txt')
