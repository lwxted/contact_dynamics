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

param = {'max_depth' : 8, 'eta' : 0.05, 'objective' : 'reg:linear'}

watchlist = [(dtest, 'eval'), (dtrain, 'train')]
num_rounds = 200
bst = xgb.train(param, dtrain, num_rounds, watchlist)

preds = bst.predict(dtest)
labels = dtest.get_label()

bst.save_model('{0}-{1}-{2}.model'.format(param['max_depth'], param['eta'], num_rounds))
bst.dump_model('{0}-{1}-{2}.raw.txt'.format(param['max_depth'], param['eta'], num_rounds))
