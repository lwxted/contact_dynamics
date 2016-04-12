#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Basic xgboost model that attempts to predict values of acceleration
"""

import os
import xgboost as xgb

base_path = os.path.dirname(os.path.realpath(__file__))
print base_path

train_file = os.path.join(base_path, 'training_data', 'contact_mode_label.train')
test_file = os.path.join(base_path, 'training_data', 'contact_mode_label.test')

dtrain = xgb.DMatrix(train_file)
dtest = xgb.DMatrix(test_file)

param = {'max_depth' : 5, 'eta' : 0.1, 'objective' : 'multi:softmax', 'tree_method' : 'exact'}

watchlist = [(dtest, 'eval'), (dtrain, 'train')]
num_rounds = 50
bst = xgb.train(param, dtrain, num_rounds, watchlist)

preds = bst.predict(dtest)
labels = dtest.get_label()

bst.save_model('{0}-{1}-{2}-label.model'.format(param['max_depth'], param['eta'], num_rounds))
bst.dump_model('{0}-{1}-{2}-label.raw.txt'.format(param['max_depth'], param['eta'], num_rounds))
