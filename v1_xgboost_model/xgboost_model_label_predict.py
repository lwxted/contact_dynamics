#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import xgboost as xgb

base_path = os.path.dirname(os.path.realpath(__file__))

predict_file = os.path.join(base_path, 'training_data', 'contact_mode_label.predict')

dpredict = xgb.DMatrix(predict_file)

bst = xgb.Booster({'nthread' : 4})
bst.load_model('./8-0.1-20-label.model')

label_pred = bst.predict(dpredict)
with open(os.path.join(base_path, 'predict_res.txt'), 'w') as f:
  for item in label_pred:
    f.write(str(round(item)))
    f.write('\n')
