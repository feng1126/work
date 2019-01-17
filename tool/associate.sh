#!/bin/sh



python ./tool/evaluate_ate_scale/evaluate_ate.py ./gps.txt ./orb.txt  --save_associations 1.log
python ./tool/orb.py ./1.log associated




