#!/bin/bash
mkdir -p $2
/home/ORB_SLAM3/build/mono_tartanair /home/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ORB_SLAM3/Examples/Monocular/TartanAir.yaml $1 > $2/mono_orbslam3_tartan_out.txt
python3 /home/mvSLAM/evaluation/tartanair_eval/evaluation/tartanair_evaluator.py $1/pose_left.txt TrajectoryKITTIKeyFrame.txt > $2/mono_orbslam3_tartan_eval.txt
mv results.txt $2/
mv TrajectoryKITTIKeyFrame.txt $2/
