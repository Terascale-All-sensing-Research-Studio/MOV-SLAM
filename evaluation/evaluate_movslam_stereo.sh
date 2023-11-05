#!/bin/bash
mkdir -p $2
/home/mvSLAM/build/stereo_video_tartan /home/mvSLAM/Examples/Stereo/TartanAir.yaml tcp://192.168.4.217:8000 > $2/stereo_video_tartan_out.txt
python3 /home/mvSLAM/evaluation/tartanair_eval/evaluation/tartanair_evaluator.py $1/pose_left.txt TrajectoryKITTIKeyFrame.txt > $2/stereo_video_tartan_eval.txt
mv results.txt $2/
mv TrajectoryKITTIKeyFrame.txt $2/
