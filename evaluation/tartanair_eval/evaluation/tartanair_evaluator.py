# Copyright (c) 2020 Carnegie Mellon University, Wenshan Wang <wenshanw@andrew.cmu.edu>
# For License information please see the LICENSE file in the root directory.

import numpy as np
from evaluator_base import ATEEvaluator, RPEEvaluator, KittiEvaluator, transform_trajs, quats2SEs
from evaluate_ate_scale import plot_traj, plot_ate
from os.path import isdir, isfile
from trajectory_transform import cam2ned, kitti2tartan
import sys

# from trajectory_transform import timestamp_associate

class TartanAirEvaluator:
    def __init__(self, scale = False, round=1):
        self.ate_eval = ATEEvaluator()
        self.rpe_eval = RPEEvaluator()
        self.kitti_eval = KittiEvaluator()
    
    def evaluate_one_trajectory(self, gt_traj_name, est_traj_name, scale=False):
        """
        scale = True: calculate a global scale
        """
        # load trajectories
        gt_traj = np.loadtxt(gt_traj_name)
        est_traj = np.loadtxt(est_traj_name)
        #est_traj_new = est_traj;
        #gt_traj_new = gt_traj[0:len(est_traj_new)];

        gt_traj_new = [];
        est_traj_new = [];

        for index in range(len(est_traj)):
            # ignore frame 0
            if(index > 0):
                traj = est_traj[index];
                row = int(traj[0]);
                gt = gt_traj[row];
                gt_traj_new.append(gt);
                est_traj_new.append(traj[1:]);

        est_traj_new = np.asarray(est_traj_new);
        gt_traj_new = np.asarray(gt_traj_new);

        est_traj_new = kitti2tartan(est_traj_new);

        if gt_traj_new.shape[0] != est_traj_new.shape[0]:
            raise Exception("POSEFILE_LENGTH_ILLEGAL")
        if gt_traj_new.shape[1] != 7 or est_traj_new.shape[1] != 7:
            raise Exception("POSEFILE_FORMAT_ILLEGAL")

        # transform and scale
        gt_traj_trans, est_traj_trans, s = transform_trajs(gt_traj_new, est_traj_new, scale)

        gt_SEs, est_SEs = quats2SEs(gt_traj_trans, est_traj_trans)
        
        ate_score, gt_ate_aligned, est_ate_aligned = self.ate_eval.evaluate(gt_traj_new, est_traj_new, scale)

        print(est_ate_aligned)
        plot_ate(gt_ate_aligned, est_ate_aligned, vis=False, savefigname='results.png', title='ATE %.4f' %(ate_score))

        print(gt_SEs);
        print("*******************************************");
        print(est_SEs);

        rpe_score = self.rpe_eval.evaluate(gt_SEs, est_SEs)
        kitti_score = self.kitti_eval.evaluate(gt_SEs, est_SEs)

        return {'ate_score': ate_score, 
                'rpe_score': rpe_score, 
                'kitti_score': kitti_score}

if __name__ == "__main__":
    
    # scale = True for monocular track, scale = False for stereo track
    aicrowd_evaluator = TartanAirEvaluator()

    result = aicrowd_evaluator.evaluate_one_trajectory(sys.argv[1], sys.argv[2], scale=True)
    print(result)
