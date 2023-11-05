/**
 * This file is part of MOV-SLAM
 *
 * Copyright (C) 2022
 *
 * MOV-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MOV-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with MOV-SLAM.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

#include <math.h>

#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

namespace MOV_SLAM
{
    typedef std::pair<set<KeyFrame *>, int> ConsistentGroup;
    typedef map<KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
                Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3>>>
        KeyFrameAndPose;

    class Optimizer
    {
    public:
        void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP,
                                     int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);
        void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                           const unsigned long nLoopKF = 0, const bool bRobust = true);
        void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &num_fixedKF, int &num_OptKF, int &num_MPs, int &num_edges);

        static int PoseOptimization(Frame *pFrame, const bool isLost, const int iterationCount = 50, const double reprojectionError = 5.0, const double reprojectErrorLost = 8.0, const double confidence = 0.95, const int algorithm = 38);

        void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace MOV_SLAM

#endif // OPTIMIZER_H
