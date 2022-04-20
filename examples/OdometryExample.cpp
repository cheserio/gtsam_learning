/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file OdometryExample.cpp
 * @brief Simple robot motion example, with prior and two odometry measurements
 * @author Frank Dellaert
 * 具有先验信息以及两个里程计测量值
 */

/**
 * Example of a simple 2D localization example
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 *  机器人初始位姿朝向x轴的正方向，每次移动2米，每次移动都有里程计的测量值
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
// 机器人的状态量为 x y theta（朝向）
#include <gtsam/geometry/Pose2.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
// 在GTSAM中测量值被表述成“因子” 常见的因子（测量值）类型被封装到了库中以便解决机器人/slam/光束平差等问题
// 在这个例子中将使用二元因子来表述里程计的测量值以及使用先验因子来表述机器人的起始位置
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
// 当我们一旦创建了因子我们就需要将其添加到因子图中，如果使用的是非线性的因子我们就需要使用非线性的因子图
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
// 最后将所有因子添加到因子图中后我们就要使用优化器来找最优解（得到最大后验估计）GTSAM提供了几种非线性优化器
// 这里使用的是LM优化器
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
// 一旦这些优化后的值被计算出来，我们就可以计算这些待求值的边界协方差（marginal covariance）
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
// 下面这个头文件提供了一个接收结果的容器，这个容器一开始存放的是迭代优化的预测值
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // 创建一个空的非线性因子图
  NonlinearFactorGraph graph;

  // 在第一个位姿上添加先验值，将其设置为原点
  // 先验因子由均值和噪声模型（协方差矩阵）组成 Pose2是2D位姿的意思 x, y , theta
  Pose2 priorMean(0.0, 0.0, 0.0);  // prior at origin
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, priorMean, priorNoise);

  // 添加里程计因子（二元因子） 每次向x正方向移动2m
  Pose2 odometry(2.0, 0.0, 0.0);
  // 为简单起见，我们将对每个里程计因子使用相同的噪声模型
  auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  // 在连续位姿之间创建里程计（Between）因子
  // 1号位姿和2号位姿之间添加测量值（因子）
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, odometry, odometryNoise);
  // 2号位姿和3号位姿之间添加测量值（因子）
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, odometry, odometryNoise);
  graph.print("\nFactor Graph:\n");  // print

  // Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  // 创建一个用于接收结果的容器，为了显示gtsam的作用故意将这里的预测位姿设置成错误的
  Values initial;
  initial.insert(1, Pose2(0.5, 0.0, 0.2));
  initial.insert(2, Pose2(2.3, 0.1, -0.2));
  initial.insert(3, Pose2(4.1, 0.1, 0.1));
  initial.print("\nInitial Estimate:\n");  // print

  // 使用 Levenberg-Marquardt 优化进行优化 需要传入因子图以及预测值
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final Result:\n");

  // Calculate and print marginal covariances for all variables
  cout.precision(2);
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}
