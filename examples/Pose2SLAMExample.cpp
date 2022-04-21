/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample.cpp
 * @brief A 2D Pose SLAM example
 * @date Oct 21, 2010
 * @author Yong Dian Jian
 * 在这个示例中是自己手动给了因子图一个闭环约束，以便提高优化后的准确率，在实际应用中需要机器人自己找到闭环约束点
 */

/**
* 一个简单的 2D 姿势大满贯示例
* 机器人在 2 平方米范围内移动
* 机器人每步移动 2 米，每次移动后转 90 度
* 机器人最初面向 X 轴（水平向右）
* 我们在姿势之间有完整的里程计数据
* 当机器人返回第一个位置时，我们有一个闭环约束
*/

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses
// 位姿描述类
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
// values存放结果中的索引
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
// 二元因子类
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
// 非线性图
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
// 高斯牛顿优化器
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
// 计算每个节点的后验概率
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
// 存放输出结果与预测值
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // 1. Create a factor graph container and add factors to it
  // 创建图
  NonlinearFactorGraph graph;

  // 2a. Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  // 添加初始位置（先验因子）
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, Pose2(0, 0, 0), priorNoise);

  // For simplicity, we will use the same noise model for odometry and loop closures
  // 对于闭环因子和二元因子使用相同的噪声
  auto model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // 2b. Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  // 添加里程计二元因子 每次载入的Pose是相对上一节点的变化位姿 这里每次移动后还需要旋转90度三次旋转后就会形成环
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, 0), model); // 在这个点之前还有一个初始位置
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(4, 5, Pose2(2, 0, M_PI_2), model);

  // 2c.添加闭环约束
  // 这个因子指明了我们已经返回到与之前相同位姿的事实。在真实系统中，这些约束可以通过多种方式识别，例如基于外观的技术
  // 带有相机图像。我们将使用另一个 Between Factor 来强制执行此约束：
  // 这里与之前插入二元边的唯一区别在于插入点的编号不是连续的（5， 2）
  graph.emplace_shared<BetweenFactor<Pose2> >(5, 2, Pose2(2, 0, M_PI_2), model);
  graph.print("\nFactor Graph:\n");  // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  // 创建预测值容器以便接收预测结果
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI_2));
  initialEstimate.insert(4, Pose2(4.0, 2.0, M_PI));
  initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\nInitial Estimate:\n");  // print

  // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
  // The optimizer accepts an optional set of configuration parameters,
  // controlling things like convergence criteria, the type of linear
  // system solver to use, and the amount of information displayed during
  // optimization. We will set a few parameters as a demonstration.
  // 创建高斯牛顿优化器 这里还针对优化器做了一些参数配置
  GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this value
  // 设置容差范围
  parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  // 设置迭代次数
  parameters.maxIterations = 100;
  // Create the optimizer ...
  // 使用配置的参数，因子图以及预测值初始化优化器
  GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  // ... and optimize
  // 开始优化
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  // 计算后验估计
  cout.precision(3);
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
  cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;

  return 0;
}
