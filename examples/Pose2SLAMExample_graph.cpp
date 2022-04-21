/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_graph.cpp
 * @brief Read graph from file and perform GraphSLAM 
 * @date June 3, 2012
 * @author Frank Dellaert
 * 从文件中读取因子图病执行GraphSLAM
 */

// For an explanation of headers below, please see Pose2SLAMExample.cpp
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

// This new header allows us to read examples easily from .graph files
#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

int main (int argc, char** argv) {

  // Read File, create graph and initial estimate
  // we are in build/examples, data is in examples/Data
  NonlinearFactorGraph::shared_ptr graph; // 创建因子图
  Values::shared_ptr initial; // 创建初始预测值
  // 创建噪声
  SharedDiagonal model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 5.0 * M_PI / 180.0).finished());
  // 读取图
  string graph_file = findExampleDataFile("w100.graph");
  boost::tie(graph, initial) = load2D(graph_file, model); // 读取的时候需要使用上面的噪声
  initial->print("Initial estimate:\n"); // 打印初始预测值

  // Add a Gaussian prior on first poses
  Pose2 priorMean(0.0, 0.0, 0.0); // prior at origin
  SharedDiagonal priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01)); 
  graph -> addPrior(0, priorMean, priorNoise); // 添加先验信息

  // Single Step Optimization using Levenberg-Marquardt 使用LM优化器
  Values result = LevenbergMarquardtOptimizer(*graph, *initial).optimize();
  result.print("\nFinal result:\n");

  // Plot the covariance of the last pose
  // 计算最后一个点的后验估计协方差
  Marginals marginals(*graph, result);
  cout.precision(2);
  cout << "\nP3:\n" << marginals.marginalCovariance(99) << endl;

  return 0;
}
