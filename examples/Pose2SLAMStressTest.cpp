/**
 * @file Pose2SLAMStressTest.cpp
 * @brief Test GTSAM on large open-loop chains
 * @date May 23, 2018
 * @author Wenqiang Zhou
 * gtsam在大型非闭环链上的测试， 区别在于点数变多了， 使用时需要加上创建的点的个数./Pose2SLAMStressTest 10 
 */

// Create N 3D poses, add relative motion between each consecutive poses. (The
// relative motion is simply a unit translation(1, 0, 0), no rotation). For each
// each pose, add some random noise to the x value of the translation part.
// Use gtsam to create a prior factor for the first pose and N-1 between factors
// and run optimization.
// 创建N个3D位姿每次只在x的正方向上移动1m没有旋转，每次输入的距离加上一定的噪声

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h> // 3D位姿态
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <random>

using namespace std;
using namespace gtsam;

// 输入节点个数
void testGtsam(int numberNodes) {
  std::random_device rd; // 创建随机数种子
  std::mt19937 e2(rd()); // 创建随机数 范围在 -maxInt , maxInt
  std::uniform_real_distribution<> dist(0, 1); // 0 - 1之间的随机分布

  vector<Pose3> poses; // 创建3D位姿
  // 创建N个节点 给这些节点先添加噪声 这是初始预测值
  for (int i = 0; i < numberNodes; ++i) {
    Matrix4 M;
    double r = dist(e2); // 随机x方向上的噪声
    r = (r - 0.5) / 10 + i;
    M << 1, 0, 0, r, 
         0, 1, 0, 0, 
         0, 0, 1, 0, 
         0, 0, 0, 1; 
    poses.push_back(Pose3(M));
  }

  // prior factor for the first pose
  // 添加先验信息（初始点） 3D位姿需要使用变换矩阵来构建 一开始 xyz都是0
  auto priorModel = noiseModel::Isotropic::Variance(6, 1e-4);
  Matrix4 first_M;
  first_M << 1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, 1, 0, 
             0, 0, 0, 1;
  Pose3 first = Pose3(first_M); // 创建节点（包含位置旋转信息）

  NonlinearFactorGraph graph; // 创建因子图
  graph.addPrior(0, first, priorModel); // 添加因子

  // vo noise model
  auto VOCovarianceModel = noiseModel::Isotropic::Variance(6, 1e-3);

  // relative VO motion
  Matrix4 vo_M;
  vo_M << 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // 创建节点每个节点的位置信息是相对变换信息 x正方向移动1m
  Pose3 relativeMotion(vo_M);
  // 添加二元因子 相对运动 运动噪声
  for (int i = 0; i < numberNodes - 1; ++i) {
    graph.add(
        BetweenFactor<Pose3>(i, i + 1, relativeMotion, VOCovarianceModel));
  }

  // inital values
  Values initial;
  for (int i = 0; i < numberNodes; ++i) {
    initial.insert(i, poses[i]); // poses在最开始定义了
  }

  LevenbergMarquardtParams params;
  params.setVerbosity("ERROR"); // 输出详细程度
  params.setOrderingType("METIS"); // 选择排序方法
  params.setLinearSolverType("MULTIFRONTAL_CHOLESKY"); // 线性求解器类型
  LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  auto result = optimizer.optimize();
}

int main(int args, char* argv[]) {
  int numberNodes = stoi(argv[1]); // 节点个数
  cout << "number of_nodes: " << numberNodes << endl;
  testGtsam(numberNodes);
  return 0;
}
