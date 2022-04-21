/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Pose2SLAMExample_g2o.cpp
 * @brief A 2D Pose SLAM example that reads input from g2o, converts it to a factor graph and does the
 * optimization. Output is written on a file, in g2o format
 * Syntax for the script is ./Pose2SLAMExample_g2o input.g2o output.g2o
 * @date May 15, 2014
 * @author Luca Carlone
 * 2Dslam示例，从g2o读入数据，将其转换为因子图然后做优化，出入到一个文件中也是以g2o的形式
 */

#include <gtsam/slam/dataset.h> // 读入数据
#include <gtsam/geometry/Pose2.h> // 位姿表示
#include <gtsam/nonlinear/GaussNewtonOptimizer.h> // 高斯牛顿优化器
#include <fstream> // 基本的文件输出

using namespace std;
using namespace gtsam;

// 用户根据自己情况调用该程序时需要修改inputFile outputFile 最大迭代次数 噪声鲁邦核函数类型选择
// HOWTO: ./Pose2SLAMExample_g2o inputFile outputFile (maxIterations) (tukey/huber)
int main(const int argc, const char *argv[]) {
  string kernelType = "none";
  int maxIterations = 100;                                    // default
  string g2oFile = findExampleDataFile("noisyToyGraph.txt");  // default

  // Parse user's inputs
  // 自己使用的时候会走这里上面是一些默认值
  if (argc > 1) {
    g2oFile = argv[1]; // 输入数据集文件名
  }
  if (argc > 3) {
    maxIterations = atoi(argv[3]); // 用户可以指定迭代次数
  }
  if (argc > 4) {
    kernelType = argv[4]; // 用户可以指定 tukey 或 huber
  }

  // 读取文件并创建因子图
  NonlinearFactorGraph::shared_ptr graph; // 创建因子图智能指针
  Values::shared_ptr initial; // 创建预测值容器指针
  bool is3D = false; // 不是3D数据
  // 如果损失函数啥也没输入
  if (kernelType.compare("none") == 0) {
    // boost::tie是创建元组 后面的readG2o会返回因子图和初始预测值
    boost::tie(graph, initial) = readG2o(g2oFile, is3D);
  }
  // 如果损失函数是huber
  if (kernelType.compare("huber") == 0) {
    std::cout << "Using robust kernel: huber " << std::endl;
    boost::tie(graph, initial) =
        readG2o(g2oFile, is3D, KernelFunctionTypeHUBER); // 使用HUBER鲁棒核函数来控制噪声 
  }
  // 如果损失函数是tukey
  if (kernelType.compare("tukey") == 0) {
    std::cout << "Using robust kernel: tukey " << std::endl;
    boost::tie(graph, initial) =
        readG2o(g2oFile, is3D, KernelFunctionTypeTUKEY); // 使用TUKEY鲁棒核函数来控制噪声 
  }

  // Add prior on the pose having index (key) = 0
  // 这里是使用方差创建噪声模型
  auto priorModel =  //
      noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
  graph->addPrior(0, Pose2(), priorModel); // 添加先验点 其余的点是通过g2o读入的
  std::cout << "Adding prior on pose 0 " << std::endl;

  GaussNewtonParams params; // 高斯牛顿优化器参数配置
  params.setVerbosity("TERMINATION"); // 优化期间打印结果的详细程度 有个枚举的列表
  // 载入迭代次数
  if (argc > 3) {
    params.maxIterations = maxIterations;
    std::cout << "User required to perform maximum  " << params.maxIterations
              << " iterations " << std::endl;
  }

  std::cout << "Optimizing the factor graph" << std::endl;
  GaussNewtonOptimizer optimizer(*graph, *initial, params); // 创建迭代器由于之前使用的都是智能指针这里需要解引用
  Values result = optimizer.optimize(); // 优化
  std::cout << "Optimization complete" << std::endl;

  std::cout << "initial error=" << graph->error(*initial) << std::endl; // 观测量和初始预测值之间的误差
  std::cout << "final error=" << graph->error(result) << std::endl;  // 观测量和预测值之间的误差

  if (argc < 3) {
    result.print("result");
  } else {
    // 输入参数大于3的话就存入了输出地址
    const string outputFile = argv[2];
    std::cout << "Writing results to file: " << outputFile << std::endl;
    NonlinearFactorGraph::shared_ptr graphNoKernel;
    Values::shared_ptr initial2;
    // 将g2o中的文件再读一遍 然后结合上面得到的结果输出到输出文件中
    boost::tie(graphNoKernel, initial2) = readG2o(g2oFile);
    writeG2o(*graphNoKernel, result, outputFile);
    std::cout << "done! " << std::endl;
  }
  return 0;
}
