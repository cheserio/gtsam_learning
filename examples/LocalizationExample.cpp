/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cpp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Frank Dellaert
 * 简单的机器人定位例子，有三个类似GPS的测量值
 */

/**
* 一个带有“GPS”测量的简单 2D
* 机器人每次迭代向前移动 2 米
* 机器人最初面向 X 轴（水平向右）
* 我们在姿势之间有完整的里程计测量数据
* 我们使用自定义因子实施“类似 GPS”的测量
*/

// 我们将使用 Pose2 变量 (x, y, theta) 来表示机器人位置
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
// 我们将使用简单的整数键来指代机器人的位姿
#include <gtsam/inference/Key.h>

// 与在 Odometry Example.c++ 中一样，我们使用 BetweenFactor 对里程计测量进行建模。
#include <gtsam/slam/BetweenFactor.h>

// 我们将所有因子添加到非线性因子图中，因为我们的因子是非线性的。
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
// 结果接收器
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
// 优化器
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
// 近似后验协方差计算
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
// 在我们开始这个例子之前，我们必须创建一个自定义的一元因子来实现一个“类似 GPS”的功能。因为标准 GPS 测量提供信息
// 仅在位置上，而不在方向上，我们不能使用简单的先验正确地模拟这个测量。

// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
// 该因子将是一元因子，仅影响单个系统变量。也使用标准的高斯噪声模型。
#include <gtsam/nonlinear/NonlinearFactor.h>

class UnaryFactor: public NoiseModelFactor1<Pose2> {
  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  // 该因子为了适配GPS数据只需要保存x， y两个值即可
  double mx_, my_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  // 构造函数需要(X, Y)测量值，变量key（在Values中查找）和噪声模型（指向噪声的共享指针）
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  ~UnaryFactor() override {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  // 使用 NoiseModelFactor1 基类有两个必须重写的函数。第一个是“evaluateError”函数。此功能实现所需的测量
  // 函数，在对提供的变量值求值时返回一个错误向量。它如果需要，还必须为此测量函数计算雅可比行列式。
  Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const override {
    // The measurement function for a GPS-like measurement h(q) which predicts the measurement (m) is h(q) = q, q = [qx qy qtheta]
    // The error is then simply calculated as E(q) = h(q) - m:
    // 这里需要一个h函数将状态量转换成与观测值同维（观测矩阵）
    // 误差就是这里转换后的预测状态量 - 测量值
    // error_x = q.x - mx
    // error_y = q.y - my
    // 按z轴旋转 省略了最后一行1
    // Node's orientation reflects in the Jacobian, in tangent space this is equal to the right-hand rule rotation matrix
    // H =  [ cos(q.theta)  -sin(q.theta) 0 ]
    //      [ sin(q.theta)   cos(q.theta) 0 ]
    const Rot2& R = q.rotation(); // q本身是有theta的 这里rotation之后就可以得到上面H矩阵
    // 如果传入了一个H（观测矩阵）的指针 这里解引用之后赋值
    // 总而言之就是根据状态量里面的theta得到了一个观测矩阵
    if (H) (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished(); // x y 是可以直接相减得到误差值的与theta没有关系
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  //第二个是允许复制因子的“克隆”功能。大多数下情况下，下面使用默认复制构造函数的代码应该工作正常
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
};  // UnaryFactor


int main(int argc, char** argv) {
  // 1. 创建一个因子图容器并向其中添加因子
  NonlinearFactorGraph graph;

  // 2a.添加里程计因素
  // 为简单起见，我们将对每个里程计因子使用相同的噪声模型
  auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  // 在连续姿势之间创建里程计（Between）因子
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise);

  //2b.添加“类似 GPS”的测量
  //我们将为此使用自定义的 UnaryFactor
  auto unaryNoise =
      noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));  // 10cm std on x,y 误差均值为10cm 
  graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise); // 第一个插入的元素是key 也就是对应的位姿节点的编号 然后是xy以及噪声指针
  graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);
  graph.print("\nFactor Graph:\n");  // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  // 创建接收容器填入错误的预测值
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n");  // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  // 创建优化器 传入因子图以及预测值
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize(); // 优化
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  // 计算每个节点的后验概率协方差
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}
