#include "arm_calc/arm_calc.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/Dense>
#include <chrono>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/logger.hpp>

using namespace std::chrono_literals;
using Vector6d = Eigen::Matrix<double, 6, 1>;

ArmCalc::ArmCalc(KDL::Chain& chain)
    : chain(chain)
    , fk_solver(chain)
    , jacobain_solver(chain)
    , jdot_solver(chain)
    , vel_solver(chain)
    ,ik_pos_solver(chain, Eigen::Vector<double,6>(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),1e-6,150,1e-10)
    , dynamin_solver(chain, KDL::Vector(0, 0, -9.81))
    {
    _temp_joint3_array.resize(6);   //提前resize需要用到的KDL::JntArray防止运行时频繁申请/释放内存
    _temp2_joint3_array.resize(6);
    last_exp_joint_pos.resize(6);
    temp_jacobain.resize(6);
    _temp_joint3_vel_array.resize(6);
    
    C.resize(6);
    G.resize(6);
    M.resize(6);
}

ArmCalc::~ArmCalc() {

}



Eigen::Matrix<double, 6, 6> ArmCalc::get_6x6_jacobian_(const KDL::Jacobian &full_jacobian)     
{
    Eigen::Matrix<double, 6, 6> jacobian_6x6;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            jacobian_6x6(i, j)       = full_jacobian(i, j);
        }
    }
    return jacobian_6x6;
}


Vector6d ArmCalc::joint_pos(const Eigen::Vector3d &pos,const Eigen::Vector3d &rpy,int *result) {
    KDL::Frame frame;
    
    frame.p.x(pos(0));
    frame.p.y(pos(1));
    frame.p.z(pos(2));
    frame.M = KDL::Rotation::EulerZYX(rpy[0], rpy[1], rpy[2]);//这里需要根据实际去改动位置
    
    *result= ik_pos_solver.CartToJnt(last_exp_joint_pos, frame,_temp_joint3_array);
    if(*result==0)  //缓存本次计算结果,方便下一次迭代
        last_exp_joint_pos=_temp_joint3_array;
    else
        _temp_joint3_array=last_exp_joint_pos;  //先赋值为上一次可行的解防止失败 
    Vector6d joint_angles(6);
    joint_angles(0) = _temp_joint3_array(0);
    joint_angles(1) = _temp_joint3_array(1);
    joint_angles(2) = _temp_joint3_array(2);
    joint_angles(3) = _temp_joint3_array(3);
    joint_angles(4) = _temp_joint3_array(4);
    joint_angles(5) = _temp_joint3_array(5);
    
    return joint_angles;
} 
/*使用示例,具体最好还是看一些好的代码
Eigen::Vector3d pos(0.3, 0.1, 0.5);           // xyz
Eigen::Vector3d rpy(0.2, 0.1, -0.3);          // yaw, pitch, roll（弧度）
int result;

Eigen::VectorXd joints = armCalc.joint_pos(pos, rpy, &result);

if (result == 0) {
    // joints(0) ~ joints(5) 就是六关节角度
}
*/



Vector6d ArmCalc::joint_vel(const Vector6d &joint_rad, 
                                   const Vector6d &arm_exp_vel)  // foot_vel = [vx, vy, vz, wx, wy, wz]（m/s + rad/s）
{
    
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];
    _temp_joint3_array(3) = joint_rad[3];
    _temp_joint3_array(4) = joint_rad[4];
    _temp_joint3_array(5) = joint_rad[5];

    // 计算完整6x6雅可比矩阵（KDL会根据链的关节数自动适配）
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);

    // 构造Eigen 6x6 Jacobian
    Eigen::Matrix<double, 6, 6> jacobian;
    for(int i = 0; i < 6; ++i) {
        for(int j = 0; j < 6; ++j) {
            jacobian(i, j) = temp_jacobain(i, j);
        }
    }

    // 关节速度 = J⁻¹ × 末端速度（线性+角速度）
    return jacobian.inverse() * arm_exp_vel;
}


/**
    @brief 计算关节角加速度
    @param joint_rad 关节角度向量
    @param joint_vel 关节角速度
    @param foot_acc  期望的足端加速度
    @return 关节角加速度向量
 */
Vector6d ArmCalc::joint_acc(const Vector6d &joint_rad, const Vector6d &joint_vel,Vector6d arm_acc)
{
    _temp_joint3_array.data=joint_rad;
    _temp2_joint3_array.data=joint_vel;
    _temp_joint3_vel_array.q.data=joint_rad;
    _temp_joint3_vel_array.qdot.data=joint_vel;

    //计算雅可比矩阵J
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    jdot_solver.JntToJacDot(_temp_joint3_vel_array,_temp_jdot_qd);

    Eigen::Matrix<double, 6, 6> Jac = get_6x6_jacobian_(temp_jacobain);
    Vector6d jdot_dq_eigen;
    for(int i=0;i<6;i++){
        jdot_dq_eigen[i]=_temp_jdot_qd(i);
    }
    return Jac.completeOrthogonalDecomposition().solve(arm_acc - jdot_dq_eigen);
}
//这里需要加上加速度限幅

//计算关节前馈动力学力矩
Vector6d ArmCalc::joint_torque_dynamic(const Vector6d &joint_rad, const Vector6d &joint_omega, const Vector6d &foot_acc) {
    _temp_joint3_array(0)=joint_rad[0];
    _temp_joint3_array(1)=joint_rad[1];
    _temp_joint3_array(2)=joint_rad[2];
    _temp_joint3_array(3)=joint_rad[3];
    _temp_joint3_array(4)=joint_rad[4];
    _temp_joint3_array(5)=joint_rad[5];

    _temp2_joint3_array(0)=joint_omega[0];
    _temp2_joint3_array(1)=joint_omega[1];
    _temp2_joint3_array(2)=joint_omega[2];
    _temp2_joint3_array(3)=joint_omega[3];
    _temp2_joint3_array(4)=joint_omega[4];
    _temp2_joint3_array(5)=joint_omega[5];

    dynamin_solver.JntToGravity(_temp_joint3_array, G);
    dynamin_solver.JntToCoriolis(_temp_joint3_array, _temp2_joint3_array, C);
    dynamin_solver.JntToMass(_temp_joint3_array, M);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 6, 6> M_;
    Eigen::Matrix<double, 6, 1> C_, G_;

    for (int i = 0; i < 6; ++i) {
        C_(i)   = C(i);
        G_(i)   = G(i);
        for (int j = 0; j < 6; ++j) {
            M_(i, j) = M(i, j);
        }
    }
    // 7. 计算前馈力矩 tau
    return M_ * joint_acc(joint_rad, joint_omega,foot_acc) + C_ + G_;
}

Vector6d ArmCalc::joint_torque_dynamic2(const Vector6d &joint_rad, const Vector6d &joint_omega, const Vector6d &joint_acc) {
    _temp_joint3_array(0)=joint_rad[0];
    _temp_joint3_array(1)=joint_rad[1];
    _temp_joint3_array(2)=joint_rad[2];
    _temp_joint3_array(3)=joint_rad[3];
    _temp_joint3_array(4)=joint_rad[4];
    _temp_joint3_array(5)=joint_rad[5];

    _temp2_joint3_array(0)=joint_omega[0];
    _temp2_joint3_array(1)=joint_omega[1];
    _temp2_joint3_array(2)=joint_omega[2];
    _temp2_joint3_array(3)=joint_omega[3];
    _temp2_joint3_array(4)=joint_omega[4];
    _temp2_joint3_array(5)=joint_omega[5];

    dynamin_solver.JntToGravity(_temp_joint3_array, G);
    dynamin_solver.JntToCoriolis(_temp_joint3_array, _temp2_joint3_array, C);
    dynamin_solver.JntToMass(_temp_joint3_array, M);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 6, 6> M_;
    Eigen::Matrix<double, 6, 1> C_, G_;

    for (int i = 0; i < 6; ++i) {
        C_(i)   = C(i);
        G_(i)   = G(i);
        for (int j = 0; j < 6; ++j) {
            M_(i, j) = M(i, j);
        }
    }
    // 7. 计算前馈力矩 tau
    return M_ * joint_acc+ C_ + G_;
}



/**
    @brief 足端期望力->计算关节力矩
    @param joint_rad 关节角度
    @param joint_force 关节末端期望力
    @return 关节空间下的力矩
 */
Vector6d ArmCalc::joint_torque_foot_force(const Vector6d &joint_rad,const Vector6d &foot_force){
    _temp_joint3_array(0)=joint_rad[0];
    _temp_joint3_array(1)=joint_rad[1];
    _temp_joint3_array(2)=joint_rad[2];
    _temp_joint3_array(3)=joint_rad[3];
    _temp_joint3_array(4)=joint_rad[4];
    _temp_joint3_array(5)=joint_rad[5];

    jacobain_solver.JntToJac(_temp_joint3_array,temp_jacobain);
    Eigen::Matrix<double, 6, 6> jacobian = get_6x6_jacobian_(temp_jacobain);
    Vector6d torque(foot_force(0),foot_force(1),foot_force(2),foot_force(3),foot_force(4),foot_force(5));
    return jacobian.transpose()* torque;
}

/**
    @brief 计算足端受力
    @param joint_rad 当前关节角度
    @param joint_torque 总力矩减去克服重力/科氏力/惯性力剩下的力矩（需要在外部计算）
    @return 笛卡尔坐标系下的足端受力
 */
Vector6d ArmCalc::foot_force(const Vector6d &joint_rad,const Vector6d &joint_torque,const Vector6d &forward_torque) {
    _temp_joint3_array(0)=joint_rad[0];
    _temp_joint3_array(1)=joint_rad[1];
    _temp_joint3_array(2)=joint_rad[2];
    _temp_joint3_array(3)=joint_rad[3];
    _temp_joint3_array(4)=joint_rad[4];
    _temp_joint3_array(5)=joint_rad[5];

    jacobain_solver.JntToJac(_temp_joint3_array,temp_jacobain);
    auto jacobian = get_6x6_jacobian_(temp_jacobain);

    /*AI解析出的更稳定的SVD分解，待尝试
    Vector6d force = jacobian.transpose()
                               .jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
                               .solve(joint_torque - forward_torque);
    return force;
    */
    return jacobian.transpose().inverse()*(joint_torque-forward_torque);
}

/**
    @brief 计算足端速度
    @param joint_rad 当前关节角度
    @param joint_omega 当前关节角速度
    @return 当前足端速度
 */
Vector6d ArmCalc::foot_vel(const Vector6d &joint_rad, const Vector6d &joint_omega) {
    _temp_joint3_array(0)=joint_rad[0];
    _temp_joint3_array(1)=joint_rad[1];
    _temp_joint3_array(2)=joint_rad[2];
    _temp_joint3_array(3)=joint_rad[3];
    _temp_joint3_array(4)=joint_rad[4];
    _temp_joint3_array(5)=joint_rad[5];

    jacobain_solver.JntToJac(_temp_joint3_array,temp_jacobain);

    auto jacobian = get_6x6_jacobian_(temp_jacobain);     //提取雅可比矩阵中与位置相关的部分
    Vector6d dq(joint_omega(0),joint_omega(1),joint_omega(2),joint_omega(3),joint_omega(4),joint_omega(5));
    return jacobian*dq;
}

/**
    @brief 计算足端位置
    @param joint_rad 关节角度向量
    @return 当前足端位置
 */
Vector6d ArmCalc::foot_pos(const Vector6d& joint_rad) {
    KDL::Frame frame;

    _temp_joint3_array(0)=joint_rad[0];
    _temp_joint3_array(1)=joint_rad[1];
    _temp_joint3_array(2)=joint_rad[2];
    _temp_joint3_array(3)=joint_rad[3];
    _temp_joint3_array(4)=joint_rad[4];
    _temp_joint3_array(5)=joint_rad[5];
    fk_solver.JntToCart(_temp_joint3_array, frame);

    Vector6d pose; // [x, y, z, roll, pitch, yaw]

    // 末端位置
    pose(0) = frame.p.x();
    pose(1) = frame.p.y();
    pose(2) = frame.p.z();

    // 末端姿态（欧拉角 ZYX）
    double roll, pitch, yaw;
    frame.M.GetRPY(roll, pitch, yaw);  // KDL提供函数获取欧拉角
    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;

    return pose;

}

