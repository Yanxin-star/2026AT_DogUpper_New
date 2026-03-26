#ifndef __ARM_STEP_H__
#define __ARM_STEP_H__

#include <Eigen/Dense>
#include <tuple>
#include <functional>

using Vector6d = Eigen::Matrix<double, 6, 1>;

typedef Eigen::Vector2d Vector2D;
typedef Eigen::Vector3d Vector3D;

typedef struct {
    double exp_vx;
    double exp_vy;
    double H;              // 摆动轨迹半径
    double Lx;             // x方向步长
    double Ly;             // x方向步长
    float T;
} CycloidStep_t;

bool UpdateCycloidStep(const Vector2D& exp_vel, CycloidStep_t* line, float time, float step_height);
std::tuple<Vector3D, Vector3D, Vector3D> GetCycloidStep(float time, CycloidStep_t& line);

class ArmStep {
public:
    typedef struct {
        double a;
        double b;
        double c;
        double d;
        double e;
        double f;
    } QuinticLineParam_t;

    typedef struct {
        double k;
        double b;
    } StraightLineParam_t;

    typedef struct {
        QuinticLineParam_t lx;
        QuinticLineParam_t ly;
        QuinticLineParam_t l1_z;
        QuinticLineParam_t l2_z;
        double time;       // 摆动相全程时间
    } StepTrajectory_t;

    typedef struct {
        StraightLineParam_t lx;
        StraightLineParam_t ly;
        StraightLineParam_t lz;
        double time;
    } SupportTrajectory_t; // 支撑相全程时间



   
    typedef struct {
        QuinticLineParam_t lx;
        QuinticLineParam_t ly;
        QuinticLineParam_t lz;      // ← 新增（原 l1_z/l2_z 已废弃）
        QuinticLineParam_t lrx;     // ← 新增
        QuinticLineParam_t lry;     // ← 新增
        QuinticLineParam_t lrz;     // ← 新增
        double time;                // 规划总时间（支持中途 replan 时传入剩余时间）
    } Armpram;             // 现已完全适配 6DOF 机械臂（
    







    ArmStep();
    ArmStep(const double x_limit,const double y_limit);
    void update_flight_trajectory(const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector2D& exp_vel, const double time, const double step_height,const double target_height=0.0,const double x_offset=0.0,const double y_offset=0.0);
    void update_flight_trajectory(
        const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector3D& exp_pos, const Vector2D& exp_vel, const double time, const double step_height);
    // 带回调函数的重载版本：在摆动相最高点时重新规划落足点
    void update_flight_trajectory(
        const Vector3D& cur_pos, const Vector3D& cur_vel, const Vector2D& exp_vel, const double time, const double step_height,
        std::function<Vector3D(const Vector3D&)> replanning_callback,
        const double target_height=0.0, const double x_offset=0.0, const double y_offset=0.0);
    void update_support_trajectory(const Vector3D& cur_pos, const Vector2D& exp_vel, double time);
    void update_support_trajectory(const Vector3D& cur_pos, const Vector3D final_pos, double time);
    std::tuple<Vector3D, Vector3D, Vector3D> get_target(double time,bool &success);








    void update_arm_trajectory(
    const Vector6d& cur_pose,   // 当前末端位姿 [x, y, z, rx, ry, rz]
    const Vector6d& cur_vel,    // 当前末端速度（6维）
    const Vector6d& exp_pose,   // 目标位姿
    const Vector6d& exp_vel,    // 目标速度（6维）
    double time);

    std::tuple<Vector6d, Vector6d, Vector6d> get_arm_target(double time, bool& success);


private:
    bool flight_trajectory_is_available{false};
    bool support_trajectory_is_available{false};
    bool arm_trajectory_is_available{false};
    StepTrajectory_t flight_trajectory;
    SupportTrajectory_t support_trajectory;


    Armpram arm_trajectory;


    
    double x_limit;
    double y_limit;
    // 中途重新规划相关变量
    bool needs_mid_replanning{false};           // 是否需要中途重新规划
    bool mid_replanning_done{false};            // 是否已完成中途重新规划
    Vector3D initial_target_pos;                // 初始规划的落足点
    std::function<Vector3D(const Vector3D&)> replanning_callback; // 重新规划回调函数
};

#endif
