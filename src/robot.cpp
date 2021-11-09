#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>
#include <Eigen/SVD>
#include <Eigen/Core>

#include "robot.h"
#include"plan.h"

using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;
namespace robot
{

// 利用Eigen库，采用SVD分解的方法求解矩阵伪逆，默认误差er为0
Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd & origin, const float er = 0) {
    // 进行svd分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin,
                                                 Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
    // 构建SVD分解结果
    Eigen::MatrixXd U = svd_holder.matrixU();
    Eigen::MatrixXd V = svd_holder.matrixV();
    Eigen::MatrixXd D = svd_holder.singularValues();

    // 构建S矩阵
    Eigen::MatrixXd S(V.cols(), U.cols());
    S.setZero();

    for (unsigned int i = 0; i < D.size(); ++i) {

        if (D(i, 0) > er) {
            S(i, i) = 1 / D(i, 0);
        } else {
            S(i, i) = 0;
        }
    }

    // pinv_matrix = V * S * U^T
    return V * S * U.transpose();
}

//设置力矩
auto SetMaxToq::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::CHECK_NONE;
}
auto SetMaxToq::executeRT()->int
{
    auto &cout = controller()->mout();

    for (int i = 0; i < 1; ++i)
    {
        std::uint16_t max_toq=1000;
        this->ecController()->motionPool()[i].writePdo(0x6072,0,max_toq);
    }
    //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
    return 1000 - count();
}
auto SetMaxToq::collectNrt()->void {}
SetMaxToq::~SetMaxToq() = default;
SetMaxToq::SetMaxToq(const std::string &name){
    aris::core::fromXmlString(command(),
                "<Command name=\"set_toq\"/>");
}

//move joint single //
auto MoveSingleMotor::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    m_id = int32Param({"motor_id"});
    dist = doubleParam("theta");

}
auto MoveSingleMotor::executeRT()->int
{
    TCurve T(0.05, 0.2);
    T.getCurveParam();

    static double begin_angle;
    std::cout << "count: " << count() << std::endl;
//    static int i=0;
    //read begin_pos
    if(count() == 1){
        begin_angle = controller()->motionPool()[m_id].actualPos();
        std::cout << "begin_angle: " << begin_angle << std::endl;
    }
    //move slow
    controller()->motionPool()[m_id].setTargetPos(begin_angle + dir_ * (dist * T.getTCurve(count())));
    return T.getTc() * 1000 - count();

}
MoveSingleMotor::MoveSingleMotor(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"mvsi\">"
        "<GroupParam>"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "<Param name=\"motor_id\" default=\"0\" abbreviation=\"m\"/>"
        "<Param name=\"theta\" default=\"0\" abbreviation=\"t\"/>"
        "</GroupParam>"
        "</Command>");
}
MoveSingleMotor::~MoveSingleMotor() = default;

//move joint test//
auto MoveSingleTest::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    m_id = int32Param({"motor_id"});
    dist = doubleParam("theta");

}
auto MoveSingleTest::executeRT()->int
{
    TCurve S(0.1,0.2);
    S.getCurveParam();
    static double begin_angle;
    std::cout << "count: " << count() << std::endl;
    if(count() == 1){
        begin_angle = controller()->motionPool()[m_id].actualPos();
        std::cout << "begin_angle: " << begin_angle << std::endl;
    }
    controller()->motionPool()[m_id].setTargetPos(begin_angle + dir_ * (dist * S.getTCurve(count())));
    return  S.getTc() * 1000 - count();
}
MoveSingleTest::MoveSingleTest(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"mvst\">"
        "<GroupParam>"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "<Param name=\"motor_id\" default=\"0\" abbreviation=\"m\"/>"
        "<Param name=\"theta\" default=\"0\" abbreviation=\"t\"/>"
        "</GroupParam>"
        "</Command>");
}
MoveSingleTest::~MoveSingleTest() = default;

//read joint single //
auto ReadSingleMotor::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    m_id = int32Param({"motor_id"});
    dist = doubleParam("theta");

}
auto ReadSingleMotor::executeRT()->int
{
    double begin_angle;
    begin_angle = controller()->motionPool()[m_id].actualPos();
    std::cout << "pos" << m_id << ": " << begin_angle << std::endl;

    return 0;

}
ReadSingleMotor::ReadSingleMotor(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"rdsi\">"
        "<GroupParam>"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "<Param name=\"motor_id\" default=\"0\" abbreviation=\"m\"/>"
        "<Param name=\"theta\" default=\"0\" abbreviation=\"t\"/>"
        "</GroupParam>"
        "</Command>");
}
ReadSingleMotor::~ReadSingleMotor() = default;

//move screw
auto MoveScrew::prepareNrt()->void
{
    //for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveScrew::executeRT()->int
{
    TCurve s1(0.05,0.2);
    s1.getCurveParam();

    static double begin_pos[7];
    if(count() == 1){
    for(int i = 0;i < 7;++i)
    {
        begin_pos[i] = controller()->motionPool()[i].actualPos();
    }
    }

    double pos0 = begin_pos[0] + PI / 3 * s1.getTCurve(count());
    double pos1 = begin_pos[1] + PI / 3 * s1.getTCurve(count());
    double pos2 = begin_pos[2] + PI / 3 * s1.getTCurve(count());
    double pos3 = begin_pos[3] + PI / 3 * s1.getTCurve(count());
    double pos4 = begin_pos[4] + PI / 3 * s1.getTCurve(count());
    double pos5 = begin_pos[5] + PI / 3 * s1.getTCurve(count());
    double pos6 = begin_pos[6] + PI / 3 * s1.getTCurve(count());

    controller()->motionPool()[0].setTargetPos(pos0);
    controller()->motionPool()[1].setTargetPos(pos1);
    controller()->motionPool()[2].setTargetPos(pos2);
    controller()->motionPool()[3].setTargetPos(pos3);
    controller()->motionPool()[4].setTargetPos(pos4);
    controller()->motionPool()[5].setTargetPos(pos5);
    controller()->motionPool()[6].setTargetPos(pos6);


    return 1;
}
MoveScrew::MoveScrew(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"mvsc\"/>");
}
MoveScrew::~MoveScrew() = default;

// read joint pos //
//读当前关节角度
auto ReadJoint::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto ReadJoint::executeRT()->int
{
    double begin_angle[7];
    for(int i = 0;i < 7;++i)
    {
        begin_angle[i] = controller()->motionPool()[i].actualPos();
    }
    aris::dynamic::dsp(7,1,begin_angle);

    return 0;
}
ReadJoint::ReadJoint(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"read_j\"/>");
}
ReadJoint::~ReadJoint() = default;

// 单关节正弦往复轨迹 //
struct MoveJSParam
{
    double j1;//振幅rad
    double time;//周期
    uint32_t timenum;//周期数
};
auto MoveJS::prepareNrt()->void
{
    MoveJSParam param;

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto& p : cmdParams())
    {
        if (p.first == "j1")
        {
            if (p.second == "current_pos")
            {
                param.j1 = controller()->motionPool()[0].actualPos();

            }
            else
            {
                param.j1 = doubleParam(p.first);
            }

        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;
    std::vector<std::pair<std::string, std::any>> ret_value;
    for (auto& option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_POS_CONTINUOUS;
    ret() = ret_value;
}
auto MoveJS::executeRT()->int
{
    auto& param = std::any_cast<MoveJSParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;
    // 访问主站 //
    auto& cout = controller()->mout();

    if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI * count() / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }

        step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI * (count() - time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {
        // 获取当前起始点位置 //
        if (count() == totaltime - time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI * (count() - totaltime + time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }

//    // 打印 //
//    if (count() % 100 == 0)
//    {
//        cout << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
//        cout << std::endl;
//    }

//    // log //
//    auto& lout = controller()->lout();
//    lout << controller()->motionAtAbs(0).targetPos() << ",";
//    lout << std::endl;

//    // read toeque //
//    std::int16_t torque=0;
//    this->ecController()->motionPool()[0].readPdo(0x6077,0x00,torque);
//    mout()<< torque <<std::endl;

    return totaltime - count();
}
auto MoveJS::collectNrt()->void {}
MoveJS::MoveJS(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"moveJS\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}

//从轴空间回原点
auto DogHome::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogHome::executeRT()->int
{
    static double begin_angle[7]={0};
    double angle[7]={0};

    if (count() == 1)
    {
        this->master()->logFileRawName("home");
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
    }

//    ecController()->motionPool()[0].readPdo(0x1030)
    TCurve s1(0.5,0.2);//0.9s
    s1.getCurveParam();

    for(int i=0;i<7;++i)
    {
        angle[i] = begin_angle[i] + (0-begin_angle[i]) * s1.getTCurve(count());
    }

    for(int i=0;i<7;++i)
    {
        controller()->motionPool()[i].setTargetPos(angle[i]);
    }
    return s1.getTc() * 1000-count();
}
DogHome::DogHome(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"home\"/>");
}
DogHome::~DogHome() = default;

auto MoveJoint::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveJoint::executeRT()->int
{
    // pos //
    static double begin_pos[7];
//    std::cout << "fafafafafafa" << std::endl;
    //static double pos_now;

    TCurve s1(0.05,0.2);
    s1.getCurveParam();

    if (count() == 1)
    {
        mout() << s1.getTc() * 1000 << std::endl;
        begin_pos[0] = controller()->motionPool()[0].actualPos();
        begin_pos[1] = controller()->motionPool()[1].actualPos();
        begin_pos[2] = controller()->motionPool()[2].actualPos();
        begin_pos[3] = controller()->motionPool()[3].actualPos();
        begin_pos[4] = controller()->motionPool()[4].actualPos();
        begin_pos[5] = controller()->motionPool()[5].actualPos();
        begin_pos[6] = controller()->motionPool()[6].actualPos();


    }

//    if(step1 == 1){
//        double pos0 = begin_pos[0] + dir_ * PI / 2 * s1.getTCurve(count());
//        controller()->motionPool()[0].setTargetPos(pos0);
//        if(controller()->motionPool()[0].actualPos() - begin_pos[0] + dir_ * PI / 2 == 0){
//            step1 = 0;
//            step2 = 1;
//            begin_pos[0] = controller()->motionPool()[0].actualPos();
//            count_temp = count();
//        }
//    }
//    else if(step2 == 1){
//        double pos0 = begin_pos[0] - dir_ * PI / 2 * s1.getTCurve(count() - count_temp);
//        controller()->motionPool()[0].setTargetPos(pos0);
//        if(controller()->motionPool()[0].actualPos() - begin_pos[0] + dir_ * PI / 2 == 0){
//            step2 = 0;
//            step3 = 1;
//            begin_pos[0] = controller()->motionPool()[0].actualPos();
//        }
//    }
//        return 1;
    double pos0 = begin_pos[0] + dir_ * PI / 3 * s1.getTCurve(count());
    double pos1 = begin_pos[1] + dir_ * PI / 3 * s1.getTCurve(count());
    double pos2 = begin_pos[2] + dir_ * PI / 3 * s1.getTCurve(count());
    double pos3 = begin_pos[3] + dir_ * PI / 3 * s1.getTCurve(count());
    double pos4 = begin_pos[4] + dir_ * PI / 3 * s1.getTCurve(count());
    double pos5 = begin_pos[5] + dir_ * PI / 3 * s1.getTCurve(count());
    double pos6 = begin_pos[6] + dir_ * PI / 3 * s1.getTCurve(count());

    controller()->motionPool()[0].setTargetPos(pos0);
    controller()->motionPool()[1].setTargetPos(pos1);
    controller()->motionPool()[2].setTargetPos(pos2);
    controller()->motionPool()[3].setTargetPos(pos3);
    controller()->motionPool()[4].setTargetPos(pos4);
    controller()->motionPool()[5].setTargetPos(pos5);
    controller()->motionPool()[6].setTargetPos(pos6);


//    // read toeque //
//    std::int16_t torque=0;
//    this->ecController()->motionPool()[0].readPdo(0x6077,0x00,torque);
//    pos_now = controller()->motionPool()[0].actualPos();
//    mout()<< "torque: " << torque <<std::endl;
//    mout()<< "posnow: " << pos_now <<std::endl;

    return 1;
}
auto MoveJoint::collectNrt()->void {}
MoveJoint::MoveJoint(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"mvj\">"
        "		<Param name=\"direction\" default=\"1.0\" abbreviation=\"d\"/>"
        "</Command>");
}

auto MoveJ::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveJ::executeRT()->int
{
    // pos //
    static double begin_pos[7];
    //static double pos_now;

    TCurve s1(0.05,0.2);
    s1.getCurveParam();

    if (count() == 1)
    {
        mout() << s1.getTc() * 1000 << std::endl;
        begin_pos[0] = controller()->motionPool()[0].actualPos();
    }
    double pos0 = begin_pos[0] + dir_ * PI / 2 * s1.getTCurve(count());
    controller()->motionPool()[0].setTargetPos(pos0);


    return s1.getTc() *1000 -count();
}
auto MoveJ::collectNrt()->void {}
MoveJ::MoveJ(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"mj\">"
        "		<Param name=\"direction\" default=\"1.0\" abbreviation=\"d\"/>"
        "</Command>");
   }

//move joint torque //
auto MoveTorque::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveTorque::executeRT()->int
{


    static double begin_angle;
    static int i=0;
    // read toeque //
    std::int16_t torque=0;
    this->ecController()->motionPool()[0].readPdo(0x6077,0x00,torque);
    mout()<< torque <<std::endl;

    controller()->motionPool()[0].setTargetToq(18);
    return 1;

}
MoveTorque::MoveTorque(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"test_mvt\">"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
MoveTorque::~MoveTorque() = default;

//move joint velosity //
auto MoveVelosity::prepareNrt()->void
{
    dir_ = doubleParam("direction");
//    pos_step = doubleParam("step");
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveVelosity::executeRT()->int
{
    static double step_velosity = 0.02;
    static double Velosity = 0;
    static double Velosity_desired = 0;
    static double max_vel=10;
    static double pos;
    static double pos_start;
    static double pos_step = -PI;
    static double pos_target;
    static double pos_error;
    static double pos_error_sum = 0;
    static double kp = 1;
    static double ki = 0.02;
    // read vel //
    if(count() == 1){
    this->ecController()->motionPool()[0].readPdo(0x6077,0x00,Velosity);
//    this->ecController()->motionPool()[0].readPdo(0x6064,0x00,pos_start);
    pos_start = controller()->motionPool()[0].actualPos();
    mout() << "vel: " << Velosity  << std::endl;
    mout() << "pos: " << pos_start << std::endl;
    pos_target = pos_start + pos_step;
    mout() << "pos target: " << pos_target << std::endl;
    }
    //PI control
    pos = controller()->motionPool()[0].actualPos();
    pos_error     =  pos_target - pos;
    pos_error_sum += pos_error * 0.001;
    Velosity_desired = kp * pos_error + ki * pos_error_sum;
    mout() << "pos: " << pos << std::endl;
    mout() << "pos_error: " << pos_error << std::endl;
    mout() << "pos_error_sum: " << pos_error_sum << std::endl;
    mout() << "Velosity_desired: " << Velosity_desired << std::endl;
    //max acc to set velosity & velosity < max_vel
    if(Velosity_desired > max_vel)Velosity_desired = max_vel;
    if(Velosity_desired - Velosity > step_velosity)Velosity += step_velosity;
    else if (Velosity_desired - Velosity)Velosity -= step_velosity;
    else{Velosity = Velosity_desired;}
    controller()->motionPool()[0].setTargetVel(Velosity);
//    if (Velosity <= 0.01 & abs(pos - pos_target) <=0.01)return 0;
    mout() << "velosity" << Velosity << std::endl;
    if (abs(Velosity) <= step_velosity && abs(pos_error) <=0.1){controller()->motionPool()[0].setTargetVel(0);return 0;}
    return 1;

}
MoveVelosity::MoveVelosity(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"mvv\">"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
MoveVelosity::~MoveVelosity() = default;

double* vec2so3(double *a){//a is omga(w)
    double* so3mat = new double[9];
    so3mat[0] =     0;    so3mat[1] = -a[2];    so3mat[2] =  a[1];
    so3mat[3] =  a[2];    so3mat[4] =     0;    so3mat[5] = -a[0];
    so3mat[6] = -a[1];    so3mat[7] =  a[0];    so3mat[8] =     0;
    return so3mat;
}

double* so32vec(double *a){
    double* w = new double[3];
    w[0] = a[7];
    w[1] = a[2];
    w[2] = a[3];
    return w;
}

double* vec2se3(double v[6]){
    double* se3mat = new double[16];
    se3mat[0] =     0; se3mat[1] = -v[2]; se3mat[2] =  v[1];  se3mat[3] = v[3];
    se3mat[4] =  v[2]; se3mat[5] =     0; se3mat[6] = -v[0];  se3mat[7] = v[4];
    se3mat[8] = -v[1]; se3mat[9] =  v[0]; se3mat[10] =    0; se3mat[11] = v[5];
    se3mat[12] =    0; se3mat[13] =    0; se3mat[14] =    0; se3mat[15] =    0;
    return se3mat;
}

double* matrixexp3(double so3[9]){
    double *R = new double[9];
    double *omgtheta = robot::so32vec(so3);
//    std::cout << "omgtheta: " << omgtheta[0] << " " << omgtheta[1] << " " << omgtheta[2] << std::endl;
    double theta = sqrt(omgtheta[0]*omgtheta[0] + omgtheta[1]*omgtheta[1] + omgtheta[2]*omgtheta[2]);
//    std::cout << "theta" << theta <<std::endl;
    if(abs(theta) <=0.01){
        R[0] = 1; R[1] = 0; R[2] = 0;
        R[3] = 0; R[4] = 1; R[5] = 0;
        R[6] = 0; R[7] = 0; R[8] = 1;
    }
    else{
        double omgmat[9];
        omgmat[0] =              0; omgmat[1] = so3[1] / theta; omgmat[2] = so3[2] / theta;
        omgmat[3] = so3[3] / theta; omgmat[4] = 0             ; omgmat[5] = so3[5] / theta;
        omgmat[6] = so3[6] / theta; omgmat[7] = so3[7] / theta; omgmat[8] =              0;
//        std::cout << std::endl;
//        for(int i=0; i<9; i++){
//            std::cout << "so3omgmat" << i << ": " << omgmat[i] << "  ";
//        }
//        std::cout << std::endl;
        double a = sin(theta); double b = 1 - cos(theta);
        R[0] = 1 + (omgmat[1]*omgmat[3] + omgmat[2]*omgmat[6])*b; R[1] = omgmat[1]*a + omgmat[2]*omgmat[7]*b;               R[2] = omgmat[2]*a + omgmat[1]*omgmat[5]*b;
        R[3] = omgmat[3]*a + omgmat[5]*omgmat[6]*b;               R[4] = 1 + (omgmat[1]*omgmat[3] + omgmat[5]*omgmat[7])*b; R[5] = omgmat[5]*a + omgmat[3]*omgmat[2]*b;
        R[6] = omgmat[6]*a + omgmat[7]*omgmat[3]*b;               R[7] = omgmat[7]*a + omgmat[6]*omgmat[1]*b;               R[8] = 1 + (omgmat[6]*omgmat[2] + omgmat[7]*omgmat[5])*b;
    }
    return R;
}

double* matrixexp6(double se3[16]){
    double* T = new double[16];
    double omgtheta[3];
    omgtheta[0] = se3[9];
    omgtheta[1] = se3[2];
    omgtheta[2] = se3[4];
    double norm = sqrt(omgtheta[0] * omgtheta[0] + omgtheta[1] * omgtheta[1] + omgtheta[2] * omgtheta[2]);
    if(abs(norm) <=0.001){
        T[0]  = 1; T[1]  = 0; T[2]  = 0; T[3]  = 0;
        T[4]  = 0; T[5]  = 1; T[6]  = 0; T[7]  = 0;
        T[8]  = 0; T[9]  = 0; T[10] = 1; T[11] = 0;
        T[12] = 0; T[13] = 0; T[14] = 0; T[15] = 1;
    }
    else{
        double theta = norm;
//        std::cout << "norm: " << norm << std::endl;
        double a = 1 - cos(theta); double b = theta - sin(theta);
//        double omghat[3];
//        omghat[0] = omgtheta[0] / theta; omghat[1] = omgtheta[1] / theta; omghat[2] = omgtheta[2] / theta;
        double omgmat[9];
        omgmat[0] =              0; omgmat[1] = se3[1] / theta; omgmat[2] = se3[2] / theta;
        omgmat[3] = se3[4] / theta; omgmat[4] = 0             ; omgmat[5] = se3[6] / theta;
        omgmat[6] = se3[8] / theta; omgmat[7] = se3[9] / theta; omgmat[8] =              0;
//        for(int i=0; i<9; i++){
//            std::cout << "omgmat" << i << ": " << omgmat[i] << "  ";
//        }
        double R[9];
        R[0] =      0; R[1] = se3[1]; R[2] = se3[2];
        R[3] = se3[4]; R[4] = 0     ; R[5] = se3[6];
        R[6] = se3[8]; R[7] = se3[9]; R[8] =      0;
        double *Rt = robot::matrixexp3(R);
//        std::cout << std::endl;
        T[0] = Rt[0]; T[1] = Rt[1]; T[2]  = Rt[2]; T[3]  = ((omgmat[1]*omgmat[3] + omgmat[2]*omgmat[6]) * b + theta) * se3[3] / theta + ((omgmat[2]*omgmat[7]) * b + omgmat[1] * a) * se3[7] / theta         + ((omgmat[1]*omgmat[5]) *b+ omgmat[2] *a ) * se3[11] / theta;
        T[4] = Rt[3]; T[5] = Rt[4]; T[6]  = Rt[5]; T[7]  = ((omgmat[5]*omgmat[6]) * b + omgmat[3] * a) * se3[3] / theta         + ((omgmat[3]*omgmat[1] + omgmat[5]*omgmat[7]) * b + theta) * se3[7] / theta + ((omgmat[3]*omgmat[2]) *b + omgmat[5] *a ) * se3[11] / theta;
        T[8] = Rt[6]; T[9] = Rt[7]; T[10] = Rt[8]; T[11] = ((omgmat[7]*omgmat[3]) * b + omgmat[6] * a) * se3[3] / theta         + ((omgmat[6]*omgmat[1]) * b + omgmat[7] * a) * se3[7] / theta         + ((omgmat[6]*omgmat[2] + omgmat[7]*omgmat[5]) *b + theta) * se3[11] / theta;
        T[12]= 0; T[13] = 0; T[14] = 0; T[15] = 1;
    }
    return T;
}

double* transtorp(double T[16]){
    double *rp = new double[12];
    rp[0] = T[0]; rp[1] = T[1]; rp[2] = T[2];
    rp[3] = T[4]; rp[4] = T[5]; rp[5] = T[6];
    rp[6] = T[8]; rp[7] = T[9]; rp[8] = T[10];
    rp[9] = T[3]; rp[10] = T[7]; rp[11] = T[11];
    return rp;
}

double* transinv(double T[16]){
    double* inv = new double[16];
    double *rp = robot::transtorp(T);
    inv[0] = rp[0]; inv[1] = rp[3]; inv[2] = rp[6]; inv[3] = -(inv[0]*inv[9] + inv[3]*inv[10] + inv[6]*inv[11]);
    inv[4] = rp[1]; inv[5] = rp[4]; inv[6] = rp[7]; inv[7] = -(inv[1]*inv[9] + inv[4]*inv[10] + inv[7]*inv[11]);
    inv[8] = rp[2]; inv[9] = rp[5]; inv[10] = rp[8]; inv[11] = -(inv[2]*inv[9] + inv[5]*inv[10] + inv[8]*inv[11]);
    inv[12] = 0   ; inv[13] =    0; inv[14] =     0; inv[15] = 1;
//    std::cout << "here" ;
    return inv;
}

double trace(double R[9]){
    double trace = R[0] + R[4] +R[8];
    return trace;
}

double* matrixlog3(double R[9]){
    double acosinput = (trace(R) - 1) /2;
    double *so3mat = new double[9];
    double omg[3] = {0,0,0};
    if(acosinput >= 1){
        so3mat[0] = 0; so3mat[1] = 0; so3mat[2] = 0;
        so3mat[3] = 0; so3mat[4] = 0; so3mat[5] = 0;
        so3mat[6] = 0; so3mat[7] = 0; so3mat[8] = 0;
    }
    else if(acosinput <= -1){
        if(abs(1 + R[8]) <= 0.01){
            double k = 1/sqrt(2*(1+R[8]));
            omg[0] = k * R[2]; omg[1] = k * R[5]; omg[2] = k * (1 + R[8]);
        }
        else if(abs(1 + R[4]) <= 0.01){
            double k = 1/sqrt(2*(1+R[4]));
            omg[0] = k * R[1]; omg[1] = k * (R[4] + 1); omg[2] = k * R[7];
        }
        else{
            double k = 1/sqrt(2*(1+R[0]));
            omg[0] = k * (R[0] + 1); omg[1] = k * R[3]; omg[2] = k * R[6];
        }
        for(int i = 0;i<3; i++)omg[i] = PI *omg[i];
        so3mat = robot::vec2so3(omg);
    }
    else{
        double theta = acos(acosinput);
        double kk = theta / (2*sin(theta));
        so3mat[0] = 0; so3mat[1] = (R[1] - R[3])*kk; so3mat[2] = (R[2] - R[6])*kk;
        so3mat[3] = (R[3] - R[1])*kk; so3mat[4] = 0; so3mat[5] = (R[5] - R[7])*kk;
        so3mat[6] = (R[6] - R[2])*kk; so3mat[7] = (R[7] - R[5])*kk; so3mat[8] = 0;
    }
    return so3mat;
}

double* matrixlog6(double T[16]){
    double *expmat = new double[16];
    double *rp = robot::transtorp(T);
    double R[9];
    double p[3];
    for(int i = 0;i<9;i++)R[i] = rp[i];
    for(int i = 0;i<3;i++)p[i] = rp[i + 9];
//    std::cout << "R and p: " << std::endl;
//    for(int i = 0;i<9;i++)std::cout << "R" << i << ": " << R[i] <<std::endl;
//    for(int i = 0;i<3;i++)std::cout << "p" << i << ": " << p[i] <<std::endl;
    double *omgmat = robot::matrixlog3(R);
    if(sqrt(omgmat[0]*omgmat[0] + omgmat[1]*omgmat[1] + omgmat[2]*omgmat[2] + omgmat[3]*omgmat[3] + omgmat[4]*omgmat[4] +
            omgmat[5]*omgmat[5] + omgmat[6]*omgmat[6] + omgmat[7]*omgmat[7] + omgmat[8]*omgmat[8]) <=0.001){
        expmat[0] = 0; expmat[1] = 0; expmat[2] = 0; expmat[3] = T[3];
        expmat[4] = 0; expmat[5] = 0; expmat[6] = 0; expmat[7] = T[7];
        expmat[8] = 0; expmat[9] = 0; expmat[10] = 0; expmat[11] = T[11];
        expmat[12] = 0; expmat[13] = 0; expmat[14] = 0; expmat[15] = 0;
    }
    else{
        double theta = acos((trace(R) - 1) / 2);
//        std::cout << "theta: " << theta <<std::endl;
        double k     = (1/theta - (1/tan(theta/2))/2)/theta;
        expmat[0] = omgmat[0]; expmat[1] = omgmat[1]; expmat[2] = omgmat[2]; expmat[3] = (1 - omgmat[0]/2 + k*(omgmat[0]*omgmat[0] + omgmat[1]*omgmat[3] + omgmat[2]*omgmat[6])) * p[0] + ((-omgmat[1]/2) + k*(omgmat[0]*omgmat[1] + omgmat[1]*omgmat[4] + omgmat[2]*omgmat[7])) * p[1] + ((-omgmat[2]/2) + k*(omgmat[0]*omgmat[2] + omgmat[1]*omgmat[5] + omgmat[2]*omgmat[8])) * p[2];
        expmat[4] = omgmat[3]; expmat[5] = omgmat[4]; expmat[6] = omgmat[5]; expmat[7] = (-omgmat[3]/2 + k*(omgmat[3]*omgmat[0] + omgmat[4]*omgmat[3] + omgmat[5]*omgmat[6])) * p[0] + (1 - omgmat[4]/2 +(omgmat[3]*omgmat[1] + omgmat[4]*omgmat[4] + omgmat[5]*omgmat[7])) * p[1] + (-omgmat[5]/2 +k*(omgmat[3]*omgmat[2] + omgmat[4]*omgmat[5] + omgmat[5]*omgmat[8])) * p[2];
        expmat[8] = omgmat[6]; expmat[9] = omgmat[7]; expmat[10] = omgmat[8]; expmat[11] = (-omgmat[6]/2 + k*(omgmat[6]*omgmat[0] + omgmat[7]*omgmat[3] + omgmat[8]*omgmat[6])) * p[0] + (-omgmat[7]/2 +(omgmat[6]*omgmat[1] + omgmat[7]*omgmat[4] + omgmat[8]*omgmat[7])) * p[1] + (1 - omgmat[8]/2 +k*(omgmat[6]*omgmat[2] + omgmat[7]*omgmat[5] + omgmat[8]*omgmat[8])) * p[2];;
        expmat[12] = 0; expmat[13] = 0; expmat[14] = 0; expmat[15] = 0;
    }
    return expmat;
}

double* se3tovec(double se3mat[16]){
    double *V = new double[6];
    V[0] = se3mat[9];
    V[1] = se3mat[2];
    V[2] = se3mat[4];
    V[3] = se3mat[3];
    V[4] = se3mat[7];
    V[5] = se3mat[11];
    return V;
}

double* adjoint(double T[16]){
    double *rp = robot::transtorp(T);
    double p[3] = {rp[9], rp[10], rp[11]};
    double *s = robot::vec2so3(p);
    double *adt = new double[36];
    adt[0]  = T[0]; adt[1]  = T[1]; adt[2]  = T[2];  adt[3]  = 0; adt[4]  = 0; adt[5]  = 0;
    adt[6]  = T[4]; adt[7]  = T[5]; adt[8]  = T[6];  adt[9]  = 0; adt[10] = 0; adt[11] = 0;
    adt[12] = T[8]; adt[13] = T[9]; adt[14] = T[10]; adt[15] = 0; adt[16] = 0; adt[17] = 0;
    adt[18] = s[0]*rp[0] + s[1]*rp[3] + s[2]*rp[6]; adt[19] = s[0]*rp[1] + s[1]*rp[4] + s[2]*rp[7]; adt[20] = s[0]*rp[2] + s[1]*rp[5] + s[2]*rp[8]; adt[21] = rp[0]; adt[22] = rp[1]; adt[23] = rp[2];
    adt[24] = s[3]*rp[0] + s[4]*rp[3] + s[5]*rp[6]; adt[25] = s[3]*rp[1] + s[4]*rp[4] + s[5]*rp[7]; adt[26] = s[3]*rp[2] + s[4]*rp[5] + s[5]*rp[8]; adt[27] = rp[3]; adt[28] = rp[4]; adt[29] = rp[5];
    adt[30] = s[6]*rp[0] + s[7]*rp[3] + s[8]*rp[6]; adt[31] = s[6]*rp[1] + s[7]*rp[4] + s[8]*rp[7]; adt[32] = s[6]*rp[2] + s[7]*rp[5] + s[8]*rp[8]; adt[33] = rp[6]; adt[34] = rp[7]; adt[35] = rp[8];
//    for(int i = 0;i<36;i++){
//        if(i%6 == 0)std::cout << std::endl;

//        std::cout << adt[i] << " " ;
//    }
    return adt;
}

double* jacobianbody(double blist[42], double thetalist[7]){
    double *jb = new double[42];
    double T[16] = {1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1};
    for(int i = 6; i>=1; i--){
        double vec[6];//blist(:,i)
        vec[0] = -blist[6*i] * thetalist[i];
        vec[1] = -blist[6*i + 1] * thetalist[i];
        vec[2] = -blist[6*i + 2] * thetalist[i];
        vec[3] = -blist[6*i + 3] * thetalist[i];
        vec[4] = -blist[6*i + 4] * thetalist[i];
        vec[5] = -blist[6*i + 5] * thetalist[i];
        double *F = robot::matrixexp6(vec2se3(vec));
        double Tout[16];
        s_pm_dot_pm(T,F,Tout);
//        aris::dynamic::dsp(4,4,Tout);
        for(int i = 0; i<16; i++){T[i] = Tout[i]; };
        double *adj = adjoint(Tout);
        jb[6*(i-1)]     =  adj[0] * blist[6*(i-1)] + adj[1]  * blist[6*(i-1) + 1] + adj[2] * blist[6*(i-1) + 2] + adj[3] * blist[6*(i-1) + 3] + adj[4] * blist[6*(i-1) + 4] + adj[5] * blist[6*(i-1) + 5];
        jb[6*(i-1) + 1] =  adj[6] * blist[6*(i-1)] + adj[7]  * blist[6*(i-1) + 1] + adj[8] * blist[6*(i-1) + 2] + adj[9] * blist[6*(i-1) + 3] + adj[10] * blist[6*(i-1) + 4] + adj[11] * blist[6*(i-1) + 5];
        jb[6*(i-1) + 2] = adj[12] * blist[6*(i-1)] + adj[13] * blist[6*(i-1) + 1] + adj[14] * blist[6*(i-1) + 2] + adj[15] * blist[6*(i-1) + 3] + adj[16] * blist[6*(i-1) + 4] + adj[17] * blist[6*(i-1) + 5];
        jb[6*(i-1) + 3] = adj[18] * blist[6*(i-1)] + adj[19] * blist[6*(i-1) + 1] + adj[20] * blist[6*(i-1) + 2] + adj[21] * blist[6*(i-1) + 3] + adj[22] * blist[6*(i-1) + 4] + adj[23] * blist[6*(i-1) + 5];
        jb[6*(i-1) + 4] = adj[24] * blist[6*(i-1)] + adj[25] * blist[6*(i-1) + 1] + adj[26] * blist[6*(i-1) + 2] + adj[27] * blist[6*(i-1) + 3] + adj[28] * blist[6*(i-1) + 4] + adj[29] * blist[6*(i-1) + 5];
        jb[6*(i-1) + 5] = adj[30] * blist[6*(i-1)] + adj[31] * blist[6*(i-1) + 1] + adj[32] * blist[6*(i-1) + 2] + adj[33] * blist[6*(i-1) + 3] + adj[34] * blist[6*(i-1) + 4] + adj[35] * blist[6*(i-1) + 5];
    }
    jb[36] = blist[36];
    jb[37] = blist[37];
    jb[38] = blist[38];
    jb[39] = blist[39];
    jb[40] = blist[40];
    jb[41] = blist[41];
    return jb;
}

double* fkinbody(double M[16], double blist[42], double thetalist[7]){
    double *T = new double[16];
    for(int i = 0; i<16; i++){
        T[i] = M[i];
    }
    for(int i = 0; i<=6; i++){
        double vec[6];//blist(:,i)
//        std::cout << "i: " << i <<std::endl;
        vec[0] = blist[6*i] * thetalist[i];
        vec[1] = blist[6*i + 1] * thetalist[i];
        vec[2] = blist[6*i + 2] * thetalist[i];
        vec[3] = blist[6*i + 3] * thetalist[i];
        vec[4] = blist[6*i + 4] * thetalist[i];
        vec[5] = blist[6*i + 5] * thetalist[i];
        double *F = robot::matrixexp6(vec2se3(vec));
        double Tout[16];
        s_pm_dot_pm(T,F,Tout);
//        aris::dynamic::dsp(4,4,Tout);
        for(int i = 0; i<16; i++){T[i] = Tout[i]; };
    }
    return T;
}

double* ikbody(double blist[42], double M[16], double T[16], double thetalist0[7]){
    double *thetalist = new double[7];
    double *blist_ = new double[42];
    std::copy(thetalist0, thetalist0 + 7, thetalist);
    std::copy(blist, blist + 42, blist_);
    double *fkb = fkinbody(M, blist_, thetalist);
    double fkbpm[16];
    for(int i = 0; i<16; i++){
        fkbpm[i] = fkb[i];
        std::cout << "fkbpm: " <<fkbpm[i] << std::endl;
    }
    double pmout[16];
    s_pm_dot_pm(fkbpm, T, pmout);
    double *trans_inv = transinv(pmout);
    double tra_inv[16];
    for(int i = 0; i<16; i++){
        tra_inv[i] = trans_inv[i];
    }
    double *mat_log6 = matrixlog6(tra_inv);
    double log6[16];
    for(int i = 0; i<16; i++){
        log6[i] = mat_log6[i];
        std::cout << "log6: " <<log6[i] << std::endl;
    }
    double *vb = se3tovec(log6);
    int i = 0;
    int maxinterations = 5;
    int err = sqrt(vb[0]*vb[0] + vb[1]*vb[1] + vb[2]*vb[2]) > 0.01 || sqrt(vb[3]*vb[3] + vb[4]*vb[4] + vb[5]*vb[5]) > 0.001;
    std::cout << "err: " << err <<std::endl;
    while(err && i <= maxinterations ){
        std::cout << "enter while";
        double *jacb = jacobianbody(blist, thetalist);
        Eigen::MatrixXd A(6,7);
        A<<     jacb[0], jacb[6], jacb[12], jacb[18], jacb[24], jacb[30], jacb[36],
                jacb[1], jacb[7], jacb[13], jacb[19], jacb[25], jacb[31], jacb[37],
                jacb[2], jacb[8], jacb[14], jacb[20], jacb[26], jacb[32], jacb[38],
                jacb[3], jacb[9], jacb[15], jacb[21], jacb[27], jacb[33], jacb[39],
                jacb[4], jacb[10], jacb[16], jacb[22], jacb[28], jacb[34], jacb[40],
                jacb[5], jacb[11], jacb[17], jacb[23], jacb[29], jacb[35], jacb[41];
        Eigen::MatrixXd pinv  = pinv_eigen_based(A);
        thetalist[0] = thetalist[0] + pinv(0,0)*vb[0] + pinv(0,1)*vb[1] + pinv(0,2)*vb[2] + pinv(0,3)*vb[3] + pinv(0,4)*vb[4] + pinv(0,5)*vb[5];
        thetalist[1] = thetalist[1] + pinv(1,0)*vb[0] + pinv(1,1)*vb[1] + pinv(1,2)*vb[2] + pinv(1,3)*vb[3] + pinv(1,4)*vb[4] + pinv(1,5)*vb[5];
        thetalist[2] = thetalist[2] + pinv(2,0)*vb[0] + pinv(2,1)*vb[1] + pinv(2,2)*vb[2] + pinv(2,3)*vb[3] + pinv(2,4)*vb[4] + pinv(2,5)*vb[5];
        thetalist[3] = thetalist[3] + pinv(3,0)*vb[0] + pinv(3,1)*vb[1] + pinv(3,2)*vb[2] + pinv(3,3)*vb[3] + pinv(3,4)*vb[4] + pinv(3,5)*vb[5];
        thetalist[4] = thetalist[4] + pinv(4,0)*vb[0] + pinv(4,1)*vb[1] + pinv(4,2)*vb[2] + pinv(4,3)*vb[3] + pinv(4,4)*vb[4] + pinv(4,5)*vb[5];
        thetalist[5] = thetalist[5] + pinv(5,0)*vb[0] + pinv(5,1)*vb[1] + pinv(5,2)*vb[2] + pinv(5,3)*vb[3] + pinv(5,4)*vb[4] + pinv(5,5)*vb[5];
        thetalist[6] = thetalist[6] + pinv(6,0)*vb[0] + pinv(6,1)*vb[1] + pinv(6,2)*vb[2] + pinv(6,3)*vb[3] + pinv(6,4)*vb[4] + pinv(6,5)*vb[5];
        double *fkb_temp = fkinbody(M,blist,thetalist);
        double fkb_temp_pm[16];
        for(int i = 0; i<16; i++)fkb_temp_pm[i] = fkb_temp[i];
        double Tout[16];
        s_pm_dot_pm(fkb_temp_pm,T,Tout);
        double *tr = transinv(Tout);
        double tr_temp[16];
        for(int i = 0; i<16; i++)tr_temp[i] = tr[i];
        double *mat_log = matrixlog6(tr_temp);
        double mat_log_temp[16];
        for(int i = 0; i<16; i++)mat_log_temp[i] = mat_log[i];
        double *vb_out = se3tovec(mat_log_temp);
        for(int i = 0; i<6; i++)vb[i] = vb_out[i];
//        std::cout << "OK" << std::endl;
        err = sqrt(vb[0]*vb[0] + vb[1]*vb[1] + vb[2]*vb[2]) > 0.01 || sqrt(vb[3]*vb[3] + vb[4]*vb[4] + vb[5]*vb[5]) > 0.001;
        if(err == 0) {std::cout << "err: " << err <<std::endl;}
        i++;
        std::cout << "i: " << i << std::endl;
    }
    return thetalist;
}

static double M[16]={1,0,0,0,
                     0,1,0,-98,
                     0,0,1,150,
                     0,0,0,1};
static double blist[42] = {0,0,1,98,0,0,
                           0,1,0,203,0,0,
                           0,0,1,232,0,0,
                           0,-1,0,247,0,0,
                           0,0,1,-98,0,0,
                           0,-1,0, -53,0,0,
                           0,0,1,0,0,0};


static double thetalist[7] = {0,0,0,0,0,0,0};
static double Tsd[16] = {0,0,-1,53,
                         0,1,0,-98,
                         1,0,1,150,
                         0,0,0,1};

int* test(int arr[3]){
    int* out = new int[3];
    return out;
}
//move seven robot velosity //
struct MoveLSParam
{
//    static double M[16];
//    static double blist[42];
//    static double thetalist[7];
//    static double Tsd[16];
};
auto MoveL::prepareNrt()->void
{

}
auto MoveL::executeRT()->int
{
//    double *fk = fkinbody(M,blist,thetalist);
//    aris::dynamic::dsp(4,4,fk);
    std::cout << "ml";
    double *ik = ikbody(blist,M,Tsd,thetalist);
    aris::dynamic::dsp(1,7,ik);
//    for(int i = 0; i<7; i++)std::cout << "ik: " << ik[i] << std::endl;
    delete ik;
    return 0;

}
MoveL::MoveL(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"ml\">"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
MoveL::~MoveL() = default;



auto createControllerSevenRobot()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 1; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[7]
        {
            0,0,0,0,0,0
        };
#else
        double pos_offset[7]//link to 80T
        {
            3.74533393586124   ,
            10.77265572006922   ,
            -4.95927599764589   ,
            8.43104603159333   ,
            0.50602191240234   ,
            791.22560061346076  ,
            1011.58989831794599
//            0,
//            0,
//            0,
//            0,
//            0,
//            0,
//            0
        };
#endif
        double pos_factor[7]
        {
            16384.0 * 32 / 2 / PI,
            16384.0 * 32 / 2 / PI,
            16384.0 * 32 / 2 / PI,
            16384.0 * 32 / 2 / PI,
            16384.0 * 32 / 2 / PI,
            16384.0 * 32 / 2 / PI,
            16384.0 * 32 / 2 / PI
        };
        double max_pos[7]
        {
            PI,PI,PI,PI,PI,PI/2.0,PI
        };
        double min_pos[7]
        {
           -PI,-PI,-PI,-PI,-PI,-PI/2.0,-PI
        };
        double max_vel[7]
        {
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI,
        };
        double max_acc[7]
        {
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,
        };

        int phy_id[7]={0,1,2,3,4,5,6};

        //zero_err
        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1608\" is_tx=\"false\">"
            "				<PdoEntry name=\"controlword\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"targer_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"max_motor_speed\" index=\"0x6080\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"target_vel\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
            "               <PdoEntry name=\"dummy_type\" index=\"0\" subindex=\"0\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1a06\" is_tx=\"true\">"
            "				<PdoEntry name=\"error_code\" index=\"0x603f\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"statusword\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_operation\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
            "				<PdoEntry name=\"dummy_type\" index=\"0\" subindex=\"0\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatMotor>";

       auto& s = controller->slavePool().add<aris::control::EthercatMotor>();
               aris::core::fromXmlString(s, xml_str);
#ifdef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif
#ifndef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif


    };
    return controller;
}


auto createPlanSevenRobot()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();

    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //自己写的命令
    plan_root->planPool().add<SetMaxToq>();
    plan_root->planPool().add<MoveJS>();
    plan_root->planPool().add<MoveJoint>();
    plan_root->planPool().add<MoveTorque>();
    plan_root->planPool().add<ReadJoint>();
    plan_root->planPool().add<MoveVelosity>();
    plan_root->planPool().add<MoveL>();
    plan_root->planPool().add<MoveJ>();
    plan_root->planPool().add<MoveScrew>();
    plan_root->planPool().add<MoveSingleMotor>();
    plan_root->planPool().add<ReadSingleMotor>();
    plan_root->planPool().add<MoveSingleTest>();
    plan_root->planPool().add<DogHome>();
    return plan_root;
}

}


//    double *T = fkinbody(M,blist,thetalist);

//    double *jaco = jacobianbody(blist,thetalist);
//        for(int i=0; i<24; i++){
//            std::cout << i << ": " << jaco[i] << " ";
//        }
//    double A[3] = {1,2,3};
//    double B[6] = {1,2,3,4,5,6};
//    double se[16] = {0,      0,        0,       0,
//                     0,      0,  -1.5708,  2.3562,
//                     0, 1.5708,        0,  2.3562,
//                     0,0,0,0};
//    double TT[16] = {0,1,2,3,
//                     4,5,6,7,
//                     8,9,10,11,
//                     12,13,14,15};
//    double R[9] = {0,0,1,1,0,0,0,1,0};
//    double TTT[16] = {1,0,0,0,
//                      0,0,-1,0,
//                      0,1,0,3,
//                      0,0,0,1};
//    double *jb = robot::adjoint(TTT);
//    double *so = robot::vec2so3(A);
//    double *se1 = robot::vec2se3(A);
//    double *T  = robot::matrixexp6(se);
//    double *r  = robot::matrixlog3(R);
//    double *t  = robot::matrixlog6(TTT);
//    for(int i=0; i<9; i++){
//        std::cout << "so3mat: " << r[i];
//    }
//    for(int i=0; i<36; i++){
//        std::cout << "adt" << i << ": " << jb[i];
//    }
//    delete jb;
//    for(int i=0; i<16; i++){
//        std::cout << "T" << i << ": " << T[i] << "  ";
//    }
//    double *h = robot::transinv(TT);
//    std::cout << std::endl;
//    for(int i=0; i<16; i++){
//        std::cout << "transinv" << i << ": " << h[i] << "  ";
//    }
//    delete so;
//    delete se1;
//    delete T;
//    delete t;








//    //inverse kinematic to calculate the acceleration of every motor
//        //get the jacobian first
//        // 获取雅可比矩阵，并对过奇异点的雅可比矩阵进行特殊处理
//    double pinv[36];
//    {
//        auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
//        fwd.cptJacobiWrtEE();
//            //QR分解求方程的解
//        double U[36], tau[7], tau2[7];
//        aris::Size p[7];
//        Size rank;
//            //auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
//            //A为输入,根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
//        s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-3);
//            //对奇异点进行特殊处理,对U进行处理
//        if (rank < 6)
//        {
//            for (int i = 0; i < 6; i++)
//            {
//                if (U[7 * i] >= 0)
//                {
//                    U[7 * i] = U[7 * i] + 0.1;
//                }
//                else
//                {
//                    U[7 * i] = U[7 * i] - 0.1;
//                }
//            }
//        }
//            // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
//        s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);
//    }
//        //calculate the desired accelaration of every motor
//    s_mm(6,1,6,pinv,next_tcp_v,v_joint);
//    //move the joint
//    for(std::size_t i = 0; i<controller()->motionPool().size(); ++i){
//        controller()->motionPool()[i].setTargetVel(v_joint[i]);
 //   }return 1;
