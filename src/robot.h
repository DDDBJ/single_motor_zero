

#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{

    class SetMaxToq : public aris::core::CloneObject<SetMaxToq, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMaxToq();
        explicit SetMaxToq(const std::string &name = "set_max_toq");
    };

    class ReadJoint :public aris::core::CloneObject<ReadJoint, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~ReadJoint();
        explicit ReadJoint(const std::string &name = "dog_read_joint");
    };

    // 正弦曲线 //
    class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        explicit MoveJS(const std::string& name = "MoveJS_plan");

    };

    class DogHome :public aris::core::CloneObject<DogHome, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogHome();
        explicit DogHome(const std::string &name = "dog_home");
    };
    
    //move joint pos
    class MoveJoint : public aris::core::CloneObject<MoveJoint, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        explicit MoveJoint(const std::string &name = "MoveJiont");
    private:
        double dir_;
    };

    //move joint pos
    class MoveJ : public aris::core::CloneObject<MoveJ, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        explicit MoveJ(const std::string &name = "MoveJ");
    private:
        double dir_;
    };

    //torque control
    class MoveTorque :public aris::core::CloneObject<MoveTorque, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~MoveTorque();
        explicit MoveTorque(const std::string &name = "move_torque");
    private:
        double dir_;
    };

    //velosity control single motor
    class MoveVelosity :public aris::core::CloneObject<MoveVelosity, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~MoveVelosity();
        explicit MoveVelosity(const std::string &name = "move_velosity");
    private:
        double dir_;
    };

    //velosity control seven robot
    class MoveL :public aris::core::CloneObject<MoveL, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~MoveL();
        explicit MoveL(const std::string &name = "move_l");
    private:
        double dir_;
    };

    class MoveScrew :public aris::core::CloneObject<MoveScrew, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~MoveScrew();
        explicit MoveScrew(const std::string &name = "move_screw");
    private:
        double dir_;
    };

    class MoveSingleMotor :public aris::core::CloneObject<MoveSingleMotor, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~MoveSingleMotor();
        explicit MoveSingleMotor(const std::string &name = "move_single_motor");
    private:
        double dir_;
        int m_id;
        double dist;
    };


    class MoveSingleTest :public aris::core::CloneObject<MoveSingleMotor, aris::plan::Plan>{
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~MoveSingleTest();
        explicit MoveSingleTest(const std::string &name = "move_single_test");
    private:
        double dir_;
        int m_id;
        double dist;
    };

    class ReadSingleMotor :public aris::core::CloneObject<ReadSingleMotor, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~ReadSingleMotor();
        explicit ReadSingleMotor(const std::string &name = "move_single_motor");
    private:
        double dir_;
        int m_id;
        double dist;
    };

    auto createControllerSevenRobot()->std::unique_ptr<aris::control::Controller>;
    auto createPlanSevenRobot()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
