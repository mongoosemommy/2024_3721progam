#pragma once

//#include <ctre/phoenix6/CANBus.hpp>
#include <frc/motorcontrol/VictorSP.h>
//#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include <rev/REVCommon.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include <frc/GenericHID.h>
namespace subsystems
{
    class Manipulator
    {
    private:    
        frc::DutyCycleEncoder arm_enc{0& 1}; //ctre mag encoder (bore)
    
     
     
        rev::CANSparkMax arm_l{8, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax arm_r{12 , rev::CANSparkMax::MotorType::kBrushless};

        //ctre::phoenix::motorcontrol::can::WPI_VictorSPX shooter_a{7};
        //ctre::phoenix::motorcontrol::can::WPI_VictorSPX shooter_b{8};
        rev::CANSparkMax shooter_a{4,rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax shooter_b{6, rev::CANSparkMax::MotorType::kBrushless}; // The two motors for the shooter.

        //frc::DigitalInput note_sensor{0};
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX intake_motor{5}; //Bag motor For intake
        //rev::CANSparkMax intake_motor{9, rev::CANSparkMax::MotorType::kBrushless};

        // Singleton pattern
        Manipulator();
        Manipulator(Manipulator const&);



        void operator=(Manipulator const&);
    public:
        double kARM_FLOOR_POS = 0.584;  // intaking
        double kARM_FENDER_POS = 0.53;  // close shot
        double kARM_START_POS = 0.376;  // start config
        double kARM_AMP_POS   = 0.325;  // amp scoring
        
        void move_arm(double power);
        void arm_to_pos(double pos);
        void intake(double power);
        void shoot(double power);
       // bool get_note_sensor();
        double get_arm_enc();
//
        // Singleton pattern
        // Manipulator(Manipulator const&) = delete;
        // void operator=(Manipulator const&) = delete;
        /** Singleton pattern. Call this instead of instantiating this class. **/
        static Manipulator& getInstance();
    };
    
} // namespace subsystems
