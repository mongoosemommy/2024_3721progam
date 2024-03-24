#pragma once
#include "../subsystems/drive.hpp"
#include <frc/SPI.h>
#include <rev/REVCommon.h>
#include <rev/CANSparkMax.h>

#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace subsystems
{
    class Drive
    {
    private:

    double Distance;
    double Speed;
    double Posi;
    double Placeholder;
    int Spd;
        //ctre::phoenix::motorcontrol::can::WPI_TalonSRX L1{3};
        //ctre::phoenix::motorcontrol::can::WPI_TalonSRX L2{4};
        //ctre::phoenix::motorcontrol::can::WPI_TalonSRX R1{1};
       //ctre::phoenix::motorcontrol::can::WPI_TalonSRX R2{2};
        
        rev::CANSparkMax L1{7, rev::CANSparkMax::MotorType::kBrushless};	
        rev::CANSparkMax L2{3, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax R1{9, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax R2{2, rev::CANSparkMax::MotorType::kBrushless};	

        
        //"rev::SparkRelativeEncoder" declares the type of encoder
        //"L1_encoder" names the encoder
        // Grabs the encoder for the specific machine
        
      
        // Singleton pattern
        Drive();
        Drive(Drive const&);
        void operator=(Drive const&);
    public:
        void move(double power, double steering);
        //void gyro_drive(double max_speed, double heading);
        //void zero_gyro();
        void move_forward(double Distance, double Speed);//By having it under the Drive class it's easier to wrok with.

        void move_reverse( double Distance, double Speed);//Exist only to classify move reverse under the 'Drive' Class

        void Turn_Right(double Deg, int Volts);/*{ //Turn right.
            if(Deg > Placeholder){ //the reason we use > is because right is consider positive and vice versa
                Volts = Spd; //Converts The 'function double' To the 'classifer double'
                L1.SetInverted(true); //sets the left half of the drive to reverse
                L2.SetInverted(true); //by doing this we spin the robot right

                //This is done after inverting to prevent movement before inversion 
                R1.SetVoltage(units::voltage::volt_t(Spd));//Allows Custom values to be put into the
                R2.SetVoltage(units::voltage::volt_t(Spd));//"Speed" of the motors AKA the power
                L1.SetVoltage(units::voltage::volt_t(Spd));
                L2.SetVoltage(units::voltage::volt_t(Spd));



            }else{
                R1.StopMotor();//Stops all motors  when we reach the desired degree
                R2.StopMotor();
                L1.StopMotor();
                R2.StopMotor();

                L1.SetInverted(false);//Returns the inverse half to it's original state
                L2.SetInverted(false);//this is done after we stop the motors to prevent unwanted movement

            }

        }*/

        void Turn_Left(double Deg, int Volts);/*{ //Turn left.
            if(Deg < Placeholder){ //the reason we use < is because left is considered negative and vice versa
                Volts = Spd; //Converts The 'function double' To the 'classifer double'
                R1.SetInverted(true); //sets the left half of the drive to reverse
                R2.SetInverted(true); //by doing this we spin the robot right

                //This is done after inverting to prevent movement before inversion 
                R1.SetVoltage(units::voltage::volt_t(Spd));//Allows Custom values to be put into the
                R2.SetVoltage(units::voltage::volt_t(Spd));//"Speed" of the motors AKA the power
                L1.SetVoltage(units::voltage::volt_t(Spd));
                L2.SetVoltage(units::voltage::volt_t(Spd));



            }else{
                R1.StopMotor();//Stops all motors  when we reach the desired degree
                R2.StopMotor();
                L1.StopMotor();
                R2.StopMotor();

                R1.SetInverted(false);//Returns the inverse half to it's original state
                R2.SetInverted(false);//this is done after we stop the motors to prevent unwanted movement

            }

        }*/


        // Singleton pattern
        // Drive(Drive const&) = delete;
        // void operator=(Drive const&) = delete;
        /** Singleton pattern. Call this instead of instantiating this class. **/
        static Drive& getInstance();
    };
    
} // namespace subsystems