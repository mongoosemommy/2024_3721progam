#include "Drive.hpp"

namespace subsystems
{
    //Alternate code: 
    /*void Drive:Go(double Distance, double +-Speed)
        if (Distance > Posi){
            R1.Set(Speed);
            R2.Set(Speed);
            L1.Set(Speed);
            L2.Set(Speed);
        } else{

            R1.StopMotor();
            R2.StopMotor();
            L1.StopMotor();
            L2.StopMotor();
        }
    
    
    
    */
    // Singleton
    Drive::Drive() {
    }
   /* void Drive::move_forward(double Distance, double Volts){ //Drive the robot forward
        if (Distance > Posi) //sees if the destination is ariveved at or not
        {   Volts = Spd;
            //what is the recommened voltage?
            R1.SetVoltage(units::voltage::volt_t(Spd));
            R2.SetVoltage(units::voltage::volt_t(Spd));
            L1.SetVoltage(units::voltage::volt_t(Spd));
            L2.SetVoltage(units::voltage::volt_t(Spd));
            //Drives the motors forward by powering them.
        } else {
            R1.StopMotor();
            R2.StopMotor();
            L1.StopMotor();
            R2.StopMotor();
            //stops the motors when we meet the condition for  the if statement
        } 
    }
    void Drive::move_reverse( double Distance, double Volts) { //A command to reverse the drive.
        if (Distance < Posi){
            Volts = Spd;

            R1.SetInverted(true);
            R2.SetInverted(true);
            L1.SetInverted(true);
            L2.SetInverted(true);
            //This inverts the motors so that when powered they drive backwards

            R1.SetVoltage(units::voltage::volt_t(Spd));
            R2.SetVoltage(units::voltage::volt_t(Spd));
            L1.SetVoltage(units::voltage::volt_t(Spd));
            L2.SetVoltage(units::voltage::volt_t(Spd));
            //Tweleve volts is the maximum speed that we set our motors too

        } else {
            R1.StopMotor();
            R2.StopMotor();
            L1.StopMotor();
            R2.StopMotor();
            //This stops the motors when we've reached our destination

            R1.SetInverted(false);
            R2.SetInverted(false);
            L1.SetInverted(false);
            L2.SetInverted(false);
            //Returns the motors to their original state AKA not inverted


            
        }

    }; */

    
    
    Drive& Drive::getInstance()
    {
        static Drive instance;
        return instance;
    }
    
    //void Drive::zero_gyro()
    //{
        //this->navx->ZeroYaw();
    //}
    
    void Drive::move(double power, double steering) //This is genius. Unironicly
    {
        
        double lpower = power + steering;
        double rpower = power - steering;
        
        this->L1.Set(lpower);
        this->L2.Set(lpower);

        this->R1.Set(-rpower);
        this->R2.Set(-rpower);
    }

    /**void Drive::gyro_drive(double max_speed, double heading)
    {
        double Kp = 0.015;
        double error = heading - this->navx->GetAngle();
        double steering = Kp * error;
        this->move(max_speed, steering);
    }*/
    
} // namespace subsystems
