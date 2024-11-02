package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

public class PID {
    public void test(){
        PIDController rotorPID = new PIDController(0.13, 0, 0);
        CANcoderConfiguration config = new CANcoderConfiguration();
        rotorPID.enableContinuousInput(-320,320);
        
    }

}
