package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.Constants;

public class PID {
    public void test(){
        PIDController rotorPID = new PIDController(0.13, 0, 0);
        CANcoderConfiguration config = new CANcoderConfiguration();

        rotorPID.enableContinuousInput(-320,320);
    }

}
