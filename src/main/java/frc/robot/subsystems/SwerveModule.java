package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import frc.robot.Constants;

public class SwerveModule {
    private CANSparkMax throttle;
    private CANSparkMax rotor;
    public CANcoder encoder; // private in normal
    private PIDController rotorPID = new PIDController(0.013, 0, 0); // self testing, kd not use oftenly and set to 0
    private CANcoderConfiguration config = new CANcoderConfiguration();

    /**
     * Constructor for a SwerveModule.
     * 
     * @param idThrottleMoter The CAN ID of the throttle motor for this module.
     * @param idRotor         The CAN ID of the rotate motor for this module.
     * @param idCANcoder      The CAN ID of the encoder for this module.
     * @param offset          The offset of the encoder for this module. offset 基準值
     */
    public SwerveModule(int idThrottleMoter, int idRotor, int idCANcoder, double offset) {
        throttle = new CANSparkMax(idThrottleMoter, MotorType.kBrushless);
        rotor = new CANSparkMax(idRotor, MotorType.kBrushless);
        encoder = new CANcoder(idCANcoder);
        config.MagnetSensor.AbsoluteSensorRange.compareTo(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        config.MagnetSensor.MagnetOffset = offset;
        encoder.getConfigurator().apply(config);
        // encoder config done
        throttle.restoreFactoryDefaults(); // not required but use it is better
        throttle.setIdleMode(IdleMode.kCoast); // set motor mode(break or coast)
        rotorPID.enableContinuousInput(-180, 180);
    }

    /**
     * Sets the desired state of the swerve module.
     * 
     * @param state The desired state of the swerve module.
     */
    public void setState(SwerveModuleState state) { // setmode(vec,angle)
        SwerveModuleState optimizedAngle = SwerveModuleState.optimize(state, getRotation2d());
        double rotorOutput = rotorPID.calculate(getRotation2d().getDegrees(), optimizedAngle.angle.getDegrees());
        double throttleOutput = optimizedAngle.speedMetersPerSecond / Constants.kThrottleMaxSpeed; // unit:m/s
        throttle.set(throttleOutput);
        rotor.set(-rotorOutput); // code to negative by default
    }

    /**
     * Gets the current rotation of the swerve module.
     * 
     * @return The current rotation of the swerve module.
     */
    private Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue());
    }
}
