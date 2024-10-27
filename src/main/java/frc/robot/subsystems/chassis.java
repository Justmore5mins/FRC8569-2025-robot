// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/*
            x
            ^
            |
            |
            |
y <----------

axis definition of SwerverDriveKinematics()
 */

public class chassis extends SubsystemBase {
  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  //define motors and set motor place
  private SwerveModule moduleA = new SwerveModule(Constants.A_T_ID,Constants.A_R_ID , Constants.A_E_ID, Constants.A_Offset); // A_E means encoder
  private SwerveModule moduleB = new SwerveModule(Constants.B_T_ID,Constants.B_R_ID , Constants.B_E_ID, Constants.B_Offset);
  private SwerveModule moduleC = new SwerveModule(Constants.C_T_ID,Constants.C_R_ID , Constants.C_E_ID, Constants.C_Offset); 
  private SwerveModule moduleD = new SwerveModule(Constants.D_T_ID,Constants.D_R_ID , Constants.D_E_ID, Constants.D_Offset);
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.A_TRANSLATION2D,Constants.B_TRANSLATION2D,Constants.C_TRANSLATION2D,Constants.D_TRANSLATION2D);

  /* GUI output */
  public chassis() {
    SmartDashboard.putNumber("A", moduleA.encoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("B", moduleB.encoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("C", moduleC.encoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("D", moduleD.encoder.getAbsolutePosition().getValue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void drive(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rSpeed){
    drive(new ChassisSpeeds(xSpeed.get(),ySpeed.get(),rSpeed.get()));
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(chassisSpeeds, 0.02); //20ms
    targetSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeed, getRotation2d()); //校正機器人
    SwerveModuleState [] targStates = kinematics.toSwerveModuleStates(targetSpeed);
    setModuleStates(targStates);
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kThrottleMaxSpeed);
    moduleA.setState(states[0]);
    moduleB.setState(states[1]);
    moduleC.setState(states[2]);
    moduleD.setState(states[3]);
  }

  private Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(-gyro.getAngle()); //逆時針為正向 
  }
}
