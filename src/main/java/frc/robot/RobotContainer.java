// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.chassis;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  public final chassis chassis = new chassis();

  private XboxController controller = new XboxController(0);

  public RobotContainer() {
    chassis.setDefaultCommand(new RunCommand(() -> chassis.drive(
      () -> -controller.getLeftY() * Constants.kThrottleMaxSpeed, 
      ()-> - controller.getLeftX() * Constants.kThrottleMaxSpeed, 
      ()-> -controller.getRightX()* Constants.kRotationMaxSpeed
    ),chassis));
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
