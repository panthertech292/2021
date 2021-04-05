// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBreak extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private final double c_leftSpeed;
  private final double c_rightSpeed;
  /** Creates a new AutoBreak. */
  public AutoBreak(DriveSubsystem s_DriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);

    c_leftSpeed = -0.65;
    c_rightSpeed = -0.65;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.changePowerSetPoints(c_leftSpeed, c_rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (DriveSubsystem.getRightEncoderVelocity() < 0);
  }
}
