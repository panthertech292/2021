/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class AutoRight90Timed extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  /**
   * Creates a new AutoRight90Timed.
   */
  public AutoRight90Timed(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.resetTimer();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.changePowerSetPoints(0.5,-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.getTimerValue()>0.99;
  }
}
