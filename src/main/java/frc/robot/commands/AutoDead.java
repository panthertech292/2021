/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDead extends CommandBase {
  /**
   * Creates a new AutoDead.
   */

  private final DriveSubsystem DriveSubsystem;
  
  public AutoDead(DriveSubsystem s_DriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.changePowerSetPoints(0.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
