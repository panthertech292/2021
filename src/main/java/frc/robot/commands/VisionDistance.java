/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class VisionDistance extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  /**
   * Creates a new VisionDistance.
   */
  public VisionDistance(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.changePowerSetPoints(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running the distance command");
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.visionDistanceArea();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.visionFinishDistance();
  }
}
