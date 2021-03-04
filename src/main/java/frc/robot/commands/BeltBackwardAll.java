// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;

public class BeltBackwardAll extends CommandBase {
  /** Creates a new BeltBackwardAll. */
  private final BeltSubsystem BeltSubsystem;
  public BeltBackwardAll(BeltSubsystem s_BeltSubsystem) {
    BeltSubsystem = s_BeltSubsystem;
    addRequirements(s_BeltSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BeltSubsystem.DriveBelts(0.0, 0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BeltSubsystem.DriveBelts(BeltConstants.kBeltBackwardSpeed, BeltConstants.kBeltBackwardSpeed, BeltConstants.kBeltBackwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
