// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;


public class BeltMoveTimed extends CommandBase {
  /** Creates a new BeltForwardAll. */
  private final BeltSubsystem BeltSubsystem;
  public BeltMoveTimed(BeltSubsystem s_BeltSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    BeltSubsystem = s_BeltSubsystem;
    addRequirements(s_BeltSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BeltSubsystem.DriveBelts(0.0,0.0,0.0);
    BeltSubsystem.resetTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    BeltSubsystem.DriveBelts(BeltConstants.kBeltForwardSpeed+0.2, BeltConstants.kBeltForwardSpeed*0.0, BeltConstants.kBeltForwardSpeed+.0125);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BeltSubsystem.DriveBelts(0.0,0.0,0.0);
    BeltSubsystem.resetTimer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return BeltSubsystem.BallTimeout();
  }
}
