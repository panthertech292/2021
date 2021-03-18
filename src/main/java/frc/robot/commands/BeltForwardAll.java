// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BeltForwardAll extends CommandBase {
  /** Creates a new BeltForwardAll. */
  private final BeltSubsystem BeltSubsystem;
  private final ShooterSubsystem ShooterSubsystem;
  private double shooterSpeed;
  public BeltForwardAll(BeltSubsystem s_BeltSubsystem, ShooterSubsystem s_ShooterSubsystem, double v_shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    BeltSubsystem = s_BeltSubsystem;
    shooterSpeed = v_shooterSpeed;
    ShooterSubsystem = s_ShooterSubsystem;
    addRequirements(s_BeltSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BeltSubsystem.DriveBelts(0.0,0.0,0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.changeSetSpeed(shooterSpeed);
    BeltSubsystem.DriveBelts(BeltConstants.kBeltForwardSpeed, BeltConstants.kBeltForwardSpeed, BeltConstants.kBeltForwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BeltSubsystem.DriveBelts(0.0,0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return BeltSubsystem.BallReadyToFire();
  }
}
