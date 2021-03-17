// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAdjustNearZone extends CommandBase {
  /** Creates a new AimAdjustNearZone. */
  public final ShooterSubsystem ShooterSubsystem;
  private double shooterSpeed;
  public AimAdjustNearZone(ShooterSubsystem s_ShooterSubsystem, double v_shooterSpeed) {
    ShooterSubsystem = s_ShooterSubsystem;
    shooterSpeed = v_shooterSpeed;
    addRequirements(s_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ShooterSubsystem.aimResetCheck()==true){
      ShooterSubsystem.changeSetSpeed(shooterSpeed);
    if(ShooterSubsystem.getAimEncoder()<ShooterConstants.kstartingAim){
      ShooterSubsystem.ShooterAimUp();
    }
    else{
      ShooterSubsystem.ShooterAimDown();
    }
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.ShooterAimStop();
    ShooterSubsystem.changeSetSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSubsystem.getAimEncoder()>=ShooterConstants.kstartingAim && ShooterSubsystem.getAimEncoder() <=ShooterConstants.kstartingAim +2;
  }
}
