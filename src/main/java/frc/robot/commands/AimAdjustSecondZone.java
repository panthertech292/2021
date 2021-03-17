// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAdjustSecondZone extends CommandBase {
  /** Creates a new AimAdjustSecondZone. */
  public final ShooterSubsystem ShooterSubsystem;
  private double shooterSpeed;
  public AimAdjustSecondZone(ShooterSubsystem s_ShooterSubsystem, double v_shooterSpeed) {
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
    if(ShooterSubsystem.getAimEncoder() <=ShooterConstants.kstartingAim+ShooterConstants.kdeltaAimSecond+2){
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSubsystem.getAimEncoder()>=ShooterConstants.kstartingAim+ShooterConstants.kdeltaAimSecond && ShooterSubsystem.getAimEncoder() <=ShooterConstants.kstartingAim+ShooterConstants.kdeltaAimSecond+2;
  }
}
