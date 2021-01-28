// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GateSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFireGated extends CommandBase {
  private final ShooterSubsystem ShooterSubsystem;
  private final GateSubsystem GateSubsystem; 
  
  /** Creates a new ShooterFireGated. */
  public ShooterFireGated(ShooterSubsystem s_ShooterSubsystem, GateSubsystem s_GateSubsystem) {
    ShooterSubsystem = s_ShooterSubsystem;
    GateSubsystem = s_GateSubsystem;

    addRequirements(s_ShooterSubsystem, s_GateSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GateSubsystem.Gate1Mid();
    ShooterSubsystem.ShooterStop();
    ShooterSubsystem.resetTimer();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.changeSetSpeed(ShooterConstants.kShooterHalf);
    if(ShooterSubsystem.getTimerValue() >= 0.5){
      GateSubsystem.Gate1Down();
    }
    if(ShooterSubsystem.getTimerValue() >= 1.2){
      GateSubsystem.Gate1Mid();
    }
    if(ShooterSubsystem.getTimerValue() >= 1.9){
      GateSubsystem.Gate1Down();
    }
    if(ShooterSubsystem.getTimerValue() >= 2.6){
      GateSubsystem.Gate1Mid();
    }
    if(ShooterSubsystem.getTimerValue() >= 3.3){
      GateSubsystem.Gate1Down();
    }
    if(ShooterSubsystem.getTimerValue() >= 4.0){
      GateSubsystem.Gate1Mid();
    }
    
    
    
    
    
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    ShooterSubsystem.ShooterStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSubsystem.getTimerValue()>4.1;
  }
}
