// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GateSubsystem;

public class GateToggleTimed extends CommandBase {
  /** Creates a new GateToggleTimed. */
  private final GateSubsystem GateSubsystem;
  public GateToggleTimed(GateSubsystem s_GateSubsystem) {
    GateSubsystem = s_GateSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_GateSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GateSubsystem.Gate1Mid();
    GateSubsystem.resetTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(GateSubsystem.getTimerValue() >= 0.5){
      GateSubsystem.Gate1Down();
    }
    if(GateSubsystem.getTimerValue() >= 1.2){
      GateSubsystem.Gate1Mid();
    }
    if(GateSubsystem.getTimerValue() >= 1.9){
      GateSubsystem.Gate1Down();
    }
    if(GateSubsystem.getTimerValue() >= 2.6){
      GateSubsystem.Gate1Mid();
    }
    if(GateSubsystem.getTimerValue() >= 3.3){
      GateSubsystem.Gate1Down();
    }
    if(GateSubsystem.getTimerValue() >= 4.0){
      GateSubsystem.Gate1Mid();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return GateSubsystem.getTimerValue()>4.1;
  }
}
