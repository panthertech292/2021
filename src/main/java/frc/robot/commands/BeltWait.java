// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BeltWait extends CommandBase {
  /** Creates a new BeltForwardAll. */
  private final BeltSubsystem BeltSubsystem;
  
  private double beltSpeed;
  private double Time;
  public BeltWait(BeltSubsystem s_BeltSubsystem, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    BeltSubsystem = s_BeltSubsystem;
    Time = time;
    addRequirements(s_BeltSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BeltSubsystem.resetTimer();
    System.out.println("Waiting I = " + BeltSubsystem.getTimerValue());
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
      System.out.println("Waiting E= " + BeltSubsystem.getTimerValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    BeltSubsystem.resetTimer();
  }

  // Returns true when the command should end.
  @Override

  public boolean isFinished() {
    System.out.println("Waiting F= " + BeltSubsystem.getTimerValue());
    return BeltSubsystem.getTimerValue()>=Time;
  }
}
