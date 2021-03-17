// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAdjustStartingPosition extends CommandBase {
  /** Creates a new AimAdjustStartingPosition. */
  public final ShooterSubsystem ShooterSubsystem;
  private boolean v_firstTime;
  private boolean v_finish;
  public AimAdjustStartingPosition(ShooterSubsystem s_ShooterSubsystem) {
    ShooterSubsystem = s_ShooterSubsystem;
    addRequirements(s_ShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.resetTimer();
    v_firstTime = true;
    v_finish = false;
    System.out.println("Starting!!!!!!!!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ShooterSubsystem.aimResetCheck()==false){
      if(ShooterSubsystem.AimSwitchValue() == false && v_firstTime == true){
        ShooterSubsystem.ShooterAimDown();
        ShooterSubsystem.resetTimer();
      }
      else{
        System.out.println("Running!!!");
        ShooterSubsystem.ShooterAimUp();
        v_firstTime = false;
      }
    }
    else{
      System.out.println("Skipped Check!!!!!!!!!!");
      v_finish = true;
      
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.ShooterAimStop();
    System.out.println(ShooterSubsystem.getAimEncoder());
    System.out.println("Zeroed!!!!!!!!!!!!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ShooterSubsystem.getAimEncoder()>=ShooterConstants.kstartingAim || v_finish;
  }
}
