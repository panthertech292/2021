/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class AutoTurnGyroWithVisionFinish extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private double AutoAngle;
  private double LeftSpeed;
  private double RightSpeed;
  private boolean visionFinish;
  private int v_turnDirection;
  /**
   * Creates a new AutoTurnGyroWithVisionFinish.
   */
  public AutoTurnGyroWithVisionFinish(DriveSubsystem s_DriveSubsystem, double v_AutoAngle, double v_LeftSpeed, double v_RightSpeed) {
    DriveSubsystem = s_DriveSubsystem;
    AutoAngle = v_AutoAngle;
    LeftSpeed = v_LeftSpeed;
    RightSpeed = v_RightSpeed;
    addRequirements(s_DriveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.zeroAngle();
    visionFinish = false;
    if(LeftSpeed>RightSpeed){
      v_turnDirection = 1; //Right Turn
    }
    else{
      v_turnDirection = -1; //Left Turn
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.changePowerSetPoints(LeftSpeed,RightSpeed);
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
    if(DriveSubsystem.gyroFinish(AutoAngle) == true){
      System.out.println("Gyro Finished");
    }
    if(DriveSubsystem.gyroWithVisionFinish(AutoAngle, v_turnDirection) == true){
      System.out.println("Vision Finished");
      System.out.println(DriveSubsystem.getAbsCurrentAngle());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return DriveSubsystem.gyroFinish(AutoAngle) || DriveSubsystem.gyroWithVisionFinish(AutoAngle, v_turnDirection);
  }
}