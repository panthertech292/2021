/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class DriveToVisionTargetArea extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  
  private double LeftSpeed;
  private double RightSpeed;
  private double targetArea;
  /**
   * Creates a new DriveToVisionTargetArea.
   */
  public DriveToVisionTargetArea(DriveSubsystem s_DriveSubsystem, double v_LeftSpeed, double v_RightSpeed, double v_targetArea) {
    DriveSubsystem = s_DriveSubsystem;
    
    LeftSpeed = v_LeftSpeed;
    RightSpeed = v_RightSpeed;
    targetArea = v_targetArea;
    addRequirements(s_DriveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Forward Before Reset");
    System.out.println(DriveSubsystem.getLeftPosition());
    System.out.println(DriveSubsystem.getRightPosition());
    DriveSubsystem.zeroDistanceSensors();
    DriveSubsystem.zeroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    if(DriveSubsystem.visionTargetSensor() != 0.0){
    DriveSubsystem.changePowerSetPoints(LeftSpeed,DriveSubsystem.AnglePID(0.0, RightSpeed));
    }
    else{
      DriveSubsystem.changePowerSetPoints(0.0, 0.0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
   /* System.out.println("Forward Done");
    System.out.println(DriveSubsystem.getLeftPosition());
    System.out.println(DriveSubsystem.getRightPosition());
    */
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.visionTargetSizeFinishApproach(targetArea);
  }
}
