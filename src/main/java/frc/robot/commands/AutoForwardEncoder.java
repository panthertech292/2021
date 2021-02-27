/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
public class AutoForwardEncoder extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private double AutoDistance;
  private double LeftSpeed;
  private double RightSpeed;
  /**
   * Creates a new AutoForwardEncoder.
   */
  public AutoForwardEncoder(DriveSubsystem s_DriveSubsystem, double v_AutoDistance, double v_LeftSpeed, double v_RightSpeed) {
    DriveSubsystem = s_DriveSubsystem;
    AutoDistance = v_AutoDistance;
    LeftSpeed = v_LeftSpeed;
    RightSpeed = v_RightSpeed;
    addRequirements(s_DriveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Forward Before Reset");
    System.out.println(DriveSubsystem.getLeftPosition());
    System.out.println(DriveSubsystem.getRightPosition());
    DriveSubsystem.zeroDistanceSensors();
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
   /* System.out.println("Forward Done");
    System.out.println(DriveSubsystem.getLeftPosition());
    System.out.println(DriveSubsystem.getRightPosition());
    */
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.encoderFinish(AutoDistance);
  }
}
