/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class DriveUntilBounceSensor extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private double LeftSpeed;
  private double RightSpeed;
  /**
   * Creates a new DriveUntilBounceSensor.
   */
  public DriveUntilBounceSensor(DriveSubsystem s_DriveSubsystem, double v_LeftSpeed, double v_RightSpeed) {
    DriveSubsystem = s_DriveSubsystem;
    
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
    return DriveSubsystem.getBounceSensor();
  }
  
}
