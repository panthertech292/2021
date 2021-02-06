/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class AutoRight90Encoder extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  /**
   * Creates a new AutoRight90Encoder.
   */
  public AutoRight90Encoder(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.zeroLeftPosition();
    DriveSubsystem.zeroDistanceSensors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.changePowerSetPoints(0.65,-0.65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
    DriveSubsystem.deltaTurn();
  System.out.println(DriveSubsystem.PerceivedAngle(DriveSubsystem.getLeftPosition()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  DriveSubsystem.encoderFinish(DriveSubsystem.rotateRobot(93));
  }
}
