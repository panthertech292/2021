/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFireFull extends CommandBase {
  private final ShooterSubsystem ShooterSubsystem;
  /**
   * Creates a new ShooterFireFull.
   */
  public ShooterFireFull(ShooterSubsystem s_ShooterSubsystem) {
    ShooterSubsystem = s_ShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSubsystem.ShooterStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ShooterSubsystem.getAimEncoder()>=ShooterConstants.kstartingAim-10 && ShooterSubsystem.getAimEncoder() <=ShooterConstants.kstartingAim +10){
    ShooterSubsystem.changeSetSpeed(0.65); //reset to .65 after testing
    System.out.println("As intended!");
    }
    else{
    ShooterSubsystem.changeSetSpeed(ShooterConstants.kShooterFull+0.05);
    }
   System.out.println(ShooterSubsystem.getEncoderRate());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.ShooterStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
