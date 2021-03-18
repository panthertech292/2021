/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class VisionAlign extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private boolean v_timeout;
  /**
   * Creates a new VisionAlign.
   */

  public VisionAlign(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_DriveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.resetTimer();
    DriveSubsystem.changePowerSetPoints(0,0);
    DriveSubsystem.zeroDistanceSensors();
    System.out.println("Starting Adjustment");
    v_timeout = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running the move command");
    DriveSubsystem.driveModePowerSetPoint();
    if(DriveSubsystem.visionTargetSensor()!=0.0){
    DriveSubsystem.resetTimer();
    DriveSubsystem.visionAlignLeft();
    DriveSubsystem.visionAlignRight();
    DriveSubsystem.visionAlignLeft();
    DriveSubsystem.visionAlignRight();
    DriveSubsystem.visionAlignLeft();
    DriveSubsystem.visionAlignRight();
    System.out.println("Seeing a target!!!!!!!!");
    }
    else{
      if(DriveSubsystem.getTimerValue()<3 && DriveSubsystem.getTimerValue()>0){
        System.out.println("No Target!!!!!!!!");
      DriveSubsystem.changePowerSetPoints(-0.85, 0.85); //Adjust values!!!
      }
      if(DriveSubsystem.getTimerValue()>3 && DriveSubsystem.getTimerValue()<6){
        System.out.println("No Target2!!!!!!!!");
        DriveSubsystem.changePowerSetPoints(0.85, -0.85); //Adjust Values!!!
        }
        if(DriveSubsystem.getTimerValue()>5.0){
          System.out.println("No Target3!!!!!!!!");
          System.out.println(v_timeout);
          v_timeout = true;
        }
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
    System.out.println("Left Encoder Value"  +   DriveSubsystem.getLeftPosition());
    System.out.println("Right Encoder Value"  +   DriveSubsystem.getRightPosition());
    System.out.println(v_timeout);

    System.out.println("Perceived Angle"   +  DriveSubsystem.PerceivedAngle(DriveSubsystem.getLeftPosition()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.visionFinish() || v_timeout;
  }
}
