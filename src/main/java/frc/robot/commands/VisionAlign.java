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
  private boolean v_aligning;
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
    DriveSubsystem.initializePID();
    DriveSubsystem.resetPop();
    System.out.println("Starting Adjustment");
    v_timeout = false;
    v_aligning = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    v_aligning = false;
    System.out.println("Running the align command");
    DriveSubsystem.driveModePowerSetPoint();
    if(DriveSubsystem.visionTargetSensor()>=1.0){
    v_aligning = true;
    DriveSubsystem.visionAlignLeft();
    DriveSubsystem.visionAlignRight();
    /*DriveSubsystem.visionAlignLeft();
    DriveSubsystem.visionAlignRight();
    DriveSubsystem.visionAlignLeft();
    DriveSubsystem.visionAlignRight();
  */
    }
    
        if(DriveSubsystem.getTimerValue()>1.5){
          System.out.println("No Target3!!!!!!!!");
          System.out.println(v_timeout);
          v_timeout = true;
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
    return (DriveSubsystem.visionFinish()&& DriveSubsystem.getRightEncoderVelocity() == 0.1) || v_timeout;
  }
}
