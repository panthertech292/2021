/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class AutoTurnAdjustableGyro extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private double AutoAngle;
  private double FastSpeed;
  private double SlowSpeed;
  private char Forward;
  private char Right;
  /**
   * Creates a new AutoTurnAdjustableGyro.
   */
  public AutoTurnAdjustableGyro(DriveSubsystem s_DriveSubsystem, double v_FastSpeed, double v_SlowSpeed, double v_AutoAngle, char v_Forward, char v_Right) {
    DriveSubsystem = s_DriveSubsystem;
    FastSpeed = v_FastSpeed;
    SlowSpeed = v_SlowSpeed;
	AutoAngle = v_AutoAngle;
	Forward = v_Forward;
	Right = v_Right;
    addRequirements(s_DriveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.zeroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
	if(Forward == 'F') {
		if(Right == 'R') { 
		    // Forward Right
            DriveSubsystem.changePowerSetPoints(FastSpeed,SlowSpeed); // (Left, Right)
		} else { 
		    // Forward Left
 			DriveSubsystem.changePowerSetPoints(SlowSpeed,FastSpeed); // (Left, Right)
		}
    } else {	
        if(Right == 'R') {
            // Reverse Right (Right if going forward - full speed on left)
            DriveSubsystem.changePowerSetPoints(-1.0*FastSpeed,-1.0*SlowSpeed); // (Left, Right)
		} else { 
		    // Reverse Left (Left if going forward - full speed on right)
 			DriveSubsystem.changePowerSetPoints(-1.0*SlowSpeed,-1.0*FastSpeed); // (Left, Right)
		}   
    }		
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  DriveSubsystem.gyroFinish(AutoAngle);
  }
}
