/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnPID extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  
  private final double TargetSpeedLeft;
  private final double TargetSpeedRight;
  private double Angle;
  private double Ratio;

  /**
   * Creates a new AutoForward.
   */
  public AutoTurnPID(DriveSubsystem s_DriveSubsystem, double v_TargetSpeedLeft, double v_TargetSpeedRight, double m_angle, double v_ratio) {
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);

    
    TargetSpeedLeft = v_TargetSpeedLeft;
    TargetSpeedRight = v_TargetSpeedRight;
    Angle = m_angle;
    Ratio = v_ratio;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("##############################");
    DriveSubsystem.resetTimer();
    DriveSubsystem.zeroDistanceSensors();
    DriveSubsystem.zeroAngle();
    DriveSubsystem.initializePID();
    DriveSubsystem.driveModePowerSetPoint();
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    if(TargetSpeedLeft<=TargetSpeedRight){
    DriveSubsystem.changePowerSetPoints(DriveSubsystem.RatioLeftPID(Ratio, TargetSpeedLeft),TargetSpeedRight);}

    if(TargetSpeedRight<TargetSpeedLeft){
      DriveSubsystem.changePowerSetPoints(TargetSpeedLeft,DriveSubsystem.RatioLeftPID(Ratio, TargetSpeedRight));
    }

    }

    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    DriveSubsystem.changePowerSetPoints(0,0);
    System.out.println("All Done");
   // System.out.println(DriveSubsystem.getLeftPosition());
  // System.out.println(DriveSubsystem.getRightPosition());
    DriveSubsystem.zeroAngle();
    DriveSubsystem.driveModeTeleop();
    System.out.println("######################################1");
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.gyroFinish(Angle);
   // return DriveSubsystem.encoderFinish(Angle);
  }
}
