// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBarrel extends SequentialCommandGroup {
  /** Creates a new AutoBarrel. */
  private final DriveSubsystem DriveSubsystem;
  
  private double v_driveSpeed;
  private double v_fudgeFactor;
  private double v_standardForward;
  private double v_firstTurnAdjustment;
  public AutoBarrel(DriveSubsystem s_DriveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
   // addCommands();
   DriveSubsystem = s_DriveSubsystem;
   addRequirements(s_DriveSubsystem);

   addCommands(
     //Template Commands
    //new AutoForwardEncoder(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed, v_RightSpeed),
    //new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(75), 1.0, -.35),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 106.0-10.0),
   new AutoForward(s_DriveSubsystem, 0.1, 0.0, 0.15),
   new AutoTurnPID(s_DriveSubsystem, .65, .15, 90.0, 2.15789),
   new AutoTurnPID(s_DriveSubsystem, .65, .30, 270.0, 2.15789)
   //new AutoDriveVisionCorrection(s_DriveSubsystem, 90.0, .7, .65),
   /*new AutoForward(s_DriveSubsystem, 0.1, .2, 0.65),
   new AutoTurnPID(s_DriveSubsystem, .2, .65, 270, 2.15789),
   new AutoDriveVisionCorrection(s_DriveSubsystem, 60.0, .7, .65),
   new AutoForward(s_DriveSubsystem, 0.1, .2, 0.65),
   new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
   new AutoForwardPID(s_DriveSubsystem, .7, .65, 90.0),
   new AutoForward(s_DriveSubsystem, 0.1, .2, 0.65),
   new AutoTurnPID(s_DriveSubsystem, .2, .65, 180, 2.15789),
   new AutoDriveVisionCorrection(s_DriveSubsystem, 300.0, .7, .65), */
   
   
   );

  }
}
