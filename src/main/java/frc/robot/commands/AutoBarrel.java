// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

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
   v_driveSpeed = 0.8;
   v_fudgeFactor = 78;
   v_firstTurnAdjustment = 0.0;
   v_standardForward = 46.0;

   addRequirements(s_DriveSubsystem);

   addCommands(
     //Template Commands
    //new AutoForwardEncoder(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed, v_RightSpeed),
    //new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(75), 1.0, -.35),
    new AutoForwardPID(s_DriveSubsystem, .65, .65, 106.0-20.0),
   /* new AutoTurnPID(s_DriveSubsystem, .8, .1, 86),
    new AutoTurnPID(s_DriveSubsystem, .8, .1, 86),
    new AutoTurnPID(s_DriveSubsystem, .8, .1, 86),
    new AutoTurnPID(s_DriveSubsystem, .8, .1, 86),0.4346 */

    /*new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    new AutoTurnPID(s_DriveSubsystem, .275, .65, 90, 0.4346),
    
    //new AutoTurnPID(s_DriveSubsystem, .275, .65, 90-1, 0.4346),
    //new AutoTurnPID(s_DriveSubsystem, .275, .65, 90-1, 0.4346),
    //new AutoTurnPID(s_DriveSubsystem, .275, .65, 90-1, 0.4346),
    new AutoForwardPID(s_DriveSubsystem, .65, .65, 36.0),
   /* new AutoTurnPID(s_DriveSubsystem, .8, .8, 86),
    new AutoTurnPID(s_DriveSubsystem, .8, .8, 86),
    new AutoTurnPID(s_DriveSubsystem, .8, .8, 86),

    new AutoForwardPID(s_DriveSubsystem, .8, .8, 90.0),
    new AutoPIDEncoders(s_DriveSubsystem, .1, .8, 78.99704244, 0.4634),
    new AutoPIDEncoders(s_DriveSubsystem, .1, .8, 78.99704244, 0.4634),
    new AutoPIDEncoders(s_DriveSubsystem, .1, .8, 78.99704244, 0.4634),
    
    new AutoForwardPID(s_DriveSubsystem, .8, .8, 60.0),
    //new AutoPIDEncoders(s_DriveSubsystem, .1, .8, 78.99704244, 0.4634),
    new AutoForwardPID(s_DriveSubsystem, .8, .8, 90.0),
   new AutoPIDEncoders(s_DriveSubsystem, .1, .8, 78.99704244, 0.4634),
    new AutoPIDEncoders(s_DriveSubsystem, .1, .8, 78.99704244, 0.4634), 
    new AutoForwardPID(s_DriveSubsystem, .8, .8, 270.0), */
    new AutoDead(s_DriveSubsystem)
   );

  }
}
