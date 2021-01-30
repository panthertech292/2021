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
    new AutoForwardEncoder(s_DriveSubsystem, 120.0 - 36.0, v_driveSpeed, v_driveSpeed),
   /*right*/ new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor-20), 1.0, -.35),
    new AutoForwardEncoder(s_DriveSubsystem, v_standardForward, v_driveSpeed, v_driveSpeed),
    new AutoForward(s_DriveSubsystem, 100000000.0, 0.0, 0.0),
    new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), 1.0, -.35),
    new AutoForwardEncoder(s_DriveSubsystem, v_standardForward, v_driveSpeed, v_driveSpeed),

    

    new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), 1.0, -.35),
    new AutoForwardEncoder(s_DriveSubsystem, v_standardForward, v_driveSpeed, v_driveSpeed),
    new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), 1.0, -.35),
    new AutoForwardEncoder(s_DriveSubsystem, 114.0, v_driveSpeed, v_driveSpeed),
   /*left*/ new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), -.35, 1.0),
   new AutoForwardEncoder(s_DriveSubsystem, v_standardForward, v_driveSpeed, v_driveSpeed),
   new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), -.35, 1.0),
   new AutoForwardEncoder(s_DriveSubsystem, v_standardForward, v_driveSpeed, v_driveSpeed),
   new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), -.35, 1.0),
   new AutoForwardEncoder(s_DriveSubsystem, 84.0, v_driveSpeed, v_driveSpeed),
   new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), -.35, 1.0),
   new AutoForwardEncoder(s_DriveSubsystem, 84.0, v_driveSpeed, v_driveSpeed),
   new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), -.35, 1.0),
   new AutoForwardEncoder(s_DriveSubsystem, v_standardForward, v_driveSpeed, v_driveSpeed),
   new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(90+v_fudgeFactor), -.35, 1.0),
   //new AutoForwardEncoder(s_DriveSubsystem, 280.0, v_driveSpeed, v_driveSpeed),
   new AutoForwardEncoder(s_DriveSubsystem, 60.0, v_driveSpeed, v_driveSpeed)



   );

  }
}
