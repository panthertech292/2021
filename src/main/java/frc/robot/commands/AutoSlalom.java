// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSlalom extends SequentialCommandGroup {
  /** Creates a new AutoSlalom. */
  private final DriveSubsystem DriveSubsystem;
  public AutoSlalom(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;

    double c_visionFirstAllDistance = 6.13;
    double c_visionSecondAllDistance = 2.17;
    double c_visionThirdAllDistance = 5.16;

    double c_driveToFirstArea = 4.36;
    double c_driveToSecondArea = 3.16;
    


   addRequirements(s_DriveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      //Inital Straight
      new AutoForwardPID(s_DriveSubsystem, .7, .65, 12),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      //First S Turn
      new AutoRight90Gyro(s_DriveSubsystem, 90, -.35, .7),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      new AutoRight90Gyro(s_DriveSubsystem, 90, .7, -.35),

      //Vision Angle Align
      new VisionAlign(s_DriveSubsystem),
      new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToFirstArea),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .3),

      
      new VisionAll(s_DriveSubsystem, c_visionFirstAllDistance),


      

      //Align 360
      new AutoRight90Gyro(s_DriveSubsystem, 90, .7, -.3),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      new VisionAll(s_DriveSubsystem, c_visionSecondAllDistance),

      //Do the 360
      new AutoRight90Gyro(s_DriveSubsystem, 175, -.3, .7),
      new AutoRight90Gyro(s_DriveSubsystem, 175, -.3, .7),

      //Align after the 360
      new VisionAll(s_DriveSubsystem, c_visionSecondAllDistance),

      //Align with second vision target
      new AutoRight90Gyro(s_DriveSubsystem, 90, .7, -.3),

      new VisionAlign(s_DriveSubsystem),
      new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToSecondArea),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      new VisionAll(s_DriveSubsystem, c_visionThirdAllDistance),

      //Final S Turn
      new AutoRight90Gyro(s_DriveSubsystem, 90, .7, -.35),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .4),
      new AutoRight90Gyro(s_DriveSubsystem, 90, -.35, .7),

      //Final Straight - Drive to finish
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      new DriveUntilBounceSensor(s_DriveSubsystem, .3, .3)


      


    /*
    new AutoForward(s_DriveSubsystem, .5, 0.775, .85),
    new AutoForward(s_DriveSubsystem, .775, 0.4, 1.1),
    new AutoForwardPID(DriveSubsystem, .7, .65, 16),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.65, 0.2, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForwardPID(DriveSubsystem, .7, .65, 120),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 360, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForwardPID(DriveSubsystem, .7, .65, 120),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new AutoForward(s_DriveSubsystem, 0.65, 0.2, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 90, 2.15789),
    new AutoForward(s_DriveSubsystem, 0.2, 0.65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForwardPID(DriveSubsystem, .7, .65, 30),
    new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
    new DriveUntilBounceSensor(s_DriveSubsystem, .3, .3)
    */
    );
  }
}
