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
  public AutoBarrel(DriveSubsystem s_DriveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands();
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);

    double c_visionFirstAllDistance = 7.6;
    double c_visionSecondAllDistance = 2.1;

    double c_driveToFirstArea = 5.4;
    double c_driveToSecondArea = 1.6;
    double c_driveToThirdArea = 4.0;


   //new AutoForward(s_DriveSubsystem, 0.7, 0.65, 0.1),
   new AutoForward(s_DriveSubsystem, -0.65, -0.65, .3),
 //  new AutoRight90Gyro(s_DriveSubsystem, 60, .7, -0.54),
 //  new AutoForward(s_DriveSubsystem, 0.0, 0.0, 0.2),
   new AutoRight90Gyro(s_DriveSubsystem, 180, .7, -.3),
   new AutoForward(s_DriveSubsystem, 0.0, 0.0, 0.2),
   new AutoRight90Gyro(s_DriveSubsystem, 180, .7, -.4)
   /*new AutoRight90Gyro(s_DriveSubsystem, 90, .7,s -0.4),
   new AutoRight90Gyro(s_DriveSubsystem, 90, .7, 0.2),
    //new AutoForward(s_DriveSubsystem, .7, 0.3, .75),
    //new AutoForward(s_DriveSubsystem, .7, 0.43, 3.0),
    //new AutoTurnPID(s_DriveSubsystem, .65, -0.3, 270.0, 2.15789),
        new VisionAlign(s_DriveSubsystem),
        new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, 5.0),
        new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
        new VisionAll(s_DriveSubsystem, 7.0),
        new AutoRight90Gyro(s_DriveSubsystem, 100, -0.25, 0.7 ),
      new AutoRight90Gyro(s_DriveSubsystem, 100, -0.27, 0.7 ),
   new AutoRight90Gyro(s_DriveSubsystem, 100, -0.45, 0.7),
   new VisionAlign(s_DriveSubsystem),
new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, 1.45),
   new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
   new VisionAll(s_DriveSubsystem, 1.88),
   new AutoRight90Gyro(s_DriveSubsystem, 85, -0.36,.7),
   new AutoForwardPID(s_DriveSubsystem, .7, .65, 60.0-(48.0*(5/6))), 


    addCommands(
      //First Straight
      new AutoForwardPID(s_DriveSubsystem, .7, .65, 106-10),
      //Breaks Robot 
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .3),
      //First Turn
      new AutoRight90Gyro(s_DriveSubsystem, 175, .7, -.3),
      new AutoForward(s_DriveSubsystem, 0.0, 0.0, 0.2),
      new AutoRight90Gyro(s_DriveSubsystem, 180, .7, -.3),
      //Go to first vision target
      new VisionAlign(s_DriveSubsystem),
      new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToFirstArea),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      new VisionAll(s_DriveSubsystem, c_visionFirstAllDistance),
      //Second Turn
      new AutoRight90Gyro(s_DriveSubsystem, 270, -0.35, 0.7 ),
      //Go to second vision target
      new VisionAlign(s_DriveSubsystem),
      new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToSecondArea),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      new VisionAll(s_DriveSubsystem, c_visionSecondAllDistance),
      //Third Turn
      new AutoRight90Gyro(s_DriveSubsystem, 85, -0.36,.7),
      //Go forward???
      new AutoForwardPID(s_DriveSubsystem, .7, .65, 60.0-18), 
      new AutoForward(s_DriveSubsystem, -.65, -.65, .3),
      //Turn around
      new AutoRight90Gyro(s_DriveSubsystem, 90, - 0.4,.7),
      new AutoRight90Gyro(s_DriveSubsystem, 90, - 0.4,.7),
      //Go to final target
      new VisionAlign(s_DriveSubsystem),
      new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToThirdArea),
      new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
      new DriveUntilBounceSensor(s_DriveSubsystem, .3, .3)




  
   
   

   /*////SECTION FOR BAD BOT
   new AutoForwardPID(s_DriveSubsystem, .7, .65, 106.0-33.0), //RECONNECT THE ENCODERS!!!!!!!
   //new AutoForward(s_DriveSubsystem, 0.7, 0.65, 0.1),
    new AutoRight90Gyro(s_DriveSubsystem, 360, .7, -0.4),
    //new AutoForward(s_DriveSubsystem, .7, 0.3, .75),
    //new AutoForward(s_DriveSubsystem, .7, 0.43, 3.0),
    //new AutoTurnPID(s_DriveSubsystem, .65, -0.3, 270.0, 2.15789),
    new VisionAlign(s_DriveSubsystem),
    new VisionAlign(s_DriveSubsystem),
    new AutoDriveVisionCorrection(s_DriveSubsystem, 74.0, .7, .65)
  //new AutoForward(s_DriveSubsystem, .3, 0.7, .75),
   // new AutoForward(s_DriveSubsystem, .4, 0.7, 3.0)
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
