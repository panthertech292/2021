/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoBounce extends SequentialCommandGroup {
  /**
   * Creates a new AutoBounce.
   */
  private final DriveSubsystem DriveSubsystem;
  public AutoBounce(DriveSubsystem s_DriveSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
   // super();
   DriveSubsystem = s_DriveSubsystem;

   double c_visionFirstAllDistance = 3.210;
   double c_driveToFirstBackupArea = 5.84;

   double c_driveToFirstArea = 2.01;

   double c_driveToSecondTarget = 3;
   double c_visionSecondCloseDistance = 6.2;
   double c_driveToSecondBackupTarget = 2.683;
   double c_visionSecondBackupDistance = 1.519;

   double c_driveToThirdArea = 3.680;
   double c_driveToThirdTarget = 22.323-.4;
   double c_visionThirdCloseDistance = 22.323;

   double c_driveToThirdBackupTarget = 0.93;
   double c_visionThirdBackupDistance = 1.53;


   addRequirements(s_DriveSubsystem);

   addCommands(
     //Inital Straight
     /*
     new AutoForwardPID(s_DriveSubsystem, .7, .65, 16),
     new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),

     //First Left to get us to first target
     new AutoRight90Gyro(s_DriveSubsystem, 90, -.45, .7),

     //Touch First Target
     new VisionAll(s_DriveSubsystem, c_visionFirstAllDistance),

     //Backup from Target after contact
     //new DriveToVisionTargetArea(s_DriveSubsystem, -.65, -.65, c_driveToFirstBackupArea),
     new AutoForwardPID(s_DriveSubsystem, -.7, -.65, 8),
     new AutoForward(s_DriveSubsystem, 0.65, 0.65, .2),

     //First S-Turn (Reversing)
     new AutoRight90Gyro(s_DriveSubsystem, 45, -.7, .35),
     new AutoForward(s_DriveSubsystem, 0.65, 0.0, .2),
     new AutoForwardPID(s_DriveSubsystem, -.5, -.45, 15),
     new AutoForward(s_DriveSubsystem, 0.65, 0.65, .2),
     new AutoRight90Gyro(s_DriveSubsystem, 45, .35, -.7),

     //Backup After S-Turn and get to position for second target
     new AutoForwardPID(s_DriveSubsystem, -.7, -.65, 12),
     new AutoForward(s_DriveSubsystem, 0.65, 0.65, .3),
     new AutoRight90Gyro(s_DriveSubsystem, 70, .7, -.7),
     //new VisionAlign(s_DriveSubsystem),
     //new AutoForwardPID(s_DriveSubsystem, .7, .65, 30),
     //new AutoForward(s_DriveSubsystem, 0.65, 0.65, .2),
     
     */
     
     //new VisionAlign(s_DriveSubsystem),
     //new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToFirstArea),
     new VisionAll(s_DriveSubsystem, c_driveToFirstArea),
     //new AutoForward(s_DriveSubsystem, -0.65, -0.65, .3),

     //Turn and align to second target
     new AutoRight90Gyro(s_DriveSubsystem, 90, -.55, .7),
     new VisionAlign(s_DriveSubsystem),
     new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToSecondTarget),
     new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
     new VisionAll(s_DriveSubsystem, c_visionSecondCloseDistance),

     //Backup
     //new DriveToVisionTargetArea(s_DriveSubsystem, -.65, -.65, c_driveToSecondBackupTarget),
     //new AutoForward(s_DriveSubsystem, 0.65, 0.65, .2),
     //new VisionAll(s_DriveSubsystem, c_visionSecondBackupDistance)
     new DriveToVisionTargetArea(s_DriveSubsystem, -0.65, -0.65, c_visionSecondBackupDistance),
  //   new AutoForwardPID(s_DriveSubsystem, -0.51, -0.5, 80)
     new AutoForward(s_DriveSubsystem, 0.80, 0.80, .3)

     /*
     //Go forward to final target area
     new AutoRight90Gyro(s_DriveSubsystem, 90, .7, -.45),
     new VisionAlign(s_DriveSubsystem),
     new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToThirdArea),
     new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),

     //Turn and align to third target
     new AutoRight90Gyro(s_DriveSubsystem, 90, -.35, .7),
     new VisionAlign(s_DriveSubsystem),
     new DriveToVisionTargetArea(s_DriveSubsystem, .65, .65, c_driveToThirdTarget),
     new AutoForward(s_DriveSubsystem, -0.65, -0.65, .2),
     new VisionAll(s_DriveSubsystem, c_visionThirdCloseDistance),

     //Backup after contact with third target
     //new DriveToVisionTargetArea(s_DriveSubsystem, -.65, -.65, c_driveToThirdBackupTarget),
     //new AutoForward(s_DriveSubsystem, 0.65, 0.65, .2),
     //new VisionAll(s_DriveSubsystem, c_visionThirdBackupDistance),

     //Turn to end zone
     new AutoRight90Gyro(s_DriveSubsystem, 90, .7, -.35)
     //new AutoForwardPID(s_DriveSubsystem, .7, .65, 18),
     //new DriveUntilBounceSensor(s_DriveSubsystem, .3, .3)
     */
   );
  }
}
