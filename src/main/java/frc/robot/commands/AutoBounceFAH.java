// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBounceFAH extends SequentialCommandGroup {
  /** Creates a new AutoBounceFAH. */
  private final DriveSubsystem DriveSubsystem;
  public AutoBounceFAH(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;

   addRequirements(s_DriveSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 16),
    new AutoForward(s_DriveSubsystem, .2, .65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 15),
    new AutoRight90Gyro(s_DriveSubsystem, 180-40, .65, -.65),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 15),
    new AutoForward(s_DriveSubsystem, .2, .65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 45, 2.15789),
    new AutoForward(s_DriveSubsystem, .65, .2, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .65, .2, 45, 2.15789),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 30), 
    new AutoForward(s_DriveSubsystem, .2, .65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 180, 2.15789),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 75),
    new AutoRight90Gyro(s_DriveSubsystem, 180-40, .65, -.65),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 75),
    new AutoForward(s_DriveSubsystem, .2, .65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 30),
    new AutoForward(s_DriveSubsystem, .2, .65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 75),
    new AutoRight90Gyro(s_DriveSubsystem, 180-40, .65, -.65),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 15),
    new AutoForward(s_DriveSubsystem, .2, .65, 0.1),
    new AutoTurnPID(s_DriveSubsystem, .2, .65, 90, 2.15789),
    new AutoForwardPID(s_DriveSubsystem, .7, .65, 30),
    new AutoDead(s_DriveSubsystem)




    );
  }
}
