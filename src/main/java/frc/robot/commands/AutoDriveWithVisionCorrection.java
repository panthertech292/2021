// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveWithVisionCorrection extends SequentialCommandGroup {
  /** Creates a new AutoDriveWithVisionCorrection. */
 
  private double TargetSpeed;
  private double Time;
  private final DriveSubsystem DriveSubsystem;
  private int c_Implementations;
  public AutoDriveWithVisionCorrection( DriveSubsystem s_DriveSubsystem, double v_Time, double v_TargetSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    DriveSubsystem = s_DriveSubsystem;
    TargetSpeed = v_TargetSpeed;
    Time = v_Time;
    c_Implementations = 10;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Deprecated
    /*  new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),

     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),

     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),
     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),
     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),
     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),
     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),
     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),
     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed),
     new VisionAlign(s_DriveSubsystem),
     new AutoForwardPID(s_DriveSubsystem, v_Time/c_Implementations, v_TargetSpeed, v_TargetSpeed) */
    );
  }
}

