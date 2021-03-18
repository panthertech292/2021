/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PickupSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoReloadFire extends SequentialCommandGroup {
  /**
   * Creates a new AutoReloadFire.
   */
  private final DriveSubsystem DriveSubsystem;
  private final ShooterSubsystem ShooterSubsystem;
  private final PickupSubsystem PickupSubsystem;
  public AutoReloadFire(ShooterSubsystem s_ShooterSubsystem, DriveSubsystem s_DriveSubsystem, PickupSubsystem s_PickupSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //super();
    DriveSubsystem = s_DriveSubsystem;
    ShooterSubsystem = s_ShooterSubsystem;
    PickupSubsystem = s_PickupSubsystem;
    addRequirements(s_ShooterSubsystem, s_DriveSubsystem, s_PickupSubsystem);

    addCommands(

    new AutoShootTimed(s_ShooterSubsystem),
    //We need a turn left
    new AutoBackwardEncoder(s_DriveSubsystem, DriveConstants.kAutoBackwardEncoderDistance, 0.5, 0.5),

    new AutoPickupTimed(s_PickupSubsystem),

    new AutoForwardEncoder(s_DriveSubsystem, DriveConstants.kAutoForwardEncoderDistance, 0.5, 0.5),

    new AutoRight90Gyro(s_DriveSubsystem, 90, 0.65, -0.65),

    new VisionAll(s_DriveSubsystem, 1.4),

    new AutoShootTimed(s_ShooterSubsystem)
    );
  }
}
