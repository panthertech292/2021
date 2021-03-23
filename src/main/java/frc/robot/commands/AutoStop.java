// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoStop extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private double v_zeroCount;
  private double v_returnSpot;
  private double v_distance;

  /** Creates a new AutoStop. */
  public AutoStop(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);
    v_zeroCount = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    v_zeroCount = 0;
    DriveSubsystem.resetTimer();
    v_returnSpot = DriveSubsystem.getLeftEncoderValue();
    DriveSubsystem.initializePID();

    //DriveSubsystem.zeroDistanceSensors();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    //System.out.println(v_zeroCount);
    v_distance = DriveSubsystem.getLeftEncoderValue() - v_returnSpot;
    System.out.println(v_distance);
    if (v_distance < 0){
      System.out.println("Running Back");
      DriveSubsystem.changePowerSetPoints(DriveSubsystem.ReusablePID1(.00055, .0003, 0.0002, DriveSubsystem.getLeftEncoderValue(), v_returnSpot,0.5),DriveSubsystem.ReusablePID2(.00055, .0003, 0.0002, DriveSubsystem.getLeftEncoderValue(), v_returnSpot,0.5));
    }
    if (v_distance > 0){
      System.out.println("Running Forward");
      DriveSubsystem.changePowerSetPoints(DriveSubsystem.ReusablePID1(.00055, .0003, 0.0002, DriveSubsystem.getLeftEncoderValue(), v_returnSpot,0.5),DriveSubsystem.ReusablePID2(.00055, .0003, 0.0002, DriveSubsystem.getLeftEncoderValue(), v_returnSpot,0.5));
    }
    if (v_distance > -150 && v_distance < 150){
        System.out.println("Increasing count!");
        v_zeroCount = v_zeroCount + 1;
    }
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return v_zeroCount > 15;
    
    
    
  }
}
