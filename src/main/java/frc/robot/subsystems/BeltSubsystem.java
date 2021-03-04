// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltConstants;

public class BeltSubsystem extends SubsystemBase {
  /** Creates a new BeltSubsystem. */
  private final WPI_TalonSRX FrontBeltMotor;
  private final WPI_TalonSRX BackBeltMotor;
  private final WPI_TalonSRX BottomBeltMotor;

  private double v_bottomBeltSpeed;
  private double v_frontBeltSpeed;
  private double v_backBeltSpeed;

  private int v_printCount;

  public BeltSubsystem() {
    FrontBeltMotor = new WPI_TalonSRX(BeltConstants.kFrontBeltMotor);
    BackBeltMotor = new WPI_TalonSRX(BeltConstants.kBackBeltMotor);
    BottomBeltMotor = new WPI_TalonSRX(BeltConstants.kBottomBeltMotor);

    v_printCount = 0;
  }
  public void BottomBeltDrive(double bottomBeltSpeed){
    v_bottomBeltSpeed = bottomBeltSpeed;
  }
  public void FrontBeltDrive(double frontBeltSpeed){
    v_frontBeltSpeed = frontBeltSpeed;
  }
  public void BackBeltDrive(double backBeltSpeed){
    v_backBeltSpeed = backBeltSpeed;
  }

  public void DriveBelts(double bottomBeltSpeed, double backBeltSpeed, double frontBeltSpeed){
    v_bottomBeltSpeed = bottomBeltSpeed;
    v_frontBeltSpeed = frontBeltSpeed;
    v_backBeltSpeed = backBeltSpeed;
  }

public void StopBelts(){
  v_bottomBeltSpeed = 0.0;
    v_frontBeltSpeed = 0.0;
    v_backBeltSpeed = 0.0;
}



  public void timedPrintOut(){
    if(v_printCount % 100 == 0){
      //Enter print statements here!
      
    }
    v_printCount = v_printCount + 1;
  }

  @Override
  public void periodic() {
    DriveBelts(v_bottomBeltSpeed, v_backBeltSpeed, v_frontBeltSpeed);
    // This method will be called once per scheduler run
  }
}
