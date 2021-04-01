/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  //Motors
  private final WPI_TalonSRX ShooterMotor;
  private final WPI_TalonSRX AimMotor;
  //Sensors
  private DigitalInput BallSensor;
  private DigitalInput AimSwitch;
  private static Encoder ShooterEncoder;
  private Encoder AimEncoder;
  // Variables
  private double v_shooterSpeed;
  private double v_aimSpeed;
  private final Timer Timer;
  private double v_encoderSetPointShooter;
  private double v_RPMTarget;
  private int v_loops;
  public int v_shooterSpinIndicator;
  public double v_aimEncoderValue;
  private int v_loop = 0;
  private double v_averageRate = 0;
  private double v_totalRate = 0;
  private int v_zeroNum = 0;
  private double v_previousError = 0;
  private double v_integral = 0;
  private double v_PIDSetpoint;
  private boolean v_reset = false;
  // PDP
  private PowerDistributionPanel PDP;
  private PIDController ShooterPID;

  public ShooterSubsystem() {

    // Motors
    ShooterMotor = new WPI_TalonSRX(ShooterConstants.kShooterMotor);
    AimMotor = new WPI_TalonSRX(ShooterConstants.kAimMotor);
    ShooterMotor.setNeutralMode(NeutralMode.Brake);
    addChild("ShooterMotor", ShooterMotor);
    // Sensors
    ShooterEncoder = new Encoder(ShooterConstants.kShooterEncoderChannel1, ShooterConstants.kShooterEncoderChannel2);
    AimEncoder = new Encoder(ShooterConstants.kAimEncoder1, ShooterConstants.kAimEncoder2);

    AimSwitch = new DigitalInput(ShooterConstants.kAimLimitSwitch);
    addChild("ShooterEncoder", ShooterEncoder);
    addChild("AimEncoder", AimEncoder);
    Timer = new Timer();
    // Variables
    v_RPMTarget = 0.0;// SmartDashboard.getNumber("v_RPMTarget", 0.0);
    // SmartDashboard.putNumber("v_RPMTarget", 0.0);
    v_shooterSpeed = 0.0;
    v_aimSpeed = 0.0;
    v_aimEncoderValue = 0.0;
    v_reset = false;
    // PDP
    PDP = new PowerDistributionPanel();
    ShooterPID = new PIDController(0.000095, 0.0000007, 0.0);
    ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  public void ShooterStop() {
    v_shooterSpeed = ShooterConstants.kShooterStop;
  }

  public void resetTimer() {
    Timer.reset();
    Timer.start();
  }

  public Double getTimerValue() {
    return Math.abs(Timer.get());
  }

  public double getEncoderRate() {
    return Math.abs(ShooterMotor.getSelectedSensorVelocity());
  }

  public double getAimEncoder() {
    v_aimEncoderValue = AimEncoder.get();
    return v_aimEncoderValue;
  }

  public void resetAimEncoder() {
    if (AimSwitch.get() == true) {
      AimEncoder.reset();
    }
  }

  public void changeSetSpeed(double desiredSpeed) {
    v_shooterSpeed = desiredSpeed;
  }

  public void changeEncoderSetPoints(double desiredEncoder) {
    v_encoderSetPointShooter = desiredEncoder;
  }

  public double getEncoderDownSetpoint() {
    return 0.0; // SmartDashboard.getNumber("Setpoint - Shooter Encoder", 1);
  }

  public int checkPDPVoltage() {
    return (int) Math.floor(PDP.getVoltage() * 10);
  }

  // BE CAREFUL! Running for too long causes the motor to overheat due to the
  // rapid changes.
  public double PID(double v_RPMTarget) {
    double error = 0.0;
    double P = 0.000095;
    double I = 0.0000007;
    double D = 0.0;
    double derivative = 0.0;
    double rcw = 0.0;
    if (v_RPMTarget > 50000) {
      error = v_RPMTarget - getEncoderRate(); // Error = Target - Actual
      v_integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
                                   // IterativeRobot)
      derivative = (error - v_previousError) / .02;
      v_previousError = error;
      rcw = P * error + I * v_integral + D * derivative;
      // SmartDashboard.putNumber("error", error);
      // SmartDashboard.putNumber("RCW", rcw);
      System.out.println(getEncoderRate());
      return rcw;
    } else {
      error = 0;
      rcw = 0;
      v_integral = 0;
      v_previousError = 0;
      return 0.0;
    }
  }

  public void UpdateTargetRPM() {
    // v_RPMTarget = SmartDashboard.getNumber("v_RPMTarget", 0.0);
  }

  public int MotorSpinUp(double TargetRPM) {
    if (v_loops < 500) {
      v_loops = v_loops + 1;
      if (getEncoderRate() > (TargetRPM - (TargetRPM * .1)) && (TargetRPM + (TargetRPM * .1)) > getEncoderRate()) {
        v_loops = 0;
        System.out.println("Motor Spun Up");
        return 1;
      } else {
        return 0;
      }
    } else {
      System.out.println("Motor Timed Out");
      v_loops = 0;
      return 2;
    }
    // 1 = ready to shoot, 0 = not ready to shoot, 2 = timed out/shoot aborted
  }

  public void ShooterAimUp() {
    v_aimSpeed = 0.65;
  }

  public void ShooterAimDown() {
    if (AimSwitch.get() == false) {
      v_aimSpeed = -0.65;
    }
  }

  public void ShooterAimStop() {
    v_aimSpeed = 0.0;
  }

  public boolean AimSwitchValue() {
    return AimSwitch.get();
  }

  public void ChangePIDSetpoint(double newSetpoint) {
    v_PIDSetpoint = newSetpoint;
  }

  public void ResetPID() {
    ShooterPID.reset();
  }

  public boolean aimResetCheck(){
    if(AimSwitchValue() == true){
      v_reset = true;
    }
    return v_reset;
  }

 
  
  @Override
  public void periodic() {
  // This method will be called once per scheduler run
    UpdateTargetRPM();
    
    //ShooterMotor.set(ShooterPID.calculate(getShooterSpeed(), v_PIDSetpoint));
    if(v_RPMTarget<=50000){
     ShooterMotor.set(v_shooterSpeed/*+RobotContainer.getShooterSpeedAdjust()*/);
     //ShooterMotor.set(ShooterPID.calculate(getEncoderRate(), v_PIDSetpoint));
    }
    else{
      //ShooterMotor.set(PID(v_RPMTarget));
    }
    AimMotor.set(v_aimSpeed);
   // System.out.println(getAimEncoder());
    resetAimEncoder();
    aimResetCheck();
    
   // SmartDashboard.putNumber("ShooterEncoderRate", getEncoderRate());
   // SmartDashboard.getNumber("v_RPMTarget", v_RPMTarget);
  }
}


