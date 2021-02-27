/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private final WPI_TalonSRX ShooterMotor;
  private final WPI_TalonSRX AimMotor;
  private DigitalInput BallSensor;
  private DigitalInput AimSwitch;
  private double v_shooterSpeed;
  private double v_aimSpeed;
  private final Timer Timer;
  private Encoder ShooterEncoder;
  private Encoder AimEncoder;
  private double v_encoderSetPointShooter;
  private double v_RPMTarget;
  private int v_loops;
  public int v_shooterSpinIndicator;
  public double v_aimEncoderValue;

  //Average Variables
  //TO DO MAKE FIT STANDARS
  private int v_loop = 0;
  private double v_averageRate = 0;
  private double v_totalRate = 0;
  private int v_zeroNum = 0;
  private double v_previousError = 0;
  private double v_integral = 0;

  //Average Array
  private final int c_maxValues = 5;
  private double[] v_averageArray = new double[c_maxValues];
  private int v_arrayPointer = 0;
  private double v_arrayTotal;
  private double v_arrayAverage;



 /* private int counter;
  //private double e1;
  private double e2;
  private double e3;
  private double e4;
  private boolean close;
  private double threshold;
  private double previousEncoderRate; */
  private PowerDistributionPanel PDP;
  

  public ShooterSubsystem() {
    ShooterMotor = new WPI_TalonSRX(ShooterConstants.kShooterMotor);
    AimMotor = new WPI_TalonSRX(ShooterConstants.kAimMotor);
    ShooterEncoder = new Encoder(ShooterConstants.kShooterEncoderChannel1, ShooterConstants.kShooterEncoderChannel2);
    AimEncoder = new Encoder(ShooterConstants.kAimEncoder1, ShooterConstants.kAimEncoder2);
    Timer = new Timer();
    PDP = new PowerDistributionPanel();
    v_RPMTarget = SmartDashboard.getNumber("v_RPMTarget", 0.0);
    SmartDashboard.putNumber("v_RPMTarget", 0.0);
    BallSensor = new DigitalInput(5);
    AimSwitch = new DigitalInput(ShooterConstants.kAimLimitSwitch);
    

    v_shooterSpeed = 0.0;
    v_aimSpeed = 0.0;
    v_aimEncoderValue = 0.0;

    addChild("ShooterMotor", ShooterMotor);
    addChild("ShooterEncoder", ShooterEncoder);
    addChild("AimEncoder", AimEncoder);
    ShooterMotor.setNeutralMode(NeutralMode.Brake);

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

  public double getEncoderPosition(){
    return ShooterEncoder.get();
  }

  public double getEncoderRate(){
    return Math.abs(ShooterEncoder.getRate());
  }

  public void resetEncoderPosition(){
    ShooterEncoder.reset();
  }
  public double getAimEncoder(){
    v_aimEncoderValue = AimEncoder.get();
    return v_aimEncoderValue;
  }
  public void resetAimEncoder(){
    if(AimSwitch.get() == true){
      v_aimEncoderValue = 0;
    }
  }

  public void changeSetSpeed(double desiredSpeed){
    v_shooterSpeed = desiredSpeed;
    //System.out.println("Shooter ChangeSetSpeed ########################");
  }

  public void changeEncoderSetPoints(double desiredEncoder){
    v_encoderSetPointShooter = desiredEncoder;
  }
  
public double getEncoderAverageRateArray(){
  double v_currentRate = getEncoderRate();
  
  v_averageArray[v_arrayPointer] = v_currentRate;
  v_arrayPointer = v_arrayPointer + 1;
  if (v_arrayPointer == c_maxValues){
    v_arrayPointer = 0;
  }
  v_arrayTotal = v_averageArray[0] + v_averageArray[1] + v_averageArray[2] + v_averageArray[3] + v_averageArray[4];
  v_arrayAverage = v_arrayTotal / c_maxValues;

  return v_arrayAverage;
}

public void resetEncoderAverageRateArray() {
  v_averageArray[0] = 0;
  v_averageArray[1] = 0;
  v_averageArray[2] = 0;
  v_averageArray[3] = 0;
  v_averageArray[4] = 0;
  v_arrayPointer = 0;
}



/*
public double getEncoderAverageRate(){
double currentRate = getEncoderRate();
  totalRate = totalRate + getEncoderRate();
  averageRate = totalRate / Loop;
  Loop = Loop + 1;//TEST THIS
  if(currentRate <= 50){
    zeroNum = zeroNum + 1;
  }
  if (zeroNum >= 5){
    averageRate = 0;
    zeroNum = 0;
    totalRate = 0;
    Loop = 0;
  }
  return averageRate;


}
*/
 /* public void changeRange(double desiredRange){
    m_shooterSpeed = ShooterConstants.kSpeedScale * desiredRange + ShooterConstants.kSpeedConstant;
    m_EncoderSetPointShooter = ShooterConstants.kEncoderScale * desiredRange + ShooterConstants.kEnocderConstant;
  }

  public void getShooterMode(double desiredRange){
    m_shooterSpeed = ShooterConstants.kSpeedScale * desiredRange + ShooterConstants.kSpeedConstant;
    m_EncoderSetPointShooter = ShooterConstants.kEncoderScale * desiredRange + ShooterConstants.kEnocderConstant;
  } */


  public double getEncoderDownSetpoint() {
    return  SmartDashboard.getNumber("Setpoint - Shooter Encoder", 1);
  }
/*
  public double QuickPid(double set, double actual) {
      double m_set;
      double m_actual;
      double delta = 0; double previousdelta; double integral = 0; double derivative = 0.0;
      double p = 1.0; double i = 0.0; double d = 0.0; double rcw = 0.0;
      double timeInterval = 0.02;
      
      m_set = set;
      m_actual = actual;
      
      delta = m_set - m_actual;
      integral += delta * timeInterval;
      derivative = (delta - previousdelta) / 0.02;



      previousdelta = delta;

  }
*/
public int checkPDPVoltage(){
  return (int) Math.floor(PDP.getVoltage()*10);
}

/*public double getDesiredEncoderSetpoint(){
  int voltage = (checkPDPVoltage()-100);
 int power = (int) ((Math.floor(m_shooterSpeed*10))-1.0);

 int index = ((power*100)+voltage);



 

}*/


public double getEstimatedEncoderValue(){
double encoderValue = 0;
double m_shooterFloor = Math.floor(v_shooterSpeed*10);
if(m_shooterFloor <= 4){
  encoderValue = 150000;
}
if(m_shooterFloor == 5){
  encoderValue = 185000;
}
if(m_shooterFloor == 6){
  encoderValue = 220000;
}
if(m_shooterFloor == 7){
  encoderValue = 255000;
}

if(m_shooterFloor == 8){
  encoderValue = 290000;
}

if(m_shooterFloor == 9){
  encoderValue = 325000;
}
if(m_shooterFloor == 10){
  encoderValue = 360000;
}


return encoderValue;
}



/*public void initCloseEnough(){
  double set = getEncoderRate();
  e1 = set;
  e2 = set;
  e3 = set;
  e4 = set;
  counter = 0;
  
  previousEncoderRate = set;
  threshold = ShooterConstants.kthreshold;
}

public boolean isCloseEnough(){
  
double actual;
double delta;
double aveDelta;
//System.out.println(counter);
counter= counter+1;
if(counter >= 4 && getEncoderRate() >= 250000){
   counter = 0;
  actual = getEncoderRate();
  
  delta = previousEncoderRate - actual;
  aveDelta = (delta + e2 + e3 + e4)/4;
  //System.out.println("aveDelta="+ aveDelta+ "Actual ="+ actual);



  if(aveDelta < threshold){
  close = true;

  } else{
    close = false;
  }
  e1 = e2;
  e2 = e3;
  e3 = e4;
  e4 = delta;
  return close;
}else {
  return close;
}
}
*/
// BE CAREFUL! Running for too long causes the motor to overheat due to the rapid changes.
public double PID(double v_RPMTarget){
  double error = 0.0;
  double P = 0.000095

  ;
  double I = 0.0000007;
  double D = 0.0;
  double derivative = 0.0;
  double rcw = 0.0;
  if (v_RPMTarget > 50000){
  error = v_RPMTarget - getEncoderRate(); // Error = Target - Actual
  v_integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
  derivative = (error - v_previousError) / .02;
  v_previousError = error;
  rcw = P*error + I*v_integral + D*derivative;
 SmartDashboard.putNumber("error", error);
 SmartDashboard.putNumber("RCW", rcw);
 System.out.println(getEncoderRate());
  return rcw;
  }
  else{
    error = 0;
    rcw = 0;
    v_integral = 0;
    v_previousError = 0;
    return 0.0;
  }
}
public void UpdateTargetRPM(){
  v_RPMTarget = SmartDashboard.getNumber("v_RPMTarget", 0.0);
}

public int MotorSpinUp(double TargetRPM){
  if(v_loops<500){
    v_loops = v_loops +1;
  if (getEncoderRate()>(TargetRPM-(TargetRPM*.1)) && (TargetRPM+(TargetRPM*.1))>getEncoderRate()){
    
    v_loops = 0;
    System.out.println("Motor Spun Up");
    return 1;
  }
  else{
    return 0;}
}
else{
  System.out.println("Motor Timed Out");
  v_loops = 0;
  return 2;
  
}
//1 = ready to shoot, 0 = not ready to shoot, 2 = timed out/shoot aborted
}
public void ShooterAimUp(){
  v_aimSpeed = 0.2;
  }
public void ShooterAimDown(){
  if(AimSwitch.get() == false)
    v_aimSpeed = -0.2;
  }
public void ShooterAimStop(){
  v_aimSpeed = 0.0;
}
public boolean AimSwitchValue(){
  return AimSwitch.get();
}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    UpdateTargetRPM();
    if(v_RPMTarget<=50000){
      ShooterMotor.set(v_shooterSpeed/*+RobotContainer.getShooterSpeedAdjust()*/);
    }
    else{
      ShooterMotor.set(PID(v_RPMTarget));
    }
   //System.out.println(AimSwitch.get());
      AimMotor.set(v_aimSpeed);
    
  //  System.out.println(getAimEncoder());
  
    SmartDashboard.putNumber("ShooterEncoderRate", getEncoderRate());
    
    SmartDashboard.getNumber("v_RPMTarget", v_RPMTarget);
    
    
    
    
  }
}

