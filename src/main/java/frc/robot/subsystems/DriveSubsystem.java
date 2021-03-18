package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final WPI_TalonSRX FrontLeftMotor;
  private final WPI_TalonSRX FrontRightMotor;
  private final WPI_TalonSRX BackLeftMotor;
  private final WPI_TalonSRX BackRightMotor;

  private final SpeedControllerGroup LeftSide;
  private final SpeedControllerGroup RightSide;

  private final Timer Timer;

  private final DifferentialDrive Drive;

  private double v_leftSpeed;
  private double v_rightSpeed;
  private double v_setPointLeft;
  private double v_setPointRight;

  private double v_zeroLeftPosition;
  private double v_zeroRightPosition;
  private double v_zeroAngle;
  private double v_deltaAngle;
  private double c_encoderConversion;
  private double v_integral;
  private double v_previousError;
  private double v_ticker = 0;
  private double v_previousAngle;
  private int v_pidEnabler = 0;
  

  private double v_targetPower;
  //Put this into constants.java
  private int c_modeTeleop = 0;
  private int c_modeSetPoint = 1;
  private int v_driveMode = c_modeTeleop;

  private double v_limeLightX;
  private double v_limeLightY;
  private double v_limeLightArea;
  private double c_VisionAreaTarget;
  private int v_loopCount;
  private int v_visionOverride;

  private int v_printCount;

  private double v_driveSpeedMod;

  //Inputs
  private DigitalInput v_testSwitch;
  private DigitalInput opticalSensor;
  private DigitalInput testDio;
  private AnalogInput sonarSensor;

  //Network Tables
  private NetworkTableEntry v_TableEntrytx;
  private NetworkTableEntry v_TableEntryty;
  private NetworkTableEntry v_TableEntryta;

  private NetworkTable table;

  AHRS ahrs;
 
  public DriveSubsystem() {

    FrontLeftMotor = new WPI_TalonSRX(DriveConstants.kFrontLeftMotor);
    FrontRightMotor = new WPI_TalonSRX(DriveConstants.kFrontRightMotor);
    BackLeftMotor = new WPI_TalonSRX(DriveConstants.kBackLeftMotor);
    BackRightMotor = new WPI_TalonSRX(DriveConstants.kBackRightMotor);
    v_testSwitch = new DigitalInput(DriveConstants.kTestSwitch);
    opticalSensor = new DigitalInput(DriveConstants.kOpticalPort);
    sonarSensor = new AnalogInput(DriveConstants.kSonarPort);
    testDio = new DigitalInput(6);
    

    LeftSide = new SpeedControllerGroup(FrontLeftMotor, BackLeftMotor);
    RightSide = new SpeedControllerGroup(FrontRightMotor, BackRightMotor);

    table = NetworkTableInstance.getDefault().getTable("limelight");
    v_TableEntrytx = table.getEntry("tx");
    v_TableEntryty = table.getEntry("ty");
    v_TableEntryta = table.getEntry("ta");

    v_limeLightX = v_TableEntrytx.getDouble(0.0);
    v_limeLightY = v_TableEntryty.getDouble(0.0);
    v_limeLightArea = v_TableEntryta.getDouble(0.0);

    c_VisionAreaTarget = DriveConstants.kVisionAreaTarget;
    v_loopCount = 0;
    v_visionOverride = 0;
    v_ticker = 0;
    v_previousAngle = 0;

    v_driveSpeedMod = 0.95;

    SmartDashboard.putNumber("Back Speed", 0.0);
    SmartDashboard.putNumber("Front Speed", 0.0);

    v_printCount = 0;

    Drive = new DifferentialDrive(LeftSide,RightSide);


    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    BackLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    BackRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    v_setPointLeft = 0.0;
    v_setPointRight = 0.0;

    Timer = new Timer();
    c_encoderConversion = 4096;

    addChild("Drive", Drive);
    addChild("Left Side", LeftSide);
    addChild("Right Side", RightSide);

    FrontLeftMotor.setNeutralMode(NeutralMode.Brake);
    FrontRightMotor.setNeutralMode(NeutralMode.Brake);
    BackRightMotor.setNeutralMode(NeutralMode.Brake);
    BackLeftMotor.setNeutralMode(NeutralMode.Brake);
    //SmartDashboard.putNumber("Ratio Proportion", 0.0);

  }

  public void updateLimeLight() {

    v_limeLightX = v_TableEntrytx.getDouble(0.0);
    v_limeLightY = v_TableEntryty.getDouble(0.0);
    v_limeLightArea = v_TableEntryta.getDouble(0.0);
  }

  public void differentialDrive(double leftspeed, double rightspeed) {
    v_leftSpeed = -leftspeed;
    v_rightSpeed = -rightspeed;
    Drive.tankDrive(v_leftSpeed, v_rightSpeed);
  }

  public void resetTimer() {
    Timer.reset();
    Timer.start();
  }

  public double getTimerValue() {
    return Math.abs(Timer.get());
  }

  // Encoder subroutines
  public double getLeftEncoderValue() {
    return BackLeftMotor.getSelectedSensorPosition();
  }

  public double getLeftPosition() {
    return ((BackLeftMotor.getSelectedSensorPosition() - v_zeroLeftPosition) / c_encoderConversion) * (Math.PI * 6);
  }

  public void zeroLeftPosition() {
    v_zeroLeftPosition = BackLeftMotor.getSelectedSensorPosition();
  }

  public double getZeroPosition() {
    return v_zeroLeftPosition;
  }

  public void zeroAngle() {
    ahrs.zeroYaw();
    v_ticker = 0;
    v_previousAngle = 0;
  }

  public double getAbsCurrentAngle() {
    return Math.abs(ahrs.getYaw());
  }

  public double getCurrentAngle() {
    return ahrs.getYaw();
    
  }
  
  public void zeroRightPosition() {
    v_zeroRightPosition = BackRightMotor.getSelectedSensorPosition();
  }

  public double getTotalAngle() {
    double angle;
    angle = getCurrentAngle();
    if (Math.abs(angle - v_previousAngle) >= 45) {
      System.out.println("Ticking!!!!! (You off :))");
      System.out.println("Angle =" + angle);
      System.out.println("Previous Angle =" + v_previousAngle);

      if((v_previousAngle)/Math.abs(v_previousAngle) !=0 && v_previousAngle !=0){
        v_ticker = v_ticker + (v_previousAngle)/Math.abs(v_previousAngle);
      }
    }
  v_previousAngle = angle;
  return Math.abs((2.0 * v_ticker * 180.0) + (angle));
  }



  public boolean gyroFinish(double angle) {
    //SmartDashboard.putNumber("Gyro Angle", getTotalAngle());
    if (angle <= getTotalAngle()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean totalFinish(double angle) {
    return (gyroFinish(angle) && encoderFinish((angle / 360) * 2 * Math.PI * 21));
  }

  public void zeroDistanceSensors() {
    zeroLeftPosition();
    zeroAngle();
    zeroRightPosition();
  }

  // Vision
  public boolean visionFinish() {
    if (v_limeLightArea!=0.0&&-1 <= v_limeLightX && v_limeLightX <= 1 || v_visionOverride == 1) {
      v_visionOverride = 0;
      return true;
    }

    else {
      return false;
    }
  }

  public boolean visionFinishDistance() {
    if (c_VisionAreaTarget - 0.1 <= v_limeLightArea && c_VisionAreaTarget + 0.1 >= v_limeLightArea) {
      return true;
    } else {
      return false;
    }
  }

  // Actually turns robot right, aimed too far left
  public void visionAlignLeft() {
    if (v_limeLightX > 1.3) {
      v_loopCount = v_loopCount + 1;
      if (RobotContainer.getRobotID() == Constants.kProductionBotID) {
        changePowerSetPoints(DriveConstants.kProdBotVisionAlignSpeedDefault + Math.abs(v_limeLightX*0.007), -DriveConstants.kProdBotVisionAlignSpeedDefault - Math.abs(v_limeLightX*0.007));
      } 
      else {
        changePowerSetPoints(DriveConstants.kVisionAlignSpeedDefault + Math.abs(v_limeLightX*0.007), -DriveConstants.kVisionAlignSpeedDefault- Math.abs(v_limeLightX*0.007));
      }
      // 50 loops = 1 sec?
      if (v_loopCount >= 150) {
        System.out.println("Alignment Aborted");
        v_loopCount = 0;
        v_visionOverride = 1;
      }
    }
  }

  // Actually turns robot left, aimed too far right
  public void visionAlignRight() {
    if (v_limeLightX < -1.3) {
      v_loopCount = v_loopCount + 1;
      System.out.println(v_loopCount);
      System.out.println("Trying to align right!");
      if (RobotContainer.getRobotID() == Constants.kProductionBotID) {
        changePowerSetPoints(-DriveConstants.kProdBotVisionAlignSpeedDefault- Math.abs(v_limeLightX*0.007),DriveConstants.kProdBotVisionAlignSpeedDefault+ Math.abs(v_limeLightX*0.007));
      } 
      else {
        changePowerSetPoints(-DriveConstants.kVisionAlignSpeedDefault- Math.abs(v_limeLightX*0.007), DriveConstants.kVisionAlignSpeedDefault+ Math.abs(v_limeLightX*0.007));
      }
      if (v_loopCount >= 150) {
        System.out.println("Alignment Aborted");
        v_loopCount = 0;
        v_visionOverride = 1;
      }
    }
  }

  public void visionDistanceArea() {
    if (v_limeLightArea > c_VisionAreaTarget) {
      if (RobotContainer.getRobotID() == Constants.kProductionBotID) {
        changePowerSetPoints(-DriveConstants.kProdBotVisionForwardSpeedDefault,
            -DriveConstants.kProdBotVisionForwardSpeedDefault);
      } else {
        changePowerSetPoints(-DriveConstants.kVisionForwardSpeedDefault, -DriveConstants.kVisionForwardSpeedDefault);
      }
    }
    if (v_limeLightArea < c_VisionAreaTarget) {
      if (RobotContainer.getRobotID() == Constants.kProductionBotID) {
        changePowerSetPoints(DriveConstants.kProdBotVisionForwardSpeedDefault,
            DriveConstants.kProdBotVisionForwardSpeedDefault);
      } else {
        changePowerSetPoints(DriveConstants.kVisionForwardSpeedDefault, DriveConstants.kVisionForwardSpeedDefault);
      }
    }
  }

  public double getRightPosition() {
    return ((BackRightMotor.getSelectedSensorPosition() - v_zeroRightPosition) / c_encoderConversion) * (Math.PI * 6);
  }

  public double getRightEncoderValue() {
    return BackRightMotor.getSelectedSensorPosition();
  }

  // Distance is in Inches
  public boolean encoderFinish(double distance) {
    double average = 0;
    // return distance <= (Math.abs(getLeftPosition()));
    average = (Math.abs(getLeftPosition()) + Math.abs(getRightPosition())) / 2;
    return distance <= average;
  }
  public boolean encoderTurnFinish(double diameter, double angle){
    double circumference = diameter*Math.PI;
    double distance = (angle/360)*(circumference);
    System.out.println("Hello?");
    if(v_leftSpeed>v_rightSpeed){
      System.out.println("Left Position = " +   Math.abs(getLeftPosition()));
      return distance <= Math.abs(getLeftPosition());
    }
    if(v_rightSpeed>v_leftSpeed){
      System.out.println("Right Position = " +   Math.abs(getRightPosition()));
      return distance <= Math.abs(getRightPosition());
    }
    else{return false;}
  }
  public double rotateRobot(double m_angle) {

    return (((42 * Math.PI) * (m_angle / 360)) * .5);
    // .769 is a fudge factor for carpet to make the calculation work
  }

  // Auto
  public void changePowerSetPoints(double setPointLeft, double setPointright) {
    v_setPointLeft = setPointLeft;
    v_setPointRight = setPointright;
  }

  public void driveModePowerSetPoint() {
    v_driveMode = c_modeSetPoint;
  }

  public void driveAuto() {
    differentialDrive(-v_setPointLeft, -v_setPointRight);
  }

  // Teleop
  public void driveModeTeleop() {
    v_driveMode = c_modeTeleop;
  }

  public void driveTeleop() {
    differentialDrive((RobotContainer.getLeftSpeed()*v_driveSpeedMod), RobotContainer.getRightSpeed()*v_driveSpeedMod);
  }
  public void slowDrive(){
    if (RobotContainer.getLeftSpeed() < 0 && RobotContainer.getRightSpeed() > 0) {
      System.out.println("Going slower!");
      v_driveSpeedMod = .80;
    }
    if (RobotContainer.getLeftSpeed() > 0 && RobotContainer.getRightSpeed() < 0){
      System.out.println("Going slower!");
      v_driveSpeedMod = .80;
    }
    if (RobotContainer.getLeftSpeed() > 0 && RobotContainer.getRightSpeed() > 0){
      v_driveSpeedMod = .90;
    }
    if (RobotContainer.getLeftSpeed() < 0 && RobotContainer.getRightSpeed() < 0){
      v_driveSpeedMod = .90;
    }
  }

  // Changes Right Side based on Left Side
  public double LeftPID(double v_targetPower) {
    double error;
    double P = 0.0256 * 2 * 1.5;
    double I = // 0.00812;
        0.0;
    double D = 0.0;
    double derivative;
    double rcw;
    error = Math.abs(getLeftPosition()) - Math.abs(getRightPosition()); // Error = Target - Actual
    v_integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - v_previousError) / .02;
    v_previousError = error;
    rcw = P * error + I * v_integral + D * derivative;
    

    v_rightSpeed = v_targetPower + rcw;
    return v_rightSpeed;
  }

  public double RightPID(double v_targetPower) {
    double error;
    double P = 0.0256 * 2 * 1.5;
    double I = // 0.00812;
        0.0;
    double D = 0.0;
    double derivative;
    double rcw;
    error = Math.abs(getRightPosition()) - Math.abs(getLeftPosition()); // Error = Target - Actual
    v_integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - v_previousError) / .02;
    v_previousError = error;
    rcw = P * error + I * v_integral + D * derivative;
    
    v_leftSpeed = v_targetPower + rcw;
    return v_leftSpeed;
  }

  public double RatioLeftPID(double v_desiredRatio, double v_targetPower) {
    double error = 2.15789;
    
    double P =  0.0256*2.5 *0.0;
    //SmartDashboard.getNumber("Ratio Proportion", 0.0256 * 2 * 1.5 * 2 * 2 * 2);
    double I = // 0.00812;
        0.0;
    double D = 0.0;
    double derivative;
    double rcw;
    if (Math.abs(getLeftPosition()) == 0) {
      error = 2.15789;
    } else {
      error = -1 * (v_desiredRatio - (Math.abs(getRightPosition()) / Math.abs(getLeftPosition())));
    }
    
    // Error = Target - Actual
   
    v_integral += (error*0.02); // Integral is increased by the error*time (which
    // is .02 seconds using normal IterativeRobot)
    
    derivative = (error - v_previousError) / 0.02;
    v_previousError = error;
    rcw = P * error + I * v_integral + D * derivative;
    
    SmartDashboard.putNumber("RCW", rcw);
    SmartDashboard.putNumber("Turn Error", error);

    v_leftSpeed = v_targetPower + rcw;
    
    return v_leftSpeed;
  }

  public double RatioRightPID(double v_desiredRatio, double v_targetPower) {
    double error = 2.15789;
    
    double P =  0.0256*2.5 *0.0;
  //  SmartDashboard.getNumber("Ratio Proportion", 0.0256 * 2 * 1.5 * 2 * 2 * 2);
    double I = // 0.00812;
        0.0;
    double D = 0.0;
    double derivative;
    double rcw;
    if (Math.abs(BackRightMotor.getSelectedSensorVelocity()) == 0) {
      error = 2.15789;
    } else {
      error = -1 * (v_desiredRatio - (Math.abs(getLeftPosition()) / Math.abs(getRightPosition())));

    } // Error = Target - Actual
    System.out.println("Ratio =  " + Math.abs(getLeftPosition()) / Math.abs(getRightPosition()));
    
    v_integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - v_previousError) / .02;
    v_previousError = error;
    rcw = P * error + I * v_integral + D * derivative;
    
    // SmartDashboard.putNumber("RCW", rcw);
    SmartDashboard.putNumber("RCW", rcw);
    SmartDashboard.putNumber("Turn Error", error);
    v_rightSpeed = v_targetPower + rcw;
    return v_rightSpeed;
  }

  public double PerceivedAngle(double distance) {
    return (360 * distance) / (Math.PI * 21);
  }

  public double AnglePID(double v_targetAngle, double v_rightSpeedBase) {
    
    double error;
    double P = 0.0256 * 0.5 * 0.5 * 0.4 * 2 * 2 * 2 * 1.35;
    double I = 0.0;// 0.00812;
    double D = 0.01;
    double derivative;
    double rcw;
    error = -1 * (v_limeLightX - v_targetAngle);
    v_integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
                                 // IterativeRobot)
    derivative = (error - v_previousError) / .02;
    v_previousError = error;
    rcw = P * error + I * v_integral + D * derivative;
    if (error <= 0) {
      v_rightSpeed = v_rightSpeedBase - rcw;
    }
    if (error > 0) {
      v_rightSpeed = v_rightSpeedBase + rcw;
    }
   // System.out.println(v_rightSpeed);
    return v_rightSpeed;
  }

  public void initializePID() {
    v_integral = 0;
    v_previousError = 0;
  }

  public double visionTargetSensor() {
   // System.out.println(v_limeLightArea);
    return v_limeLightArea;
  }

  public double limelightXAngle(){
    return v_limeLightX;
  }
  public boolean gyroWithVisionFinish(double AutoAngle, double v_turnDirection){
    boolean v_done = false;
    if((v_limeLightArea > 0.1) && (AutoAngle-40 <= getAbsCurrentAngle())){
      if(limelightXAngle()*v_turnDirection <= 0 ){
        v_done = true;
      }
      else{
        v_done = false;
      }
    }
    return v_done;
  }
  public void timedPrintOut(){
    if(v_printCount % 100 == 0){
    //Enter print statements here!  
    }
    v_printCount = v_printCount + 1;
    ahrs.getWorldLinearAccelX();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Sets drive mode
    if (v_driveMode == c_modeTeleop) {
      driveTeleop();
      slowDrive();
    } else {
      driveAuto();
    }
    
    updateLimeLight();
    timedPrintOut();
    
    //  SmartDashboard.putNumber("LimelightX", v_limeLightX);
  }
}