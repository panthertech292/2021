/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.BeltSubsystem;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GateSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.*;

//import frc.robot.subsystems.*;
//import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final static XboxController io_drivercontroller = new XboxController(Constants.kDriverController);
  private final static XboxController io_opercontroller = new XboxController(Constants.kOperController);
  private double v_AutoDistance;
  private double v_AutoAngle;
  private double v_Time;
  private double v_LeftSpeed;
  private double v_RightSpeed;
  private double v_TargetRPM;
  private double v_TargetSpeed;
  private int operAngle;
  private static boolean v_resetEncoder = false;
  

  final static int io_lefttrigger = 2;
  final static int io_righttrigger = 3;

  //Robot ID checker
  public final static AnalogInput io_IDchecker = new AnalogInput(Constants.kRobotIDChecker);

  // Subsystems
  private final DriveSubsystem s_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
  private final PickupSubsystem s_PickupSubsystem = new PickupSubsystem();
  private final GateSubsystem s_GateSubsystem = new GateSubsystem();
  private final BeltSubsystem s_BeltSubsystem = new BeltSubsystem();

  // Auto Commands
  private final Command z_AutoForward = new AutoForward(s_DriveSubsystem, v_LeftSpeed, v_RightSpeed, v_Time);
  private final Command z_AutoForwardPID = new AutoForwardPID(s_DriveSubsystem, v_LeftSpeed, v_RightSpeed, v_AutoDistance);
  private final Command z_AutoForwardEncoder = new AutoForwardEncoder(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed,
      v_RightSpeed);
  private final Command z_AutoBackwardEncoder = new AutoBackwardEncoder(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed,
      v_RightSpeed);
  private final Command z_AutoBackward = new AutoBackward(s_DriveSubsystem);
  private final Command z_AutoRight90Encoder = new AutoRight90Encoder(s_DriveSubsystem);
  private final Command z_AutoRight90Timed = new AutoRight90Encoder(s_DriveSubsystem);
  private final Command z_AutoRight90Gyro = new AutoRight90Gyro(s_DriveSubsystem, v_AutoAngle, v_LeftSpeed,
      v_RightSpeed);
  private final Command z_AutoDead = new AutoDead(s_DriveSubsystem);
  private final Command z_AutoBounce = new AutoBounce(s_DriveSubsystem);
  private final Command z_AutoSquareRight = new AutoSquareRight(s_DriveSubsystem);

  private final Command z_AutoShootTimed = new AutoShootTimed(s_ShooterSubsystem);
  private final Command z_AutoPickupTimed = new AutoPickupTimed(s_PickupSubsystem);

  private final Command z_AutoReloadFire = new AutoReloadFire(s_ShooterSubsystem, s_DriveSubsystem, s_PickupSubsystem);

  private final Command z_AutoBarrel = new AutoBarrel(s_DriveSubsystem);
  private final Command z_AutoSlalom = new AutoSlalom(s_DriveSubsystem);

  private final Command z_AutoDriveVisionCorrection = new AutoDriveVisionCorrection(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed, v_RightSpeed);

  // Vision Commands
  private final Command z_VisionAlign = new VisionAlign(s_DriveSubsystem);
  private final Command z_VisionDistance = new VisionDistance(s_DriveSubsystem, v_AutoAngle);
  private final Command z_VisionAll = new VisionAll(s_DriveSubsystem, v_AutoAngle);
  // Drive Commands
  private final Command z_DriveTeleop = new DriveTeleop(s_DriveSubsystem);

  // Shooter Commands
  private final Command z_ShooterFireHalf = new ShooterFireHalf(s_ShooterSubsystem);
  private final Command z_ShooterFireFull = new ShooterFireFull(s_ShooterSubsystem);
  private final Command z_ShooterFireGated = new ShooterFireGated(s_ShooterSubsystem, s_GateSubsystem);
  private final Command z_ShooterFireGatedSpinUp = new ShooterFireGatedSpinUp(s_ShooterSubsystem, s_GateSubsystem, v_TargetRPM);
  private final Command z_AimAdjustDown = new AimAdjustDown(s_ShooterSubsystem);
  private final Command z_AimAdjustUp = new AimAdjustUp(s_ShooterSubsystem);
  private final Command z_AimAdjustStartingPosition = new AimAdjustStartingPosition(s_ShooterSubsystem);
  private final Command z_AimAdjustNearZone = new AimAdjustNearZone(s_ShooterSubsystem);
  private final Command z_AimAdjustSecondZone = new AimAdjustSecondZone(s_ShooterSubsystem);
  private final Command z_AimAdjustThirdZone = new AimAdjustThirdZone(s_ShooterSubsystem);
  private final Command z_AimAdjustFarZone = new AimAdjustFarZone(s_ShooterSubsystem);
  private final Command z_ShooterFirePID = new ShooterFirePID(s_ShooterSubsystem, v_TargetRPM);
  private final Command z_ShooterFireBelts = new ShooterFireBelts(s_ShooterSubsystem, s_BeltSubsystem);
  private final Command z_TotalFireNearZone = new TotalFireNearZone(s_ShooterSubsystem, s_BeltSubsystem);
  private final Command z_TotalFireSecondZone = new TotalFireSecondZone(s_ShooterSubsystem, s_BeltSubsystem);
  private final Command z_TotalFireThirdZone = new TotalFireThirdZone(s_ShooterSubsystem, s_BeltSubsystem);
  private final Command z_TotalFireFarZone = new TotalFireFarZone(s_ShooterSubsystem, s_BeltSubsystem);
  private final Command z_FullShootFromFar = new FullShootFromFar(s_ShooterSubsystem, s_BeltSubsystem, s_DriveSubsystem);
  // Pickup Commands
  private final Command z_PickupRunHalf = new PickupRunHalf(s_PickupSubsystem);


  private final Command z_gate1Down = new Gate1Down(s_GateSubsystem);
  private final Command z_gate1Mid = new Gate1Up(s_GateSubsystem);
  private final Command z_gate1Max = new Gate1Max(s_GateSubsystem);
  private final Command z_GateToggleTimed = new GateToggleTimed(s_GateSubsystem);

  //Belt Commands

  private final Command z_BeltForwardAll = new BeltForwardAll(s_BeltSubsystem);
  private final Command z_BeltSchedule = new BeltSchedule(s_BeltSubsystem);
  private final Command z_BeltMoveTimed = new BeltMoveTimed(s_BeltSubsystem);
  private final Command z_BeltBackwardAll = new BeltBackwardAll(s_BeltSubsystem);

  SendableChooser<Command> o_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    configureButtonBindings();
    /*o_chooser.addOption("Auto Forward", z_AutoForward);
    o_chooser.addOption("Auto Forward Encoder", z_AutoForwardEncoder);
    o_chooser.addOption("Auto Backward Encoder", z_AutoBackwardEncoder);
    o_chooser.addOption("Auto Backward", z_AutoBackward);
    o_chooser.addOption("Auto RIght 90 Encoder", z_AutoRight90Encoder);
    o_chooser.addOption("Auto Right 90 Timed", z_AutoRight90Timed);
    o_chooser.addOption("Auto Right 90 Gyro", z_AutoRight90Gyro);*/
    o_chooser.addOption("Barrel", z_AutoBarrel);


    // Vision stuff
    o_chooser.addOption("Vision Right", z_VisionAlign);
    Shuffleboard.getTab("Autonomous").add(o_chooser);
    s_DriveSubsystem.setDefaultCommand(z_DriveTeleop);
    s_GateSubsystem.setDefaultCommand(z_gate1Down);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    final JoystickButton d_aButton = new JoystickButton(io_drivercontroller, Button.kA.value);
    final JoystickButton d_bButton = new JoystickButton(io_drivercontroller, Button.kB.value);
    final JoystickButton d_xButton = new JoystickButton(io_drivercontroller, Button.kX.value);
    final JoystickButton d_startButton = new JoystickButton(io_drivercontroller, Button.kStart.value);
    final JoystickButton d_backButton = new JoystickButton(io_drivercontroller, Button.kBack.value);
     final JoystickButton d_yButton = new JoystickButton(io_drivercontroller, Button.kY.value);
    // final JoystickButton d_lbumper = new JoystickButton(io_drivercontroller,
    // Button.kBumperLeft.value);
    // final JoystickButton d_rbumper = new JoystickButton(io_drivercontroller,
    // Button.kBumperRight.value);

    final JoystickButton o_aButton = new JoystickButton(io_opercontroller, Button.kA.value);
    final JoystickButton o_bButton = new JoystickButton(io_opercontroller, Button.kB.value);
    final JoystickButton o_xButton = new JoystickButton(io_opercontroller, Button.kX.value);
    final JoystickButton o_yButton = new JoystickButton(io_opercontroller, Button.kY.value);
    final JoystickButton o_startButton = new JoystickButton(io_opercontroller, Button.kStart.value);
    final JoystickButton o_backButton = new JoystickButton(io_opercontroller, Button.kBack.value);
    final POVButton o_dPadUp = new POVButton(io_opercontroller, 0);
    final POVButton o_dPadRight = new POVButton(io_opercontroller, 90);
    final POVButton o_dPadDown = new POVButton(io_opercontroller, 180);
    final POVButton o_dPadLeft = new POVButton(io_opercontroller, 270);
    final JoystickButton o_rBumper = new JoystickButton(io_opercontroller,Button.kBumperRight.value);
    final JoystickButton o_lBumper = new JoystickButton(io_drivercontroller,Button.kBumperLeft.value);
    // Driver Button Binds6
    //d_aButton.whileHeld(z_BeltForwardAll);
    //d_bButton.whenPressed(z_AutoBarrel);
    //d_xButton.whileHeld(z_ShooterFirePID);
    //d_xButton.whenPressed(z_ShooterFireBelts);
    //d_yButton.whenPressed(z_AimAdjustFarZone);
    
    
    
    if(getRobotID()== 0){
      d_backButton.whileHeld(z_AimAdjustDown);
      d_startButton.whileHeld(z_AimAdjustUp);
      
    }
    d_aButton.whenPressed(z_VisionAlign);
    d_bButton.whenPressed(z_AutoSquareRight);
    // o_xButton.whenPressed(z_VisionAll);
    // d_lbumper.whenPressed(m_driveJogLeft);
    // d_rbumper.whenPressed(m_driveJogRight);

    // o_bButton.whenPressed(z_AutoForward);
    // d_xButton.whenPressed(z_AutoSquareRight);
    o_aButton.whenPressed(z_TotalFireNearZone);
    o_yButton.whenPressed(z_TotalFireSecondZone);
    o_xButton.whenPressed(z_TotalFireThirdZone);
    // o_yButton.whenReleased(z_gate1Up);
    o_bButton.whenPressed(z_FullShootFromFar);
    o_startButton.whenPressed(z_BeltForwardAll);
    o_backButton.whileHeld(z_BeltBackwardAll);
    o_rBumper.whenPressed(z_ShooterFireBelts);


    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public static double getLeftSpeed() {
    return io_drivercontroller.getY(GenericHID.Hand.kLeft);
  }

  public static double getRightSpeed() {
    return io_drivercontroller.getY(GenericHID.Hand.kRight);
  }
 /* public static double getDriveSpeedAdjust() {
    double v_driveSpeedAdjust;
    v_driveSpeedAdjust = 1 - ((io_drivercontroller.getRawAxis(io_righttrigger) / 2.5)
        + (io_drivercontroller.getRawAxis(io_lefttrigger) / 2.5));
    return v_driveSpeedAdjust;
  } */

  public static int getRobotID() {
    if (io_IDchecker.getVoltage() < 2.5) {
      return Constants.kProductionBotID; //Returns robot ID, production bot, red tape on jumper .1
    }
    else{
      return Constants.kBackupBotID; //Returns robot ID, backup bot, black tape on jumper .0
    }
  }
  

  public Command getAutonomousCommand() {
     //An ExampleCommand will run in autonomous
    return o_chooser.getSelected();
  }
  /*public static double getShooterSubsystemRate(){
    double rate;
    rate = ShooterSubsystem.getShooterSpeed();
    return rate;
  }*/
  public Command getInitialAimCommand(){
    if(getRobotID()==0){
    return z_AimAdjustStartingPosition;
    }
    else{
      return z_DriveTeleop;
    }
  }
  
  

}
