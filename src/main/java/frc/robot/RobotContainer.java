// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.Constants.OIConstants;
import frc.robot.simulation.FieldSim;

//Import Commands here
import frc.robot.commands.ArmIntake.*;
import frc.robot.commands.Compressor.*;
import frc.robot.commands.Conveyor.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.FloorIntake.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Swerve.JogDriveModule;
import frc.robot.commands.Swerve.JogTurnModule;
import frc.robot.commands.Swerve.PositionTurnModule;
import frc.robot.commands.Swerve.SetSwerveDrive;
import frc.robot.commands.Swerve.ToggleFieldOriented;
//import frc.robot.commands.TankDrive.*;
import frc.robot.commands.Turret.*;

//Import Subsystems here
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ArmIntake;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.TankDrive;
/*
 * to subsystems add: arm, turret, elevator, floorintake, armintake, conveyorbelt, limelight,
 * ledlights, compressor, swervedrive, tankdrive, shooter, linebreak sensors, color sensors,
 * limit switches, flight sensor, gunner, driver
 */

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /*
   * This is where agent names and abilities are stored
   * You can add different speeds and autonomous routines into each
   * section. 
   */
  ArmIntake armIntake = new ArmIntake("alexis", 0.4);
  Compressor compressor = new Compressor("charzard");
  Conveyor conveyor = new Conveyor("conner", 0.4);
  Elevator elevator = new Elevator("elle");
  FloorIntake floorIntake = new FloorIntake("frank", 0.4);
  Shooter shooter = new Shooter("sunny", 0.4);
  TankDrive tankDrive = new TankDrive("wally", 0.4);
  Turret turret = new Turret("todd"); 
  
   
  // Driver & Gunner controllers
  static Joystick leftJoystick = new Joystick(OIConstants.kDriverControllerPort);
  static Joystick rightJoystick = new Joystick(OIConstants.kDriverControllerPort);

  

  private XboxController m_DriverController = new XboxController(OIConstants.kDriverControllerPort);

  final GamepadButtons driver = new GamepadButtons(m_DriverController, true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preferences.removeAll();
    Pref.deleteUnused();
    Pref.addMissing();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    // Configure the button bindings

    m_fieldSim.initSim();
    initializeAutoChooser();
    // sc.showAll();
    // Configure default commands
  // m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // new SetSwerveDrive(
         //m_robotDrive,

        // () -> -m_coDriverController.getRawAxis(1),
         //() -> -m_coDriverController.getRawAxis(0),
         //() -> -m_coDriverController.getRawAxis(4)));
        m_robotDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_robotDrive,
            () -> -leftJoystick.getRawAxis(1),
            () -> -leftJoystick.getRawAxis(0),
            () -> -rightJoystick.getRawAxis(4)));


      //Controller Buttons
        //Driver 
        // JoystickButton d_AButton = new JoystickButton(d_leftJoystick, 1);
        // JoystickButton d_BButton = new JoystickButton(d_leftJoystick, 2);
        // JoystickButton d_XButton = new JoystickButton(d_leftJoystick,3);
        // JoystickButton d_YButton = new JoystickButton(d_leftJoystick, 4);
        // JoystickButton d_RBumper = new JoystickButton(d_leftJoystick, 5);
        // JoystickButton d_LBumper = new JoystickButton(d_leftJoystick, 6);
        // JoystickButton d_TFrames = new JoystickButton(d_leftJoystick, 7);
        JoystickButton TLines = new JoystickButton(leftJoystick,8);

        // //Gunner
        // JoystickButton g_AButton = new JoystickButton(g_leftJoystick, 1);
        // JoystickButton g_BButton = new JoystickButton(g_leftJoystick, 2);
        // JoystickButton g_XButton = new JoystickButton(g_leftJoystick, 3);
        // JoystickButton g_YButton = new JoystickButton(g_leftJoystick, 4);
        // JoystickButton g_RBumper = new JoystickButton(g_leftJoystick, 5);
        // JoystickButton g_LBumper = new JoystickButton(g_leftJoystick, 6);
        // JoystickButton g_TFrames = new JoystickButton(g_leftJoystick, 7);
        // JoystickButton g_TLines = new JoystickButton(g_leftJoystick, 8);

         

      // Driver Button Commands
        TLines.onTrue(new ToggleFieldOriented(m_robotDrive));

      // Gunner Button Commands
        
       

  }

  public void initialize(){
    turret.initalize();
  }

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
   
    SmartDashboard.putData("Auto Selector", m_autoChooser);

  }

  public void simulationPeriodic() {
    m_fieldSim.periodic();
    periodic();
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
