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
// import frc.robot.commands.auto.DriveForward;
// import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.swerve.JogDriveModule;
import frc.robot.commands.swerve.JogTurnModule;
import frc.robot.commands.swerve.PositionTurnModule;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.ToggleFieldOriented;
import frc.robot.simulation.FieldSim;
//Import Commands here
import frc.robot.subsystems.DriveSubsystem;

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

 // public final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /*
   * This is where agent names and abilities are stored
   * You can add different speeds and autonomous routines into each
   * section. 
   */
 
  
  // The driver's controller

  static Joystick leftJoystick = new Joystick(OIConstants.kDriverControllerPort);
  static Joystick rightJoystick = new Joystick(OIConstants.kDriverControllerPort);

  private XboxController m_DriverController = new XboxController(OIConstants.kDriverControllerPort);
  private XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  final GamepadButtons driver = new GamepadButtons(m_DriverController, true);
  final GamepadButtons gunner = new GamepadButtons(m_coDriverController, true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preferences.removeAll();
    Pref.deleteUnused();
    Pref.addMissing();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    // Configure the button bindings

 //   m_fieldSim.initSim();
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
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> rightJoystick.getRawAxis(4)));


      //Controller Buttons
    

        JoystickButton button_8 = new JoystickButton(leftJoystick,8);
        JoystickButton button_7 = new JoystickButton(leftJoystick, 7);
        JoystickButton X_button = new JoystickButton(leftJoystick, 4);       

      // Driver Button Commands
      //  button_8.onTrue(new ToggleFieldOriented(m_robotDrive));

      // Gunner Button Commands
        
       

  }

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
   
    SmartDashboard.putData("Auto Selector", m_autoChooser);

  }

  // public void simulationPeriodic() {
  //   m_fieldSim.periodic();
  //   periodic();
  // }

  // public void periodic() {
  //   m_fieldSim.periodic();
  // }

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
