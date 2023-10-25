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
import frc.robot.commands.ConnersBrain.RollUp;
import frc.robot.commands.ConnersBrain.Stop;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.commands.Flywheel.*;


//Import Commands here

//import frc.robot.commands.TankDrive.*;


//Import Subsystems here

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
  

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /*
   * This is where agent names and abilities are stored
   * You can add different speeds and autonomous routines into each
   * section. 
   */
  
  
  
  // Driver & Gunner controllers
  static Joystick leftJoystick = new Joystick(OIConstants.kDriverControllerPort);
  static Joystick rightJoystick = new Joystick(OIConstants.kDriverControllerPort);

  private XboxController m_DriverController = new XboxController(OIConstants.kDriverControllerPort);
  private XboxController m_coDriverController = new XboxController(OIConstants.kCoDriverControllerPort);

  Conveyor conveyor = new Conveyor("conner", 0.2);
  Flywheel flywheel = new Flywheel("g",0.7);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preferences.removeAll();
   
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    // Configure the button bindings

   
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
       
           


      //Controller Buttons
        //Driver 
        JoystickButton d_AButton = new JoystickButton(leftJoystick, 1);
        JoystickButton d_BButton = new JoystickButton(leftJoystick, 2);
        JoystickButton d_XButton = new JoystickButton(leftJoystick,3);
        JoystickButton d_YButton = new JoystickButton(leftJoystick, 4);
        JoystickButton d_RBumper = new JoystickButton(leftJoystick, 5);
        JoystickButton d_LBumper = new JoystickButton(leftJoystick, 6);
        JoystickButton d_TFrames = new JoystickButton(leftJoystick, 7);
        JoystickButton d_TLines = new JoystickButton(leftJoystick,8);
   
        //Gunner
        JoystickButton g_AButton = new JoystickButton(leftJoystick, 1);
        JoystickButton g_BButton = new JoystickButton(leftJoystick, 2);
        JoystickButton g_XButton = new JoystickButton(leftJoystick, 3);
        JoystickButton g_YButton = new JoystickButton(leftJoystick, 4);
        JoystickButton g_RBumper = new JoystickButton(leftJoystick, 5);
        JoystickButton g_LBumper = new JoystickButton(leftJoystick, 6);
        JoystickButton g_TFrames = new JoystickButton(leftJoystick, 7);
        JoystickButton g_TLines = new JoystickButton(leftJoystick, 8);

      d_AButton.onTrue(new RollUp(conveyor));
      d_AButton.onFalse(new Stop(conveyor));

      d_XButton.onTrue(new shooterCommand(flywheel));
      d_XButton.onFalse(new stop(flywheel));

      // Driver Button Commands
       

      // Gunner Button Commands
        
       

  }

  public void initialize(){
    
  }

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
   
    SmartDashboard.putData("Auto Selector", m_autoChooser);

  }

  public void simulationPeriodic() {
   
  }

  
   

  public double getThrottle() {
    return -leftJoystick.getThrottle();
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

}
