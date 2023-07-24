// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
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

//Import Commands here

import frc.robot.commands.Shooter.*;

//import frc.robot.commands.TankDrive.*;
import frc.robot.commands.Turret.MoveLeft;
import frc.robot.commands.Turret.MoveRight;
import frc.robot.commands.Turret.Stop;
//Import Subsystems here

import frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Turret;

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
 
  Shooter shooter = new Shooter("sunny", 0.4);
  Turret turret = new Turret("todd"); 
  
  
  // Driver & Gunner controllers
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
    
  
    initializeAutoChooser();
  
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

         

      // Driver Button Commands
        d_XButton.onTrue(new MoveLeft(turret));
        d_XButton.onFalse(new Stop(turret));
        d_BButton.onTrue(new MoveRight(turret));
        d_BButton.onFalse(new Stop(turret));



      // Gunner Button Commands
        
       

  }

 

  public void initialize(){
    turret.initalize();
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
