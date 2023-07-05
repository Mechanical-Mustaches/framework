package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase{
    //Instance Variables
    private String name;
    private double max_speed;
    private double max_turn = .7;
    private CANSparkMax m_FrontRight = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax m_BackRight = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax m_FrontLeft = new CANSparkMax(99, MotorType.kBrushless);
    private CANSparkMax m_BackLeft = new CANSparkMax(99, MotorType.kBrushless);

    private MotorControllerGroup rodger = new MotorControllerGroup(m_FrontRight, m_BackRight);
    private MotorControllerGroup louie = new MotorControllerGroup(m_FrontLeft, m_BackLeft);

    private DifferentialDrive drivechain = new DifferentialDrive(louie, rodger);

    //Constructor
    public TankDrive(String name, Double max_speed){
        this.name = name;
        this.max_speed = max_speed;
    }

    //Starting Command
    public void initalize(){
        System.out.println(name + " is coming in hot");
    }

    //Movement
    public void Forward(){

    }






}