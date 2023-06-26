package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private String name;
    private CANSparkMax motor = new CANSparkMax(96, MotorType.kBrushless);
   
    //Constructor
    public Turret(String name){
        this.name = name;
    }

    //Starting Command
    public void initalize(){
        System.out.println(name + "is booting up");
    }

    //Movement
    
}

