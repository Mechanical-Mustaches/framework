package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorIntake extends SubsystemBase{
   //Instance Variables
    private String name;
    private Double speed;
    private CANSparkMax motor = new CANSparkMax(95, MotorType.kBrushless);
    
    //Constructor
    public FloorIntake(String name, Double speed){
        this.name = name;
        this.speed = speed;
    }

    //Starting Command
    public void initalize(){
        System.out.println(name + "is booting up");
    }

    //Movement
    public void Forward(){
        motor.set(speed);
    }

    public void Reverse(){
        motor.set(-speed);
    }

    public void Off(){
        motor.set(0);
    }


}
