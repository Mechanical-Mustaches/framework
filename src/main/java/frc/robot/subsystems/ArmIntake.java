package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmIntake extends SubsystemBase{
    private String name;
    private CANSparkMax motor = new CANSparkMax(97, MotorType.kBrushless);
    private double speed;
    RelativeEncoder encoder;
     
    //Constructor
    public ArmIntake(String name, Double speed){
        this.name = name;
        this.speed = speed;
    }

    //Starting Command
    public void initialize(){
        System.out.println(name + "is booting up");
    }   

    //Movement
    public void Forward(){
        motor.set(speed);
    }

    public void Reverse(){
        motor.set(-speed);
    }

    public void Stop(){
        motor.set(0);
    }

}
