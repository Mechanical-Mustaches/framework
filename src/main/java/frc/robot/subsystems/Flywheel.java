package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    //Variables 
    private double fast;
    private String name;
    private CANSparkMax a = new CANSparkMax(11, MotorType.kBrushless);

    //Constructor 
    public Flywheel (String name, double fast){
        this.name = name;
        this.fast = fast;
    }

    public void Shoot(){
        a.set(fast);
    }

    public void REV(){
        a.set(-fast);
    }

        public void Stop(){
            a.set(0);
        }



    



    
}
