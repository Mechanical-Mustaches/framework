package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm {
    private String name;
    private CANSparkMax BemoIntake = new CANSparkMax(3, MotorType.kBrushless);

    public Arm(String name){ 
        this.name = name;
    }

    public void Consume(){
        //Action = BemoIntake
        //Speed = 50%
        BemoIntake.set(0.5);
    }

    //Get --> Get the value of the object (being produced)
    //Set --> Set a value to a object (What we want to be produced)

    
} // <-- Keep Bracket 
