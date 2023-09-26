package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Conveyor:
 * Motor: 10
 * Functions:
 * Roll up 
 * Reverse
 */

public class Conveyor extends SubsystemBase{
    //Private: ONLY THE CLASS IT
    //Public: EVERYONE CAN SEE IT
    private String name; 
    private CANSparkMax fire = new CANSparkMax(10, MotorType.kBrushless);
    
    //Double: decimal --> .0 .001 .001
    //Int: whole number -->1, 34, 420
    //Boolean: True or False
    //Double, Ints, and Boolean ALL RETURN SOMETHING
    private double fast;

    //Voids: DON'T RETURN ANYTHING --> setting variables to things

    public Conveyor(String name, double fast){
        this.name = name;
        this.fast = fast;
    }

    public void rollUp(){
        fire.set(fast);
    }

    public void rev(){
        fire.set(-fast);
    }
    
    public void Stop(){
        fire.set(0);
    }
}
