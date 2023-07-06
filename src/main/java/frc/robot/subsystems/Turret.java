package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private String name;
    private CANSparkMax motor = new CANSparkMax(89, MotorType.kBrushless);
    RelativeEncoder encoder;
    private double speed = 0.4;

    //Constructor
    public Turret(String name){
        this.name = name;
    }

    //Starting Command
    public void initalize(){
        System.out.println(name + "is booting up");
        encoder.setPosition(0);
    }

    //Movement
    /*
     * this code is the one that I wrote for toddet
     */

     public void moveRight(){
        motor.set(speed);
     }

     public boolean isMovingRight(){
        if(encoder.getPosition() >= 20){
            return true;
        }
        return false;
     }

     /************************************/

     public void moveLeft(){
        motor.set(-speed);
     }

     public boolean isMovingLeft(){
        if(encoder.getPosition() <= -20){
            return true;
        }
        return false;
     }

    /************************************/

     public void hardStop(){
        motor.set(0);
     }

    //Might need a isStopped boolean class
    
    
    
}

