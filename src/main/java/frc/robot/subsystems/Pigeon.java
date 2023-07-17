package frc.robot.subsystems;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.BasePigeon;

/*  
 * https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_pigeon2.html
 * this link is good to look at the Pigeon data base
 * and figure out the functions that go along with it :)
 */
 

public abstract class Pigeon extends SubsystemBase {
    private Pigeon2 bird = new Pigeon2(0); //See where it it plugged in on robot
    private String name;
    private BasePigeon m_basePigeon;

    Pigeon2Configuration config = new Pigeon2Configuration();

    Pigeon2_Faults faults = new Pigeon2_Faults();

    // set mount pose as rolled 90 deg counter-clockwise
    WPI_PigeonIMU gryo = new WPI_PigeonIMU(0);


    public Pigeon(BasePigeon m_basePigeon){
        
    }

    // This is not needed to use since the device is flat on the robot
    public void configurePigeon(){
        config.MountPoseYaw = 0;
        config.MountPosePitch = 0;
        config.MountPoseRoll = 0;
        bird.configAllSettings(config);

    }

    public String getYaw(){
        return "Yaw: " + bird.getYaw();
    }

    public String getPitch(){
        return "Pitch: " + bird.getPitch();
    }

    public String getRoll(){
        return "Roll: " + bird.getRoll();
    }

    public void gravity(){
        double gravityVec[] = new double[3];
        //This gets the gravity vector of the pigeon
        bird.getGravityVector(gravityVec);
    }

    public void ohNo_Faults(){
        //returns the last error generated from bird
        ErrorCode faultsError = bird.getFaults(faults);
    }

}
