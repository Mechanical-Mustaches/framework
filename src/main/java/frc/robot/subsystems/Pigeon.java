package frc.robot.subsystems;
//import com.ctre.phoenix.sensors.Pigeon2;
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
    private WPI_Pigeon2 bird = new WPI_Pigeon2(30); //See where it it plugged in on robot
    private String name;
    private BasePigeon m_basePigeon;

    Pigeon2Configuration config = new Pigeon2Configuration();

    Pigeon2_Faults faults = new Pigeon2_Faults();

    // set mount pose as rolled 90 deg counter-clockwise
    //WPI_PigeonIMU gryo = new WPI_PigeonIMU(0);


    public Pigeon(BasePigeon m_basePigeon){
        this.m_basePigeon = m_basePigeon;
    }

    public double bird_getYaw() {return m_basePigeon.getYaw();}
    public double bird_getPitch() {return m_basePigeon.getPitch();}
    public double bird_getRoll() {return m_basePigeon.getRoll();}

    public void setYaw(double yaw) {m_basePigeon.setYaw(yaw, 10);}
    public void addYaw(double yaw) {m_basePigeon.addYaw(yaw, 10);}
    public void setYawToCompass() {m_basePigeon.setYawToCompass(10);}
    public void setAccumZ(double accumZ) {m_basePigeon.setAccumZAngle(accumZ, 10);}
    public abstract boolean getFaults();

    public double getCompass() { return m_basePigeon.getCompassHeading();}
    public double getAccumZ(){
        double[] accums = new double[3];
        m_basePigeon.getAccumGyro(accums);
        return accums[2];
    }

    public double[] getRawGyros(){
        double[] gyrs = new double[3];
        m_basePigeon.getRawGyro(gyrs);
        return gyrs;
    }

    public int getUpTime() { return m_basePigeon.getUpTime();}
    public double getTemp() {return m_basePigeon.getTemp();}


    // This is not needed to use since the device is flat on the robot
    public void configurePigeon(){
        config.MountPoseYaw = 0;
        config.MountPosePitch = 0;
        config.MountPoseRoll = 0;
    //    bird.configAllSettings(config);

    }

    public String getYaw(){
       return "Yaw: " + bird.getYaw();
    }

    public String getPitch(){
        return "Pitch: "+ bird.getPitch();
    }

    public String getRoll(){
        return "Roll: " + bird.getRoll();
    }

    public void gravity(){
        double gravityVec[] = new double[3];
        //This gets the gravity vector of the pigeon
      //  bird.getGravityVector(gravityVec);
    }

    public void ohNo_Faults(){
        //returns the last error generated from bird
     //   ErrorCode faultsError = bird.getFaults(faults);
    }

}
