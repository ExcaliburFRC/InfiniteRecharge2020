package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Limelight extends SubsystemBase {

     /*
        Values:
            Getters:
                tv - Whether the limelight has any valid targets (0 or 1)
                tx - Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
                ty - Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
                ta - Target Area (0% of image to 100% of image)
                tshort - Sidelength of shortest side of the fitted bounding box (pixels)
                tlong - Sidelength of longest side of the fitted bounding box (pixels)
                thor - Horizontal sidelength of the rough bounding box (0 - 320 pixels)
                tvert - Vertical sidelength of the rough bounding box (0 - 320 pixels)
            Setters:
                LED mode : 1 (off), 2 (blink), 3 (on)
                Cam Mode : 0 Vision, 1 Driving
    */

    int tv;
    double tx;
    double ty;
    double ta;
    double tshort;
    double tlong;
    double thor;
    double tvert;
    double camMode, ledMode, pipeline;

    DoubleSolenoid hightSolenoid;

    public static enum CamModes{
        VISION(0), DRIVING(1);

        public final int value;

        private CamModes(int val){
            this.value = val;
        }
    } 

    public static enum LedModes{
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        public final int value;

        private LedModes(int val){
            this.value = val;
        }
    } 


    public Limelight(){
        hightSolenoid = new DoubleSolenoid(RobotMap.LIMELIGHT_SOLENOID_PORTS[0], RobotMap.LIMELIGHT_SOLENOID_PORTS[1]);
    }

    public double getVar(String var){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(var).getDouble(0);
    }

    @Override
    public void periodic(){
        //need to think about only activating some times so that it isnt too CPU intensive
        // this.tv = (int) (this.getVar("tv"));
        // this.tx = this.getVar("tx");
        // this.ty = this.getVar("ty");
        // this.ta = this.getVar("ta");
        // this.tshort = this.getVar("tshort");
        // this.tlong = this.getVar("tlong");
        // this.thor = this.getVar("thor");
        // this.tvert = this.getVar("tvert");
    }
    
    public void setLEDMode(int mode){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
    }

    public void setLEDMode(LedModes mode){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode.value);
    }

    public void setCamMode(int mode){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(mode);
    }

    public void setCamMode(CamModes mode){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(mode.value);
    }

    public void setPipeline(int pipenum){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipenum);
    }

    public int getTv() {
        return (int) getVar("tv");
    }

    public boolean isDetectingTarget(){
        return getTv() == 1;
    }

    public double getTx() {
        return getVar("tx");
    }

    public double getTy() {
        return getVar("ty");
    }

    public double getTa() {
        return ta;
    }

    public double getTshort() {
        return tshort;
    }

    public double getTlong() {
        return tlong;
    }


    public double getThor() {
        return thor;
    }

    public double getTvert() {
        return tvert;
    }

    public double getCamMode() {
        this.camMode = this.getVar("camMode");
        return camMode;
    }

    public double getLEDMode() {
        this.ledMode = this.getVar("ledMode");
        return ledMode;
    }

    public double getPipeline(){
        this.pipeline = this.getVar("pipeline");
        return pipeline;
    }

    public void setLifterState(boolean on){
        Value value = on ? Value.kForward : Value.kReverse;
        hightSolenoid.set(value);
    }

    public boolean getLifterPistonPosition(){
        return hightSolenoid.get() == Value.kForward ? true : false;
      }
}