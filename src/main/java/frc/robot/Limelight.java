package frc.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight implements PIDSource {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    VisionMode visMode = VisionMode.kVisionLEDOn;
    boolean usePresetMode = false;
    private double x;
    private double y;
    private double area;

    public Limelight () {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        usePresetMode = true;
    }
    public Limelight (int pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
        usePresetMode = false;
    }
    public Limelight (int ledState, int camMode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledState);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        usePresetMode = false;
    }
    public Limelight (int ledState, int camMode, int pipeline) {
        
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledState);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
        usePresetMode = false;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {  }

    @Override
    public PIDSourceType getPIDSourceType() { return null; }

    @Override
    public double pidGet() { return getX(); }

    //read values periodically
    public double getX() { return this.x; }
    public double getY() { return this.y; }
    public double getArea() { return this.area; }
    public void ledState(int state) { NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(state); }
    public void cameraState(int state) { NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(state); }
    public void setPipeline(int pipeline) { NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); }
    public int getLedState() { return (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getDouble(0.0); }
    public int getCameraState() { return (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").getDouble(0.0); }
    public int getPipeline() { return (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0.0); }
    public void changeVisionState(VisionMode newMode) { 
        usePresetMode = true;
        visMode = newMode;
    }
    public void update() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        if (usePresetMode) {
            switch(visMode) {
                case kVisionLEDOn :
                    if (getLedState() != 3) {
                        ledState(3);
                    }
                    if (getCameraState() != 0) {
                        cameraState(0);
                    }
                break;
                case kVisionLEDOff :
                    if (getLedState() != 1) {
                        ledState(1);
                    }
                    if (getCameraState() != 0) {
                        cameraState(0);
                    }
                break;
                case kDriverCam :
                    if (getLedState() != 1) {
                        ledState(1);
                    }
                    if (getCameraState() != 1) {
                        cameraState(1);
                    }
                break;
            }
        }
    }
}