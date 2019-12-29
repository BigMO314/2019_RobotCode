package frc.robot.controlperiod;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Limelight;
import frc.robot.VisionMode;
import frc.robot.configuration.HatchPosition;
import frc.robot.configuration.IntakeAngle;
import frc.robot.robotmechanism.CargoManagement;
import frc.robot.robotmechanism.Climber;
import frc.robot.robotmechanism.Drivetrain;
import frc.robot.robotmechanism.HatchMech;

public class Teleoperated {
    private XboxController ctl_Driver;
    private XboxController ctl_Operator;
    private Drivetrain rbt_Drivetrain;
    private HatchMech rbt_HatchMech;
    private CargoManagement rbt_CargoManagement;
    private Climber rbt_Climber;
    private Limelight lml_Vision;

    public Teleoperated(XboxController ctl_Driver, XboxController ctl_Operator, Drivetrain rbt_Drivetrain, HatchMech rbt_HatchMech, CargoManagement rbt_CargoManagement, Climber rbt_Climber, Limelight lml_Vision) {
        this.ctl_Driver = ctl_Driver;
        this.ctl_Operator = ctl_Operator;
        this.rbt_Drivetrain = rbt_Drivetrain;
        this.rbt_HatchMech = rbt_HatchMech;
        this.rbt_CargoManagement = rbt_CargoManagement;
        this.rbt_Climber = rbt_Climber;
        this.lml_Vision = lml_Vision;
    }

    public void arcadeDrive(double t, double s) {
        rbt_Drivetrain.setDrive(t - s, t + s);
    }

    public void update() {
        //Drivetrain
    
        if (ctl_Driver.getTriggerAxis(Hand.kLeft) > 0.1) {
            lml_Vision.changeVisionState(VisionMode.kVisionLEDOff);
            arcadeDrive((ctl_Driver.getY(Hand.kLeft) * 0.35), (ctl_Driver.getX(Hand.kRight) * 0.35));
        }
        else if (ctl_Driver.getTriggerAxis(Hand.kRight) > 0.1) {
            lml_Vision.changeVisionState(VisionMode.kVisionLEDOff);
            arcadeDrive(ctl_Driver.getY(Hand.kLeft), ctl_Driver.getX(Hand.kRight) * 0.7);
        }
        else if (ctl_Driver.getBButton()) {
            lml_Vision.changeVisionState(VisionMode.kVisionLEDOn);
            rbt_Drivetrain.followTarget(ctl_Driver.getY(Hand.kLeft) * 0.5);
        }
        else {
            lml_Vision.changeVisionState(VisionMode.kVisionLEDOff);
            arcadeDrive((ctl_Driver.getY(Hand.kLeft) * 0.75), (ctl_Driver.getX(Hand.kRight) * 0.7));
        }
        if (rbt_Climber.isDrivingDrivetrain()) { rbt_Drivetrain.setDrive(-0.3, -0.3); }
        //Cargo
        if (ctl_Operator.getTriggerAxis(Hand.kRight) > 0.1 && !rbt_CargoManagement.isObjectDetected()) {
            rbt_CargoManagement.setIntakePower(0.8);
        }
        else if (ctl_Operator.getTriggerAxis(Hand.kLeft) > 0.1) {
            rbt_CargoManagement.setIntakePower(0.8);
        }
        else if (ctl_Operator.getXButton()) {
            rbt_CargoManagement.setIntakePower(-0.3);
        }
        else {
            rbt_CargoManagement.setIntakePower(0.0);
        }

        if (ctl_Operator.getBumperPressed(Hand.kRight)) {
            rbt_CargoManagement.stopResetting();
            rbt_CargoManagement.setGoingToZero(false);
            rbt_CargoManagement.Point(IntakeAngle.kDown);
        }
        else if (ctl_Operator.getBumperPressed(Hand.kLeft)) {
            rbt_CargoManagement.setGoingToZero(true);
            rbt_CargoManagement.Point(IntakeAngle.kUp);
        }
        else if (ctl_Operator.getYButton()){
            rbt_CargoManagement.setRotationPower(ctl_Operator.getY(Hand.kLeft));
        }
        else if (ctl_Operator.getStartButton()) {
            if (ctl_Operator.getBackButtonPressed()) {
                rbt_CargoManagement.startResetting();
            }
        }

        //Hatch
        if (ctl_Operator.getAButton()) {
            rbt_HatchMech.setHatchPower(1.0);
        }
        else if (ctl_Operator.getBButton()) {
            rbt_HatchMech.setHatchPower(-1.0);
        }
        else {
            rbt_HatchMech.setHatchPower(0.0);
        }

        //Climber
        if (ctl_Driver.getBackButton()) {
            if (ctl_Driver.getAButtonPressed()) {
                rbt_Climber.goToLevel2();
            }
            else if (ctl_Driver.getYButtonPressed()) {
                rbt_Climber.goToLevel3();
            }
        }
        else if (ctl_Driver.getXButtonPressed()) { rbt_Climber.dontClimb(); }
        if (ctl_Operator.getBumper(Hand.kLeft)) {
            rbt_Climber.overrideClimber();
            if (Math.abs(ctl_Operator.getY(Hand.kLeft)) > 0.1) {
                rbt_Climber.setFrontClimber(ctl_Operator.getY(Hand.kLeft));
            }
            else { rbt_Climber.setFrontClimber(0.0); }
            if (Math.abs(ctl_Operator.getY(Hand.kRight)) > 0.1) {
                rbt_Climber.setBackClimber(ctl_Operator.getY(Hand.kRight));
            }
            else { rbt_Climber.setBackClimber(0.0); }
            if (ctl_Driver.getBumper(Hand.kLeft)) { rbt_Climber.setDrive(1.0); }
            else { rbt_Climber.setDrive(0.0); }
        }
        else { rbt_Climber.dontOverride(); }
    }
}