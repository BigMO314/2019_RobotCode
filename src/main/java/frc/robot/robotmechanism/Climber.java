package frc.robot.robotmechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.configuration.ConfigRobot;
import frc.robot.configuration.IntakeAngle;


public class Climber {

    private TalonSRX mtr_Front_Climb;
    private TalonSRX mtr_Back_Climb;
    private TalonSRX mtr_Drive_Climb;

    private Timer timeOut;

    private double frontTarget = 0.0;
    private double backTarget = 0.0;
    private double driveTarget = 0.0;
    private double tolerance = 1.0;
    private double targetTime = 0.2;

    private Timer tmrFrontPID;
    private Timer tmrBackPID;
    private Timer tmrDrivePID;

    private boolean isLevel2 = false;
    private boolean isLevel3 = false;
    private boolean override = false;
    private int climbStage = 0;

    private boolean isDrivingForward = false;

    public Climber(TalonSRX mtr_Front_Climb, TalonSRX mtr_Back_Climb, TalonSRX mtr_Drive_Climb){
        this.mtr_Front_Climb = mtr_Front_Climb;
        this.mtr_Back_Climb = mtr_Back_Climb;
        this.mtr_Drive_Climb = mtr_Drive_Climb;

        timeOut = new Timer();
        timeOut.reset();
        timeOut.start();
        tmrFrontPID = new Timer();
        tmrFrontPID.reset();
        tmrFrontPID.start();
        tmrBackPID = new Timer();
        tmrBackPID.reset();
        tmrBackPID.start();
        tmrDrivePID = new Timer();
        tmrDrivePID.reset();
        tmrDrivePID.start();

        mtr_Front_Climb.configForwardSoftLimitEnable(false);
        mtr_Front_Climb.configReverseSoftLimitEnable(false);
        mtr_Back_Climb.configForwardSoftLimitEnable(false);
        mtr_Back_Climb.configReverseSoftLimitEnable(false);

        mtr_Front_Climb.configFeedbackNotContinuous(false, 0);
        configureHeightPID(1.0, 0.00001, 0.01);
        configureDrivePID(1.0, 0.0, 0.01);
    }

    public void configureHeightPID(double p, double i, double d) {
        mtr_Front_Climb.config_kP(0, p, 0);
        mtr_Front_Climb.config_kI(0, i, 0);
		mtr_Front_Climb.config_kD(0, d, 0);
		
		//TODO: Why did we only set front pid?
    }

    public void configureDrivePID(double p, double i, double d) {
        mtr_Drive_Climb.config_kP(0, p, 0);
        mtr_Drive_Climb.config_kI(0, i, 0);
        mtr_Drive_Climb.config_kD(0, d, 0);
    }

    public void configurePeakOutputDrive(double forward, double reverse) {
        mtr_Drive_Climb.configPeakOutputForward(forward);
        mtr_Drive_Climb.configPeakOutputReverse(reverse);
    }

    public void configurePeakOutputClimb(double forward, double reverse) {
        mtr_Front_Climb.configPeakOutputForward(forward);
        mtr_Front_Climb.configPeakOutputReverse(reverse);
        mtr_Back_Climb.configPeakOutputForward(forward);
        mtr_Back_Climb.configPeakOutputReverse(reverse);
    }

    public void resetFrontHeight() { mtr_Front_Climb.setSelectedSensorPosition(0, 0, 0); }
    public double getFrontHeight() { return mtr_Front_Climb.getSelectedSensorPosition(0) * ConfigRobot.inchesPerCount; }
    public void resetBackHeight() { mtr_Back_Climb.setSelectedSensorPosition(0, 0, 0); }
    public double getBackHeight() { return mtr_Back_Climb.getSelectedSensorPosition(0) * ConfigRobot.inchesPerCount; }
    public void resetDistance() { mtr_Drive_Climb.setSelectedSensorPosition(0, 0, 0); }
    public double getDistance() { return mtr_Drive_Climb.getSelectedSensorPosition(0) * ConfigRobot.inchesPerCount_Climber_Drive; }

    public boolean isAtFrontHeight() {
        return tmrFrontPID.get() >= targetTime;
    }

    public boolean isAtBackHeight() {
        return tmrBackPID.get() >= targetTime;
    }

    public boolean isAtDistance() {
        return tmrDrivePID.get() >= targetTime;
    }

    public void goToFrontHeight(double height) {
        frontTarget = height;
        height /= ConfigRobot.inchesPerCount;
        mtr_Front_Climb.set(ControlMode.Position, height);
        tmrFrontPID.reset();
    }

    public void goToBackHeight(double height) {
        backTarget = height;
        height /= ConfigRobot.inchesPerCount;
        mtr_Back_Climb.set(ControlMode.Position, height);
        tmrFrontPID.reset();
    }

    public void goToDistance(double distance) {
        driveTarget = distance;
        distance /= ConfigRobot.inchesPerCount_Climber_Drive;
        mtr_Drive_Climb.set(ControlMode.Position, distance);
        tmrDrivePID.reset();
    }

    public void setFrontClimber(double power) {
        mtr_Front_Climb.set(ControlMode.PercentOutput, power);
    }

    public void setBackClimber(double power) {
        mtr_Back_Climb.set(ControlMode.PercentOutput, power);
    }

    public void setDrive(double power) {
        mtr_Drive_Climb.set(ControlMode.PercentOutput, power);
    }

    public int getStage() { return climbStage; }

    public void goToLevel2() { isLevel2 = true; isLevel3 = false; }
    public void goToLevel3() { isLevel2 = false; isLevel3 = true; }
    public void dontClimb() { isLevel2 = false; isLevel3 = false; override = false; }
    public void overrideClimber() { override = true; }
    public void dontOverride() { override = false; }

    public boolean isClimbing() { if (isLevel2 == true || isLevel3 == true) return true; else return false; }

    public double getFrontTimer() { return tmrFrontPID.get(); }

    public boolean isDrivingDrivetrain() { return isDrivingForward; }
    
    public void update() {
        if (isLevel2) {
            switch (climbStage) {
                case 0:
                //lift up
                System.out.println("Yes");
                goToFrontHeight(6.83);
                goToBackHeight(9.0);
                climbStage++;
                break;

                case 1:
                if (isAtFrontHeight() == true) { climbStage++; }
                break;

                case 2:
                //drive to first elevator
                resetDistance();
                goToDistance(10.0);
                climbStage++;
                break;

                case 3:
                if (isAtDistance() == true) { climbStage++; }
                break;

                case 4:
                //lift up front elevator
                goToFrontHeight(0.0);
                climbStage++;
                break;

                case 5:
                if (isAtFrontHeight()) { climbStage++; }
                break;

                case 6:
                //drive to back elevator
                resetDistance();
                goToDistance(13.0);
                climbStage++;
                break;

                case 7:
                if (isAtDistance()) { climbStage++; }
                break;

                case 8:
                //lift back elevator
                goToBackHeight(0.0);
                climbStage++;
                break;

                case 9:
                if (isAtBackHeight()) { climbStage++; }
                break;
                //drive onto platform
                case 10:
                timeOut.reset();
                timeOut.start();
                isDrivingForward = true;
                climbStage++;
                break;

                case 11:
                if (timeOut.get() > 0.25) {
                    isDrivingForward = false;
                    dontClimb();
                }
                break;
            }
        }
        else if (isLevel2 == false && isLevel3 == true) {
            switch (climbStage) {
                case 0:
                //lift up
                System.out.println("Yes");
                goToFrontHeight(15.0);
                goToBackHeight(21.0);
                climbStage++;
                break;

                case 1:
                if (isAtFrontHeight() == true && isAtBackHeight()) { climbStage++; }
                break;

                case 2:
                //drive to first elevator
                resetDistance();
                goToDistance(18.0);
                climbStage++;
                break;

                case 3:
                if (isAtDistance() == true) { climbStage++; }
                break;

                case 4:
                //lift up front elevator
                goToFrontHeight(0.5);
                climbStage++;
                break;

                case 5:
                if (isAtFrontHeight()) { climbStage++; }
                break;

                case 6:
                //drive to back elevator
                resetDistance();
                goToDistance(13.0);
                climbStage++;
                break;

                case 7:
                if (isAtDistance()) { climbStage++; }
                break;

                case 8:
                //lift back elevator
                goToBackHeight(0.0);
                climbStage++;
                break;

                case 9:
                if (isAtBackHeight()) { climbStage++; }
                break;
                //drive onto platform
                case 10:
                timeOut.reset();
                timeOut.start();
                climbStage++;
                break;

                case 11:
                if (isAtDistance()) {
                    climbStage = 0;
                    dontClimb();
                }
                break;
            }
        }
        else if (isLevel2 == false && isLevel3 == false && override == false) {
            goToFrontHeight(0.0);
            goToBackHeight(0.0);
            climbStage = 0;
        }
        if (Math.abs((mtr_Front_Climb.getSelectedSensorPosition(0) * ConfigRobot.inchesPerCount) - frontTarget) > tolerance) { tmrFrontPID.reset(); tmrFrontPID.start(); }
        if (Math.abs((mtr_Back_Climb.getSelectedSensorPosition(0) * ConfigRobot.inchesPerCount) - backTarget) > tolerance) { tmrBackPID.reset(); tmrBackPID.start(); }
        if (Math.abs((mtr_Drive_Climb.getSelectedSensorPosition(0) * ConfigRobot.inchesPerCount_Climber_Drive) - driveTarget) > tolerance) { tmrDrivePID.reset(); tmrDrivePID.start(); }

    }
}