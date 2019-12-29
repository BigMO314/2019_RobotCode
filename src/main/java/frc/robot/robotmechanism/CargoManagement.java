package frc.robot.robotmechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.configuration.ConfigRobot;
import frc.robot.configuration.IntakeAngle;
import frc.robot.pidloops.PIDLoop;
import java.lang.Math;

public class CargoManagement {
    private TalonSRX mtr_IntakeRotate;
    private VictorSPX mtr_Intake;

    private double m_Power = 0.0;
    private double m_RotationPower = 0.0;

    private boolean m_IsResetting = false;
    private boolean m_GoingToZero = false;

    private double target = 0.0;
    private double tolerance = 1.0;
    private double targetTime = 0.2;

    private Timer tmrAnglePID;

    private DigitalInput pho_Uptake;
    private DigitalInput lim_Intake;
    private boolean isObject = false;

    public CargoManagement(TalonSRX mtr_IntakeRotate, VictorSPX mtr_Intake, DigitalInput pho_Uptake, DigitalInput lim_Intake) {
        this.mtr_IntakeRotate = mtr_IntakeRotate;
        this.mtr_Intake = mtr_Intake;
        this.pho_Uptake = pho_Uptake;
        this.tmrAnglePID = new Timer();
        this.tmrAnglePID.reset();
        this.tmrAnglePID.start();
        this.lim_Intake = lim_Intake;
        configureAnglePid(1.5, 0.0, 0.003);
        mtr_IntakeRotate.configForwardSoftLimitThreshold((int)(0.0 / ConfigRobot.degreesPerCount));
        mtr_IntakeRotate.configReverseSoftLimitThreshold((int)(-120.0 / ConfigRobot.degreesPerCount));
        mtr_IntakeRotate.configForwardSoftLimitEnable(true);
        mtr_IntakeRotate.configReverseSoftLimitEnable(true);

        mtr_IntakeRotate.configFeedbackNotContinuous(false, 0);
    }

    public void configureAnglePid(double p, double i, double d) {
        mtr_IntakeRotate.config_kP(0, p, 0);
        mtr_IntakeRotate.config_kI(0, i, 0);
        mtr_IntakeRotate.config_kD(0, d, 0);

    }

    public void resetAngle() { mtr_IntakeRotate.setSelectedSensorPosition(0, 0, 0); }
    public double getAngle() { return mtr_IntakeRotate.getSelectedSensorPosition(0); }

    public void configPeakOutput(double forward, double reverse) {
        mtr_IntakeRotate.configPeakOutputForward(forward);
        mtr_IntakeRotate.configPeakOutputReverse(reverse);
    }

    public boolean isAtAngle() {
        return tmrAnglePID.get() >= targetTime;
    }

    public boolean isPIDEnabled() { return mtr_IntakeRotate.getControlMode() == ControlMode.Position; }

    public void goToAngle(double angle) {
        target = angle;
        angle /= ConfigRobot.degreesPerCount;
        mtr_IntakeRotate.set(ControlMode.Position, -angle);
        tmrAnglePID.reset();
    }

    public void Point(IntakeAngle pos) {
        switch(pos) {
            case kUp: goToAngle(0.0); break;
            case kDown: goToAngle(110.0); break;
        }
    }

    public void setRotationPower(double rotationPower) {
        mtr_IntakeRotate.set(ControlMode.PercentOutput, rotationPower);
    }

    public void startResetting() {
        m_IsResetting = true;
    }

    public void stopResetting() {
        m_IsResetting = false;
    }

    public void setGoingToZero(boolean input) {
        m_GoingToZero = input;
    }

    public void setIntakePower(double power) {
        m_Power = power;
    }

    public boolean isObjectDetected() {
        return isObject;
    }

    public void update() {
        mtr_Intake.set(ControlMode.PercentOutput, m_Power);
        isObject = pho_Uptake.get();
        if (Math.abs((mtr_IntakeRotate.getSelectedSensorPosition(0) * ConfigRobot.degreesPerCount) - target) > tolerance) { tmrAnglePID.reset(); tmrAnglePID.start(); }
        // if (m_GoingToZero == true) {
        //     if (mtr_IntakeRotate.getSelectedSensorPosition() >= -1.0 || mtr_IntakeRotate.getSelectedSensorPosition() <= 1.0) {
        //         if (lim_Intake.get() == false) {
        //             m_IsResetting = true;
        //         }
        //     }
        // }
        if (m_IsResetting) {
            setRotationPower(0.5);
            m_GoingToZero = true;
            if (lim_Intake.get()) {
                resetAngle();
                Point(IntakeAngle.kUp);
                m_IsResetting = false;
            }
        }
    }
}