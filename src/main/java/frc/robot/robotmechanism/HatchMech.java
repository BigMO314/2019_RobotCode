package frc.robot.robotmechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.configuration.ConfigRobot;
import frc.robot.configuration.HatchPosition;
import frc.robot.pidloops.EncoderLoop;
import frc.robot.pidloops.PIDLoop;

public class HatchMech {
    private VictorSPX mtr_Hatch;

    private double m_Power;
    
    private Encoder enc_Angle;
    private EncoderLoop pid_Angle;

    private DigitalInput lim_Hatch;

    public HatchMech(VictorSPX mtr_Hatch, Encoder enc_Angle, DigitalInput lim_Hatch) {
        this.mtr_Hatch = mtr_Hatch;
        this.enc_Angle = enc_Angle;
        this.lim_Hatch = lim_Hatch;
        pid_Angle = new EncoderLoop(0.3, 0.1, 0.04, enc_Angle);
    }

    public void ConfigureAnglePID(double P, double I, double D) { pid_Angle.setPID(P, I, D); }
	public void ConfigAngleOutputRange(double min, double max) { pid_Angle.setOutputRange(min, max); }
	public void goToAngle(double angle) {
		//pid_Angle.setPID(0.024, 0.0, 0.0245);
		pid_Angle.setSetpoint(angle * ConfigRobot.countsPerDegree);
		pid_Angle.setAbsoluteTolerance(0.5);
        pid_Angle.enable();
        //angle /= ConfigRobot.degreesPerCount_Hatch;
    }

	public boolean isAnglePIDEnabled() { return pid_Angle.isEnabled(); }
	public boolean isAtAngle() { return pid_Angle.onTarget(); }
	public double getAngle() { return enc_Angle.get(); }
    public void disableAnglePID() { pid_Angle.disable(); }
    public double getP() { return pid_Angle.getP(); }
    public void resetAngle() { pid_Angle.resetSource(); }


    public void setHatchPower(double power) {
        m_Power = power;
    }

    public void point(HatchPosition pos) {
        switch(pos) {
            case kHold: goToAngle(65.0); break;
            case kClosed: goToAngle(76.0); break;
            case kCargo: goToAngle(60.0); break;
        }
    }

    public void goToZero() {
        if (lim_Hatch.get()) {
            m_Power = -1.0;
        }
        else {
            m_Power = 0.0;
        }
    }

    public void update() {
        if(isAnglePIDEnabled()){
            if(isAtAngle()) pid_Angle.disable();
            setHatchPower(pid_Angle.get());
        }
        mtr_Hatch.set(ControlMode.PercentOutput, m_Power);
    }
}