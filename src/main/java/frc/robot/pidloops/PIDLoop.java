package frc.robot.pidloops;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
	
class dummyOutput implements PIDOutput {
	@Override
	public void pidWrite(double output) { }
}

public class PIDLoop extends PIDController {
	private Timer tmrTarget = new Timer();
	private double targetTime = 0.0;

	public PIDLoop(double pValue, double iValue, double dValue, PIDSource source) { super(pValue, iValue, dValue, source, new dummyOutput()); }
	public PIDLoop(double pValue, double iValue, double dValue, double fValue, PIDSource source) { super(pValue, iValue, dValue, fValue, source, new dummyOutput()); }
	
	@Override
	public void enable() {
		super.enable();
		tmrTarget.reset();
		tmrTarget.start();
	}
	
	@Override
	public void disable() {
		super.disable();
		tmrTarget.stop();
	}
	
	@Override
	public boolean onTarget() { 
		return super.onTarget() && tmrTarget.get() > targetTime; 
	}
	
	@Override
	public double get() {
		if (isEnabled()) return super.get();
		else return 0.0;
	}
	
	@Override
	protected void calculate() {
		super.calculate();
		if (isEnabled() && !super.onTarget()) tmrTarget.reset();
	}
	
	public void resetSource() { }
	public void calibrateSource() { }
	
	public void setTargetTime(double time) { targetTime = time; }

	public void setP(double value) { setPID(value, getI(), getD(), getF()); }
	public void setI(double value) { setPID(getP(), value, getD(), getF()); }
	public void setD(double value) { setPID(getP(), getI(), value, getF()); }
	public void setF(double value) { setPID(getP(), getI(), getD(), value); }
}