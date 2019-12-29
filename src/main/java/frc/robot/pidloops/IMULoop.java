package frc.robot.pidloops;

import com.kauailabs.navx.frc.AHRS;

public class IMULoop extends PIDLoop {
    private final AHRS imu_Source;
    
	public IMULoop(AHRS source) { this(0.0, 0.0, 0.0, 0.0, source); }
	public IMULoop(double pValue, double iValue, double dValue, AHRS source) { this(pValue, iValue, dValue, 0.0, source); }
	public IMULoop(double pValue, double iValue, double dValue, double fValue, AHRS source) { 
		super(pValue, iValue, dValue, fValue, source);
		this.imu_Source = source;
	}

	@Override
	public void resetSource() { imu_Source.reset(); }
	
	//@Override
	//public void calibrateSource() { imu_Source.calibrate(); }
}