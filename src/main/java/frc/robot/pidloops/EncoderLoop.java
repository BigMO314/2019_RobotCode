package frc.robot.pidloops;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderLoop extends PIDLoop {
    private final Encoder enc_Source;
    
	public EncoderLoop(Encoder source) { this(0.0, 0.0, 0.0, 0.0, source); }
	public EncoderLoop(double pValue, double iValue, double dValue, Encoder source) { this(pValue, iValue, dValue, 0.0, source); }
	public EncoderLoop(double pValue, double iValue, double dValue, double fValue, Encoder source) { 
		super(pValue, iValue, dValue, fValue, source);
		this.enc_Source = source;
	}

	@Override
	public void resetSource() { enc_Source.reset(); }
	
	//@Override
	//public void calibrateSource() { enc_Source.calibrate; }
}