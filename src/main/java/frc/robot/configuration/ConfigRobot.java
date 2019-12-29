package frc.robot.configuration;

import java.lang.Math;

public class ConfigRobot {
    public static final double m_Pi = Math.PI;
    public static final double drivetrainWheelDiameter = 6.125;
    public static final double drivetrainWheelCircumference = m_Pi * drivetrainWheelDiameter;
    public static final double countsPerInch = drivetrainWheelCircumference / 1024;
    //arm
    public static final double degreesPerCount = (360.0 / 4096.0) * (16.0 / 48.0) * (20.0 / 64.0);
    //hatch
    public static final double degreesPerCount_Hatch = 360.0 / 2048.0;
    public static final double countsPerDegree = 2048.0 / 360.0;

    public static final double inchesPerCount = ((1.128 * m_Pi) / 8192); //* (18.0 / 54.0) * (54.0 / 34.0) * (34.0 / 18.0);
    public static final double inchesPerCount_Climber_Drive = ((2.5 * m_Pi) / 4096);





}