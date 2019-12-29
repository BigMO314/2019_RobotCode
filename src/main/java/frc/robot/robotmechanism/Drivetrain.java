package frc.robot.robotmechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Limelight;
import frc.robot.pidloops.EncoderLoop;
import frc.robot.pidloops.IMULoop;
import frc.robot.pidloops.PIDLoop;
import frc.robot.configuration.ConfigRobot;

public class Drivetrain {
    private TalonSRX mtr_L_Drive_1;
    private TalonSRX mtr_R_Drive_1;

    private double m_L_Power = 0.0;
    private double m_R_Power = 0.0;

    private Encoder enc_Drive;
    private EncoderLoop pid_Drive;

    private AHRS imu_DriveAngle;
    private IMULoop pid_Angle;

    private Limelight lml_Vision;

    public Drivetrain(TalonSRX mtr_L_Drive_1, TalonSRX mtr_R_Drive_1, Encoder enc_Drive, AHRS imu_DriveAngle, Limelight lml_Vision) {
        this.mtr_L_Drive_1 = mtr_L_Drive_1;
        this.mtr_R_Drive_1 = mtr_R_Drive_1;
        this.lml_Vision = lml_Vision;
    }

    //Basic functions
    public void setDrive(double lPower, double rPower) {
        m_L_Power = lPower;
        m_R_Power = rPower;
    }

    public void cheezyDrive(double t, double s) { setDrive(t -s, t + s); }
    public void followTarget(double t) { cheezyDrive(t, lml_Vision.getX() * 0.02); }

    public void update() {
        mtr_L_Drive_1.set(ControlMode.PercentOutput, m_L_Power);
        mtr_R_Drive_1.set(ControlMode.PercentOutput, m_R_Power);
    }
}