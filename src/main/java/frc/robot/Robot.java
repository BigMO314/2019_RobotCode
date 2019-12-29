package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robotmechanism.CargoManagement;
import frc.robot.robotmechanism.Climber;
import frc.robot.robotmechanism.Drivetrain;
import frc.robot.robotmechanism.HatchMech;
import frc.robot.configuration.ConfigRobot;
import frc.robot.controlperiod.Teleoperated;
import edu.wpi.first.wpilibj.CameraServer;

public class Robot extends TimedRobot {

  UsbCamera camera1;
  UsbCamera camera2;

  //Drivetrain
    TalonSRX mtr_L_Drive_1 = new TalonSRX(0);
    VictorSPX mtr_L_Drive_2 = new VictorSPX(1);
    VictorSPX mtr_L_Drive_3 = new VictorSPX(2);
    TalonSRX mtr_R_Drive_1 = new TalonSRX(3);
    VictorSPX mtr_R_Drive_2 = new VictorSPX(4);
    VictorSPX mtr_R_Drive_3 = new VictorSPX(5);
    //2 EncodersCode R', 
    Encoder enc_Drive = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    Encoder enc_L_Drive = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
    AHRS imu_Drive = new AHRS(SerialPort.Port.kMXP);
    Limelight lml_Vision = new Limelight();
    Drivetrain rbt_Drivetrain = new Drivetrain(mtr_L_Drive_1, mtr_R_Drive_1, enc_Drive, imu_Drive, lml_Vision);

  //Mechanism Base

    //Hatch Mech
      VictorSPX mtr_Hatch = new VictorSPX(6);
      Encoder enc_HatchAngle = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
      DigitalInput lim_Hatch = new DigitalInput(6);

      HatchMech rbt_HatchMech = new HatchMech(mtr_Hatch, enc_HatchAngle, lim_Hatch);
    //Cargo Intake
      VictorSPX mtr_Intake = new VictorSPX(7);
      //Cargo Intake Position
      TalonSRX mtr_IntakePosition = new TalonSRX(8);
      DigitalInput pho_Uptake = new DigitalInput(7);
      DigitalInput lim_Intake = new DigitalInput(8);
      
    CargoManagement rbt_CargoManagement = new CargoManagement(mtr_IntakePosition, mtr_Intake, pho_Uptake, lim_Intake);
  
    //Climber
    TalonSRX mtr_Front_Climb = new TalonSRX(9);
    TalonSRX mtr_Back_Climb = new TalonSRX(10);
    TalonSRX mtr_Drive_Climb = new TalonSRX(11);

    Climber rbt_Climber = new Climber(mtr_Front_Climb, mtr_Back_Climb, mtr_Drive_Climb);
  //Controls
    XboxController ctl_Driver = new XboxController(0);
    XboxController ctl_Operator = new XboxController(1);
  //ControlPeriods
    //Teleoperated
      Teleoperated prd_Teleoperated = new Teleoperated(ctl_Driver, ctl_Operator, rbt_Drivetrain, rbt_HatchMech, rbt_CargoManagement, rbt_Climber, lml_Vision);

      // Dashboard.NumberEntry dsh_ClimberP = new Dashboard.NumberEntry("Distance P");
      // Dashboard.NumberEntry dsh_ClimberI = new Dashboard.NumberEntry("Distance I");
      // Dashboard.NumberEntry dsh_ClimberD = new Dashboard.NumberEntry("Distance D");
  public void robotInit() {
    camera1 = CameraServer.getInstance().startAutomaticCapture("Camera 1", 0);
    camera2 = CameraServer.getInstance().startAutomaticCapture("Camera 2", 1);
    camera1.setBrightness(50);
    camera2.setBrightness(50);
    camera2.setWhiteBalanceManual(50);
    camera2.setExposureManual(50);
    camera1.setResolution(160, 100);
    camera2.setResolution(160, 100);
    camera1.setFPS(30);
    camera2.setFPS(10);
    
    //Drivetrain
      //Left
        mtr_L_Drive_1.setInverted(true);
        mtr_L_Drive_2.setInverted(true);
        mtr_L_Drive_3.setInverted(true);

        mtr_L_Drive_2.follow(mtr_L_Drive_1);
        mtr_L_Drive_3.follow(mtr_L_Drive_1);
      
      //Right
        mtr_R_Drive_1.setInverted(false);
        mtr_R_Drive_2.setInverted(false);
        mtr_R_Drive_3.setInverted(false);

        mtr_R_Drive_2.follow(mtr_R_Drive_1);
        mtr_R_Drive_3.follow(mtr_R_Drive_1);

    mtr_Intake.setInverted(true);

    mtr_IntakePosition.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    enc_HatchAngle.reset();

    //Climber
      mtr_Front_Climb.setInverted(false);
      mtr_Back_Climb.setInverted(false);
      mtr_Drive_Climb.setInverted(false);
      mtr_Front_Climb.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
      mtr_Back_Climb.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
      mtr_Drive_Climb.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
      rbt_Climber.configurePeakOutputDrive(0.8, -0.8);
      rbt_Climber.configurePeakOutputClimb(0.7, -0.7);

    // dsh_ClimberP.set(0.1);//0.1
    // dsh_ClimberI.set(0.01);//0.01
    // dsh_ClimberD.set(0.06);//0.06

    rbt_HatchMech.ConfigAngleOutputRange(-0.5, 0.5);
    rbt_CargoManagement.configPeakOutput(0.9, -0.9);
    rbt_CargoManagement.resetAngle();

    rbt_Climber.resetFrontHeight();
    rbt_Climber.resetBackHeight();
    rbt_Climber.resetDistance();

    lml_Vision.changeVisionState(VisionMode.kVisionLEDOff);

  }

  @Override
  public void robotPeriodic() {
    //rbt_Climber.configureHeightPID(dsh_ClimberP.get(), dsh_ClimberI.get(), dsh_ClimberD.get());
    SmartDashboard.putNumber("Front Height", mtr_Front_Climb.getSelectedSensorPosition());
    SmartDashboard.putNumber("Back Height", mtr_Back_Climb.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Limit", lim_Intake.get());
    lml_Vision.update();
  }

  @Override
  public void autonomousInit() {
    
  }
  @Override
  public void autonomousPeriodic() {
    prd_Teleoperated.update();
    rbt_Drivetrain.update();
    rbt_CargoManagement.update();
    rbt_HatchMech.update();
    lml_Vision.update();
  }

  @Override
  public void teleopPeriodic() {
    prd_Teleoperated.update();
    rbt_Drivetrain.update();
    rbt_CargoManagement.update();
    rbt_HatchMech.update();
    rbt_Climber.update();
    lml_Vision.update();
  }

  @Override
  public void testPeriodic() {
  }
}