package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class ManipulatorSubsystem extends SubsystemBase {

    private AnalogInput m_inletSensor = new AnalogInput(ManipulatorConstants.kInletColorSensor);
    private AnalogInput m_outletSensor = new AnalogInput(ManipulatorConstants.kOutletColorSensor);

    private final CANSparkMax m_elevator;
    private final CANSparkMax m_intakeFront;
    private final CANSparkMax m_intakeRear;
    private final CANSparkMax m_CNDA;
//    private final CANSparkMax m_climber;

    private final VictorSPX  m_boxLeft;
    private final TalonSRX  m_boxRight;
    private final VictorSPX  m_chute;

    private final SparkPIDController m_CNDAPID;
    private final SparkPIDController m_elevatorPID;

    private final RelativeEncoder m_elevatorEncoder;
    private final RelativeEncoder m_CNDAEncoder;

//    private static double m_intakeMotorSpeed = 0;
    private static double m_intakeFrontMotorSpeed = 0;
    private static double m_intakeRearMotorSpeed = 0;
    private static double m_chuteMotorSpeed = 0;

    private static double m_boxLeftMotorSpeed = 0;
    private static double m_boxRightMotorSpeed = 0;

    private static double m_elevatorSetPos = 0;

    private static double m_CNDASetPos = ManipulatorConstants.kCNDAStartupPos;


    private static boolean m_boxFull = false;
    private static boolean m_chuteFull = false;

    private static boolean m_inletDetect;
    private static boolean m_outletDetect;

    private static boolean m_inletLastDetect;
    private static boolean m_outletLastDetect;

    public ManipulatorSubsystem() {
        m_elevator = new CANSparkMax(ManipulatorConstants.kElevatorCanId, MotorType.kBrushless);
        m_intakeFront = new CANSparkMax(ManipulatorConstants.kIntakeFrontCanId, MotorType.kBrushless);
        m_intakeRear = new CANSparkMax(ManipulatorConstants.kIntakeRearCanId, MotorType.kBrushless);
        m_CNDA = new CANSparkMax(ManipulatorConstants.kCNDACanId, MotorType.kBrushless);
//        m_climber = new CANSparkMax(ManipulatorConstants.kClimberCanId, MotorType.kBrushless);

        m_boxRight = new TalonSRX(ManipulatorConstants.kBoxRightCanId);
        m_chute = new VictorSPX(ManipulatorConstants.kChuteCanId);
        m_boxLeft = new VictorSPX(ManipulatorConstants.kBoxLeftCanId);

        m_elevatorEncoder = m_elevator.getEncoder();
        m_CNDAEncoder = m_CNDA.getEncoder();

        m_elevatorPID = m_elevator.getPIDController();
        m_elevatorPID.setP(1);
        m_elevatorPID.setI(0);
        m_elevatorPID.setD(0);
        m_elevatorPID.setFF(0);
        m_elevatorPID.setOutputRange(-1, 1);

        m_CNDAPID = m_CNDA.getPIDController();
        m_CNDAPID.setP(ManipulatorConstants.kCNDAP);
        m_CNDAPID.setI(ManipulatorConstants.kCNDAI);
        m_CNDAPID.setD(ManipulatorConstants.kCNDAD);
        m_CNDAPID.setFF(ManipulatorConstants.kCNDAFF);
        m_CNDAPID.setOutputRange(-1, 1);

        m_elevator.burnFlash();
        m_CNDA.burnFlash();
    }

    @Override
    public void periodic() {

        //in periodic you will actually tell the motors to drive, the methods relating to the motors only change the speed to drive at

        m_elevatorPID.setReference(m_elevatorSetPos, ControlType.kPosition);
        m_CNDAPID.setReference(m_CNDASetPos, ControlType.kPosition);

        m_intakeFront.set(m_intakeFrontMotorSpeed);
        m_intakeRear.set(m_intakeRearMotorSpeed);

        m_boxLeft.set(VictorSPXControlMode.PercentOutput, m_boxLeftMotorSpeed);
        m_boxRight.set(TalonSRXControlMode.PercentOutput, m_boxRightMotorSpeed);

        m_chute.set(VictorSPXControlMode.PercentOutput, m_chuteMotorSpeed);


        /* 
        //emulated elevator set position
        if (m_elevatorSetPos < m_elevatorCurPos) {
           m_elevatorCurPos = Math.max(m_elevatorCurPos - .1, 0);
        } else if (m_elevatorSetPos > m_elevatorCurPos) {
           m_elevatorCurPos = Math.min(m_elevatorCurPos + .1, 100);
        }

        //emulated CNDA set position
        if (m_CNDASetPos < m_CNDACurPos) {
           m_CNDACurPos = Math.max(m_CNDACurPos - .1, 0);
        } else if (m_CNDASetPos > m_CNDACurPos) {
           m_CNDACurPos = Math.min(m_CNDACurPos + .1, 10);
        }
        */

        //sets sensors detect and last detect
        m_inletLastDetect = m_inletDetect;
        m_outletLastDetect = m_outletDetect;

        m_inletDetect = m_inletSensor.getVoltage() > ManipulatorConstants.kInletThreshold;
        m_outletDetect = m_outletSensor.getVoltage() > ManipulatorConstants.kOutletThreshold;

        SmartDashboard.putNumber("color Sensor inlet", m_inletSensor.getVoltage());
        SmartDashboard.putNumber("color Sensor outlet", m_outletSensor.getVoltage());

        SmartDashboard.putBoolean("Box full", m_boxFull);
        SmartDashboard.putBoolean("Chute full", m_chuteFull);

        SmartDashboard.putNumber("intake front motor speed", m_intakeFrontMotorSpeed);
        SmartDashboard.putNumber("intake rear motor speed", m_intakeRearMotorSpeed);

        SmartDashboard.putNumber("chute motor speed", m_chuteMotorSpeed);

        SmartDashboard.putNumber("box left motor speed", m_boxLeftMotorSpeed);
        SmartDashboard.putNumber("box right motor speed", m_boxRightMotorSpeed);

        SmartDashboard.putNumber("elevator set position", m_elevatorSetPos);
        SmartDashboard.putNumber("elevator current position", m_elevatorEncoder.getPosition());

        SmartDashboard.putNumber("CNDA set position", m_CNDASetPos);
        SmartDashboard.putNumber("CNDA current position", m_CNDAEncoder.getPosition());

//        SmartDashboard.putNumber("CNDA current position", m_CNDACurPos);
//         SmartDashboard.putNumber("elevator current position", m_elevatorCurPos);
    }
    
    //regular motors

    // public void intakeMotorSpeed(double speed) {
    // m_intakeMotorSpeed = speed;
    // }

    public void intakeFrontMotorSpeed(double speed) {
        m_intakeFrontMotorSpeed = speed;
    }

    public void intakeRearMotorSpeed(double speed) {
        m_intakeRearMotorSpeed = speed;
    }

    public void chuteMotorSpeed(double speed) {
        m_chuteMotorSpeed = speed;
    }

    //box motors
    public void boxLeftMotorSpeed(double speed) {
        m_boxLeftMotorSpeed = speed;
    }

    public void boxRightMotorSpeed(double speed) {
        m_boxRightMotorSpeed = speed;
    }

    //position motors
    public void elevatorSetPosition(double position) {
        m_elevatorSetPos = position;
    }

    public void CNDASetPosition(double position) {
        m_CNDASetPos = position;
    }

    //get positions    
    public double getElevatorPosition() {
        return m_elevatorEncoder.getPosition();
        //return m_elevatorCurPos;
        
    }

    public double getCNDAPosition() {
        return m_CNDAEncoder.getPosition();
        //return m_CNDACurPos;
    }

    //color sensors
    public boolean inletSensorDetect(boolean lastDetect) {
        if (lastDetect) {
            return m_inletLastDetect;
        } else {
            return m_inletDetect;
        }
    }

    public boolean outletSensorDetect(boolean lastDetect) {
        if (lastDetect) {
            return m_outletLastDetect;
        } else {
            return m_outletDetect;
        }
    }

    public void boxSetState(boolean isFull) {
        m_boxFull = isFull;
    }

    public boolean isBoxFull() {
        return m_boxFull;
    }

    public void chuteSetState(boolean isFull) {
        m_chuteFull = isFull;
    }

    public boolean isChuteFull() {
        return m_chuteFull;
    }

}
