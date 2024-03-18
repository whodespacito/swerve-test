package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {

    private AnalogInput m_inletSensor = new AnalogInput(ManipulatorConstants.kInletColorSensor);
    private AnalogInput m_outletSensor = new AnalogInput(ManipulatorConstants.kOutletColorSensor);

    private static double m_intakeMotorSpeed = 0;
    private static double m_chuteMotorSpeed = 0;

    private static double m_boxMotorSpeed = 0;

    private static double m_elevatorSetPos = 0;
    private static double m_elevatorCurPos = 0;

    private static double m_CNDASetPos = 0;
    private static double m_CNDACurPos = 0;


    private static boolean m_boxFull = false;
    private static boolean m_chuteFull = false;

    private static boolean m_inletDetect;
    private static boolean m_outletDetect;

    private static boolean m_inletLastDetect;
    private static boolean m_outletLastDetect;

    public ManipulatorSubsystem() {}

    @Override
    public void periodic() {

        //in periodic you will actually tell the motors to drive, the methods relating to the motors only change the speed to drive at

        //emulated elevator set position
        if (m_elevatorSetPos < m_elevatorCurPos) {
           m_elevatorCurPos = Math.max(m_elevatorCurPos - .1, 0);
        } else if (m_elevatorSetPos > m_elevatorCurPos) {
           m_elevatorCurPos = Math.min(m_elevatorCurPos + .1, 10);
        }

        //emulated CNDA set position
        if (m_CNDASetPos < m_CNDACurPos) {
           m_CNDACurPos = Math.max(m_CNDACurPos - .1, 0);
        } else if (m_CNDASetPos > m_CNDACurPos) {
           m_CNDACurPos = Math.min(m_CNDACurPos + .1, 10);
        }

        //sets sensors detect and last detect
        m_inletLastDetect = m_inletDetect;
        m_outletLastDetect = m_outletDetect;

        m_inletDetect = m_inletSensor.getVoltage() > ManipulatorConstants.kInletThreshold;
        m_outletDetect = m_outletSensor.getVoltage() > ManipulatorConstants.kOutletThreshold;

        SmartDashboard.putNumber("color Sensor inlet", m_inletSensor.getVoltage());
        SmartDashboard.putNumber("color Sensor outlet", m_outletSensor.getVoltage());

        SmartDashboard.putBoolean("Box full", m_boxFull);
        SmartDashboard.putBoolean("Chute full", m_chuteFull);

        SmartDashboard.putNumber("intake motor speed", m_intakeMotorSpeed);
        SmartDashboard.putNumber("chute motor speed", m_chuteMotorSpeed);
        SmartDashboard.putNumber("box motor speed", m_boxMotorSpeed);

        SmartDashboard.putNumber("elevator set position", m_elevatorSetPos);
        SmartDashboard.putNumber("elevator current position", m_elevatorCurPos);

        SmartDashboard.putNumber("CNDA set position", m_CNDASetPos);
        SmartDashboard.putNumber("CNDA current position", m_CNDACurPos);
    }
    
    
    public void intakeMotorSpeed(double speed) {
        m_intakeMotorSpeed = speed;
    }

    public void chuteMotorSpeed(double speed) {
        m_chuteMotorSpeed = speed;
    }

    public void elevatorSetPosition(double position) {
        m_elevatorSetPos = position;
    }

    public void CNDASetPosition(double position) {
        m_CNDASetPos = position;
    }

    public void boxMotorSpeed(double speed) {
        m_boxMotorSpeed = speed;
    }


    public double getElevatorPosition() {
        return m_elevatorCurPos;
    }

    public double getCNDAPosition() {
        return m_CNDACurPos;
    }

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
