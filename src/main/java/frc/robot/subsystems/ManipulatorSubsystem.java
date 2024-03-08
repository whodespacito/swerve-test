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

    private static double m_elevatorMotorSpeed = 0;
    private static double m_boxMotorSpeed = 0;

    private static double m_elevatorPosition = 0;

    private static boolean m_boxFull = false;

    private static boolean m_inletDetect;
    private static boolean m_outletDetect;

    private static boolean m_inletLastDetect;
    private static boolean m_outletLastDetect;

    public ManipulatorSubsystem() {}

    @Override
    public void periodic() {

        //in periodic you will actually tell the motors to drive, the methods relating to the motors only change the speed to drive at

        m_elevatorPosition = Math.min(Math.max(0, m_elevatorPosition + m_elevatorMotorSpeed), 10);

        //sets sensors detect and last detect
        m_inletLastDetect = m_inletDetect;
        m_outletLastDetect = m_outletDetect;

        m_inletDetect = m_inletSensor.getVoltage() > ManipulatorConstants.kInletThreshold;
        m_outletDetect = m_outletSensor.getVoltage() > ManipulatorConstants.kOutletThreshold;

        SmartDashboard.putNumber("color Sensor inlet", m_inletSensor.getVoltage());
        SmartDashboard.putNumber("color Sensor outlet", m_outletSensor.getVoltage());

        SmartDashboard.putBoolean("Box full", m_boxFull);

        SmartDashboard.putNumber("intake motor speed", m_intakeMotorSpeed);
        SmartDashboard.putNumber("chute motor speed", m_chuteMotorSpeed);
        SmartDashboard.putNumber("elevator motor speed", m_elevatorMotorSpeed);
        SmartDashboard.putNumber("box motor speed", m_boxMotorSpeed);

        SmartDashboard.putNumber("elevator position", m_elevatorPosition);

    }
    
    
    public void intakeMotorSpeed(double speed) {
        m_intakeMotorSpeed = speed;
    }

    public void chuteMotorSpeed(double speed) {
        m_chuteMotorSpeed = speed;
    }

    public void elevatorMotorSpeed(double speed) {
        m_elevatorMotorSpeed = speed;
    }

    public void boxMotorSpeed(double speed) {
        m_boxMotorSpeed = speed;
    }

    public double getElevatorPosition() {
        return m_elevatorPosition;
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

}
