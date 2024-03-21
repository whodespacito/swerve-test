// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulatorStates;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreSpeaker extends Command {
    private ManipulatorSubsystem m_manipulator;

    private BooleanSupplier m_outletTrailing = () -> m_manipulator.outletSensorDetect(true) && !m_manipulator.outletSensorDetect(false);

    private int m_observedEdge;

    public ScoreSpeaker(ManipulatorSubsystem manipulatorSubsystem) {
        m_manipulator = manipulatorSubsystem;
        addRequirements(manipulatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_observedEdge = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_manipulator.boxLeftMotorSpeed(ManipulatorConstants.kBoxLeftOutFast);
        m_manipulator.boxRightMotorSpeed(ManipulatorConstants.kBoxRightOutFast);

//        m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
        m_manipulator.intakeFrontMotorSpeed(ManipulatorConstants.kIntakeFrontOut);
        m_manipulator.intakeRearMotorSpeed(ManipulatorConstants.kIntakeRearOut);

        m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteIn);

        if (m_outletTrailing.getAsBoolean()) {
            m_observedEdge++;
        }

        SmartDashboard.putNumber("scoreSpeaker observed edge", m_observedEdge);
        SmartDashboard.putString("command:", "Score Speaker");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_observedEdge >= 2;
    }
}
