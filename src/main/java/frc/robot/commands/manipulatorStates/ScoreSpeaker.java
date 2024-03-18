// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulatorStates;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreSpeaker extends Command {
    private ManipulatorSubsystem m_manipulator;
    private Timer m_timer;

    public ScoreSpeaker(Timer timer, ManipulatorSubsystem manipulatorSubsystem) {
        m_manipulator = manipulatorSubsystem;
        m_timer = timer;
        addRequirements(manipulatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_manipulator.boxMotorSpeed(ManipulatorConstants.kBoxInFast);
        m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
        m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteOut);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(ManipulatorConstants.kScoreSpeakerTime);
    }
}
