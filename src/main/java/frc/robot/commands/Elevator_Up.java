package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.security.PrivilegedActionException;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Elevator_Drive;

public class Elevator_Up extends Command {

        private final Elevator_Drive m_elevator_Drive;
        private DoubleSupplier m_elevatorspeed;

    public Elevator_Up(Elevator_Drive Subsystem) {

        m_elevator_Drive = Subsystem;
        addRequirements(m_elevator_Drive);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator_Drive.goToSetpoint1(0.5);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
