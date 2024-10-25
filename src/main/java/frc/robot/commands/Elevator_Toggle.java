package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Elevator_Tilt;

public class Elevator_Toggle extends InstantCommand {

        private final Elevator_Tilt m_elevator_Tilt;

    public Elevator_Toggle(Elevator_Tilt subsystem) {

        m_elevator_Tilt = subsystem;
        addRequirements(m_elevator_Tilt);

    }

    @Override
    public void initialize() {
        m_elevator_Tilt.myValveToggle();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
