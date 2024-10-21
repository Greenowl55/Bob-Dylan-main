package frc.robot.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import static frc.robot.Autos.Paths.*;

import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimpleCenter extends SequentialCommandGroup{
    public SimpleCenter (CommandSwerveDrivetrain swerve, Intake Intake, Shooter Shooter, Elevator_Drive elevator, Elevator_Tilt rams){

    Pose2d pose = PathPlannerAuto.getStaringPoseFromAutoFile("Center Simple");

        addCommands(
            Commands.runOnce(() ->  swerve.seedFieldRelative(pose)),
            // Commands.waitSeconds(0.1),
            // rams.runOnce(() -> rams.myValveForward()),
            // elevator.runOnce(() -> elevator.Elevatormove(-0.5)),
            // Commands.waitSeconds(4),
            // elevator.runOnce(() -> elevator.Elevatormove(0)),
            Shooter.runOnce(() -> Shooter.ShooterRunFront(1)),
            Shooter.runOnce(()-> Shooter.ShooterRunBack(1)),
            Intake.runOnce(() -> Intake.Intakerun(-0.3)),
            Commands.waitSeconds(2),
            Intake.runOnce(() -> Intake.Intakerun(1) ),
            Commands.waitSeconds(1),
            Shooter.runOnce(() -> Shooter.ShooterRunFront(0)),
            Shooter.runOnce(()-> Shooter.ShooterRunBack(0)),
            Intake.runOnce(() -> Intake.Intakerun(0)),
            // elevator.runOnce(()-> elevator.Elevatormove(0.5)),
            // Commands.waitSeconds(1.75),
            // elevator.runOnce(()-> elevator.Elevatormove(0)),
            new PathPlannerAuto(("Center Simple")), 
            Commands.waitSeconds(5),
            Commands.runOnce( () -> swerve.seedFieldRelative())
        
        );
    }

}