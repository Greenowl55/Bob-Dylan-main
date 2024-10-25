package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator_Drive extends SubsystemBase {
PIDController pid = new PIDController(10, 0, 0.2); // I CHOSE THESE VALUES RANDOMLY THEY WONT WORK MAKE SURE TO TUNE YOUR SYSTEM!!!
  TalonFX motor = new TalonFX(15);
  //elevator_Motor = new TalonFX(15);


  // The simplest approach to using PID
  public Command goToSetpoint1(double setpoint) {
    // run continuously until we cancel the command
    return run(() -> {
      var angleRotations = motor.getPosition().getValueAsDouble(); // get our position using the in built encoder
      var outputVoltage = pid.calculate(angleRotations, setpoint); // calculate our output voltage for the given setpoint
      motor.setVoltage(outputVoltage); // apply the voltage to the motor
    });
  }

  // Stop the loop when it reaches the setpoint. This approach is often less desirable as you have to then keep the system at the setpoint but if you have something like feedforward it can be useful so I'll leave it.
  public Command goToSetpoint2(double setpoint) {
    // run continuously until we cancel the command
    return run(() -> {
        var angleRotations = motor.getPosition().getValueAsDouble(); // get our position using the in built encoder
        var outputVoltage = pid.calculate(angleRotations, setpoint); // calculate our output voltage for the given setpoint
        motor.setVoltage(outputVoltage); // apply the voltage to the motor
      }).until(pid::atSetpoint); // set the end condition to the loop being stable
    }


//Old code
private TalonFX elevator_Motor;   

    public Elevator_Drive() {

elevator_Motor = new TalonFX(15);

//         /* Invert Motor? and set Break Mode */
elevator_Motor.setInverted(false);
elevator_Motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
     }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

//     // Put methods for controlling this subsystem
//     // here. Call these from Commands.

    public void Elevatormove(double speed) {
        elevator_Motor.set(speed);
    }
}

