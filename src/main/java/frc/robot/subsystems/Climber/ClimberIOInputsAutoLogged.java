package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("HookOutputCurrent", hookOutputCurrent);
        table.put("HookOutputVoltage", hookOutputVoltage);
        table.put("HookPositionDeg", hookPositionDeg);
        table.put("HookVelocity", hookVelocity);

        table.put("WheelOutputCurrent", wheelOutputCurrent);
        table.put("WheelOutputVoltage", wheelOutputVoltage);
        table.put("WheelPositionDeg", wheelPositionDeg);
        table.put("WheelVelocity", wheelVelocity);
    }

    @Override
    public void fromLog(LogTable table) {
        hookOutputCurrent = table.get("HookOutputCurrent", hookOutputCurrent);
        hookOutputVoltage = table.get("HookOutputVoltage", hookOutputVoltage);
        hookPositionDeg = table.get("HookPositionDeg", hookPositionDeg);
        hookVelocity = table.get("HookVelocity", hookVelocity);

        wheelOutputCurrent = table.get("WheelOutputCurrent", wheelOutputCurrent);
        wheelOutputVoltage = table.get("WheelOutputVoltage", wheelOutputVoltage);
        wheelPositionDeg = table.get("WheelPositionDeg", wheelPositionDeg);
        wheelVelocity = table.get("WheelVelocity", wheelVelocity);
    }

    @Override
    public ClimberIOInputsAutoLogged clone() {
        ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();

        copy.hookOutputCurrent = hookOutputCurrent;
        copy.hookOutputVoltage = hookOutputVoltage;
        copy.hookPositionDeg = hookPositionDeg;
        copy.hookVelocity = hookVelocity;

        copy.wheelOutputCurrent = wheelOutputCurrent;
        copy.wheelOutputVoltage = wheelOutputVoltage;
        copy.wheelPositionDeg = wheelPositionDeg;
        copy.wheelVelocity = wheelVelocity;

        return copy;
    }
}