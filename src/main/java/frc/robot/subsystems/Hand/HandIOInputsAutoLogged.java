// Raymond: lowercase package.
// Raymond: same as Climber's - this is @AutoLog generated code, and you wrote it but Hand.java never uses it (it uses the raw HandIO.HandIOInputs). either delete it and let @AutoLog generate it, or actually use this class as the inputs field.
package frc.robot.subsystems.Hand;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// Raymond: add Cloneable - reference is `implements LoggableInputs, Cloneable`.
public class HandIOInputsAutoLogged extends HandIO.HandIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("HandMotorVolts", handMotorVolts);
        table.put("HandMotorCurrent", handMotorCurrent);
        table.put("HandPositionDeg", handPositionDeg);
        table.put("HandMotorVelocity", handMotorVelocity);
        table.put("FullClose", fullClose);
        table.put("FullOpen", fullOpen);
    }

    @Override
    public void fromLog(LogTable table) {
        handMotorVolts = table.get("HandMotorVolts", handMotorVolts);
        handMotorCurrent = table.get("HandMotorCurrent", handMotorCurrent);
        handPositionDeg = table.get("HandPositionDeg", handPositionDeg);
        handMotorVelocity = table.get("HandMotorVelocity", handMotorVelocity);
        fullClose = table.get("FullClose", fullClose);
        fullOpen = table.get("FullOpen", fullOpen);
    }

    @Override
    public HandIOInputsAutoLogged clone() {
        HandIOInputsAutoLogged copy = new HandIOInputsAutoLogged();

        copy.handMotorVolts = handMotorVolts;
        copy.handMotorCurrent = handMotorCurrent;
        copy.handPositionDeg = handPositionDeg;
        copy.handMotorVelocity = handMotorVelocity;
        copy.fullClose = fullClose;
        copy.fullOpen = fullOpen;

        return copy;
    }
}