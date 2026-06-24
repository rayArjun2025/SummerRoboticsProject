package frc.robot.subsystems.Vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;


public class VisionIOInputsAutoLogged extends VisionIOInputs implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("HasTarget", hasTarget);
    table.put("TargetXDegrees", targetXDegrees);
    table.put("TargetYDegrees", targetYDegrees);
    table.put("TargetAreaPercent", targetAreaPercent);
    table.put("VisibleTagIds", visibleTagIds);
    table.put("PoseObservations", poseObservations);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    hasTarget = table.get("HasTarget", hasTarget);
    targetXDegrees = table.get("TargetXDegrees", targetXDegrees);
    targetYDegrees = table.get("TargetYDegrees", targetYDegrees);
    targetAreaPercent = table.get("TargetAreaPercent", targetAreaPercent);
    visibleTagIds = table.get("VisibleTagIds", visibleTagIds);
    poseObservations = table.get("PoseObservations", poseObservations);
  }

  @Override
  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.hasTarget = this.hasTarget;
    copy.targetXDegrees = this.targetXDegrees;
    copy.targetYDegrees = this.targetYDegrees;
    copy.targetAreaPercent = this.targetAreaPercent;
    copy.visibleTagIds = this.visibleTagIds.clone();
    copy.poseObservations = this.poseObservations.clone();
    return copy;
  }
}