// Raymond: lowercase package - frc.robot.subsystems.vision.
// Raymond: this file is a straight copy of the reference one (only the package differs) and it's
// normally auto-generated from @AutoLog anyway. heads up: it logs/clones the original reference
// fields, NOT the extra ones you bolted onto VisionIOInputs (estimatedPose, tagCount,
// timestampSeconds, avgTagDistance). so those silently don't get logged.
package frc.robot.subsystems.Vision;

import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

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
