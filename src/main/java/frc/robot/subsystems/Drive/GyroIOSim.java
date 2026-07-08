// Raymond: lowercase package - frc.robot.subsystems.drive (rename the folder). otherwise a clean
// copy of the reference, nothing else here.
package frc.robot.subsystems.drive;

// import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  // private final GyroSimulation gyroSimulation;

  // public GyroIOSim(GyroSimulation gyroSimulation) {
  //     this.gyroSimulation = gyroSimulation;
  // }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    // inputs.yaw_Rot2d = gyroSimulation.getGyroReading();
    // inputs.yawVel_radps = Units.degreesToRadians(
    // gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = null; // PhoenixUtil.getSimulationOdometryTimeStamps();
    // inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
