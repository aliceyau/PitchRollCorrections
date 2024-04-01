package PitchRollCorrections.utilities;

public class algorithm1 {
  public static void main(String[] args) {
    // Input values
    double platformPitch = -1.43; // Example platform pitch in degrees
    double platformRoll = -0.04; // Example platform roll in degrees
    double cameraAzimuth = 216.72493085263568; // Example camera azimuth in degrees
    double cameraSpecificPitch = -6.861661647151266; // Example camera pitch in its own body frame

    // Convert degrees to radians
    double platformPitchRad = Math.toRadians(platformPitch);
    double platformRollRad = Math.toRadians(platformRoll);
    double cameraAzimuthRad = Math.toRadians(cameraAzimuth);
    double cameraSpecificPitchRad = Math.toRadians(cameraSpecificPitch);

    // Step 1: Combine Platform's Pitch and Roll with Camera Azimuth
    Quat4d platformRotation = calculatePlatformRotation(platformPitchRad,
        platformRollRad);
    Quat4d cameraAzimuthRotation = calculateYawRotation(cameraAzimuthRad);

    Quat4d combinedRotation = combineRotations(platformRotation,
        cameraAzimuthRotation);

    // Step 2: Correct Camera's Pitch
    Quat4d cameraSpecificPitchRotation = calculatePitchRotation(cameraSpecificPitchRad);
    Quat4d finalRotation = combineRotations(cameraSpecificPitchRotation,
        combinedRotation);

    // Extract Euler angles from the final rotation
    double[] euler = getEulerAngles(finalRotation);

    // Print the corrected azimuth and pitch
    System.out.println("Corrected Azimuth: " + -euler[1] + " degrees");
    System.out.println("Corrected Pitch: " + (euler[0]) + " degrees");
  }

  // Helper method to calculate rotation around Y (yaw)
  private static Quat4d calculateYawRotation(double yaw) {
    Quat4d rotation = new Quat4d();
    rotation.set(new AxisAngle4d(0, 1, 0, yaw));
    return rotation;
  }

  // Helper method to calculate rotation around X (pitch)
  private static Quat4d calculatePitchRotation(double pitch) {
    Quat4d rotation = new Quat4d();
    rotation.set(new AxisAngle4d(1, 0, 0, pitch));
    return rotation;
  }

  // Helper method to calculate rotation around YXZ axes
  private static Quat4d calculatePlatformRotation(double pitch, double roll) {
    Quat4d pitchRotation = calculatePitchRotation(pitch);
    Quat4d rollRotation = new Quat4d();
    rollRotation.set(new AxisAngle4d(0, 0, 1, roll));
    return combineRotations(pitchRotation, rollRotation);
  }

  // Helper method to combine two rotations
  private static Quat4d combineRotations(Quat4d rotation1, Quat4d rotation2) {
    Quat4d result = new Quat4d(rotation1);
    result.mul(rotation2);
    return result;
  }

  // Helper method to extract YXZ Euler angles from a quaternion
  private static double[] getEulerAngles(Quat4d quaternion) {
    Matrix3d matrix = new Matrix3d();
    matrix.set(quaternion);
    double[] euler = new double[2]; // Yaw and Pitch only (excluding Roll)
    euler[0] = Math.toDegrees(Math.atan2(matrix.m12, matrix.m22)); // Yaw
    euler[1] = Math.toDegrees(Math.asin(-matrix.m02)); // Pitch
    return euler;
  }
}
