package PitchRollCorrections.utilities;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class algorithm2 {
  public static void main(String[] args) {
    // Input values
    // double platformPitch = -1.43 /* your platform's pitch value */;
    // double platformRoll = -0.04/* your platform's roll value */;
    // double platformYaw = 0.0/* your platform's yaw value */;
    // double cameraAzimuth = 216.72493085263568/* your camera's azimuth value */;
    // double cameraPitch = -6.861661647151266 /* your camera's pitch value */;
    double platformPitch = 0 /* your platform's pitch value */;
    double platformRoll = 0/* your platform's roll value */;
    double platformYaw = 0.0/* your platform's yaw value */;
    double cameraAzimuth = 216.72493085263568/* your camera's azimuth value */;
    double cameraPitch = -6.861661647151266 /* your camera's pitch value */;

    // Convert angles to radians
    double platformPitchRad = Math.toRadians(platformPitch);
    double platformRollRad = Math.toRadians(platformRoll);
    double platformYawRad = Math.toRadians(platformYaw);
    double cameraAzimuthRad = Math.toRadians(cameraAzimuth);
    double cameraPitchRad = Math.toRadians(cameraPitch);

    // Correct camera azimuth and pitch
    double[] correctedAngles = correctCameraAngles(platformPitchRad, platformRollRad, platformYawRad,
        cameraAzimuthRad, cameraPitchRad);

    // Output corrected angles in degrees
    System.out.println("Corrected Camera Azimuth: " + Math.toDegrees(correctedAngles[0]) + " degrees");
    System.out.println("Corrected Camera Pitch: " + Math.toDegrees(correctedAngles[1]) + " degrees");
  }

  private static double[] correctCameraAngles(double platformPitch, double platformRoll, double platformYaw,
      double cameraAzimuth, double cameraPitch) {
    // Create rotation matrices for platform and camera orientations
    Matrix3d platformRotation = createPlatformRotationMatrix(platformPitch, platformRoll, platformYaw);
    Matrix3d cameraRotation = createCameraRotationMatrix(cameraAzimuth, cameraPitch);

    // Combine platform and camera rotations
    Matrix3d totalRotation = new Matrix3d();
    totalRotation.mul(platformRotation, cameraRotation);

    // Extract corrected camera azimuth and pitch from the combined rotation matrix
    double correctedAzimuth = Math.atan2(-totalRotation.m20, totalRotation.m00);
    double correctedPitch = Math.asin(totalRotation.m10);

    return new double[] { correctedAzimuth, correctedPitch };
  }

  private static Matrix3d createPlatformRotationMatrix(double pitch, double roll, double yaw) {
    Matrix3d rotationMatrix = new Matrix3d();
    rotationMatrix.rotY(yaw);
    rotationMatrix.rotX(pitch);
    rotationMatrix.rotZ(roll);
    return rotationMatrix;
  }

  private static Matrix3d createCameraRotationMatrix(double azimuth, double pitch) {
    Matrix3d rotationMatrix = new Matrix3d();
    rotationMatrix.rotY(azimuth);
    rotationMatrix.rotX(pitch);
    return rotationMatrix;
  }
}
