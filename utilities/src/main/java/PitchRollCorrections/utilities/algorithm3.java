package PitchRollCorrections.utilities;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class algorithm3 {
  public static void main(String[] args) {

    // Initial orientation of the plane
    // Initial orientation of the plane
    Vector3d xAxis = new Vector3d(1, 0, 0);
    Vector3d yAxis = new Vector3d(0, 1, 0);
    Vector3d zAxis = new Vector3d(0, 0, 1);

    // Yaw, pitch, and roll values in degrees
    double yaw = 18.0;
    double pitch = 20.0;
    double roll = 30.0;

    /*
     * Correct frame of reference
     * Using a VectorNav GNSS, the frame of reference is as follows:
     * X: Forward. Roll is about this axis. +Roll is right wing down
     * Y: Right. Pitch is about this axis. +Pitch is nose up
     * Z: Down. Yaw is about this axis. +Yaw clockwise when looking down
     *
     * The VN-310 uses 3,2,1 Euler angle sequence (Tait-Bryan angles) for its
     * orientation:
     * 1. Yaw (Z-axis) is rotated first
     * 2. Pitch (Y-axis) is rotated second
     * 3. Roll (X-axis) is rotated last
     */
    // Convert angles to radians
    double yawRad = Math.toRadians(yaw);
    double pitchRad = Math.toRadians(pitch);
    double rollRad = Math.toRadians(roll);

    // Rotate around Z-axis (Yaw)
    Matrix3d yawMatrix = new Matrix3d();
    yawMatrix.rotZ(yawRad);
    xAxis = rotateVector(yawMatrix, xAxis);
    yAxis = rotateVector(yawMatrix, yAxis);
    zAxis = rotateVector(yawMatrix, zAxis);

    // Print axes after yaw rotation
    printAxes("After Yaw Rotation:", xAxis, yAxis, zAxis);

    // Rotate around Y-axis (Pitch)
    Matrix3d pitchMatrix = new Matrix3d();
    pitchMatrix.rotY(pitchRad);
    xAxis = rotateVector(pitchMatrix, xAxis);
    yAxis = rotateVector(pitchMatrix, yAxis);
    zAxis = rotateVector(pitchMatrix, zAxis);

    // Print axes after pitch rotation
    printAxes("After Pitch Rotation:", xAxis, yAxis, zAxis);

    // Rotate around X-axis (Roll)
    Matrix3d rollMatrix = new Matrix3d();
    rollMatrix.rotX(rollRad);
    xAxis = rotateVector(rollMatrix, xAxis);
    yAxis = rotateVector(rollMatrix, yAxis);
    zAxis = rotateVector(rollMatrix, zAxis);

    // Print axes after roll rotation
    printAxes("After Roll Rotation:", xAxis, yAxis, zAxis);

    // Get orthogonal vector to the plane
    Vector3d orthogonalVector = calculateOrthogonalVector(xAxis, yAxis);
    System.out.println("Orthogonal Vector to the Plane: " + orthogonalVector);
    System.out.println("Plane Equation: " + -orthogonalVector.getX() / orthogonalVector.getZ() + "x + "
        + -orthogonalVector.getY() / orthogonalVector.getZ() + "y = z");
  }

  private static Vector3d rotateVector(Matrix3d matrix, Vector3d vector) {
    Vector3d result = new Vector3d();
    matrix.transform(vector, result);
    return result;
  }

  private static void printAxes(String message, Vector3d xAxis, Vector3d yAxis, Vector3d zAxis) {
    System.out.println(message);
    System.out.println("X-Axis: " + xAxis);
    System.out.println("Y-Axis: " + yAxis);
    System.out.println("Z-Axis: " + zAxis);
    System.out.println();
  }

  private static Vector3d calculateOrthogonalVector(Vector3d xAxis, Vector3d yAxis) {
    Vector3d result = new Vector3d();
    result.cross(xAxis, yAxis);
    return result;
  }
}
