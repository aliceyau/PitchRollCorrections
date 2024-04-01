package PitchRollCorrections.utilities;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class algorithm4 {
  public static void main(String[] args) {
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
    double yaw = Math.toRadians(131.73731758488202);
    double pitch = Math.toRadians(0.21);
    double roll = Math.toRadians(-0.8);
    double cameraAzimuth = Math.toRadians(0);
    double cameraPitch = Math.toRadians(-0.3);

    System.out.println("---------------------- Creating Pitch Roll Plane ------------------------- ");
    Vector3d orthogonalToPlane = createPitchRollPlane(pitch, roll, yaw);
    System.out.println("---------------------- Calculating Camera Vector ------------------------- ");
    Vector3d cameraVector = calculateCameraVector(orthogonalToPlane, cameraAzimuth, cameraPitch);
    double trueAzimuth = CalculateAzimuth(cameraVector);
    double trueElevation = CalculateElevation(cameraVector);
    System.out.println("Corrected Azimuth: " + trueAzimuth);
    System.out.println("Corrected Elevation: " + trueElevation);
  }

  private static Vector3d calculateCameraVector(Vector3d pitchRollOrthogonalvector, double azimuth,
      double elevation) {

    // System.out.println("\nPitch/Roll Orthogonal Vector: " +
    // pitchRollOrthogonalvector);
    Vector3d orthogonalVectorToXZPlane = new Vector3d(0, 1, 0);
    Vector3d intersectionVector = calculateOrthogonalVector(pitchRollOrthogonalvector, orthogonalVectorToXZPlane);
    intersectionVector.negate(); // align with SAE standard convention
    System.out.println("Intersection Vector between pitch/roll plane and xz plane: " + intersectionVector);
    Vector3d rotatedVector = rotateVector(intersectionVector, pitchRollOrthogonalvector, azimuth);
    Vector3d elevationRotationVector = calculateOrthogonalVector(rotatedVector, pitchRollOrthogonalvector);
    rotatedVector = rotateVector(rotatedVector, elevationRotationVector, elevation);
    System.out.println("FINAL Rotated Camera Vector: " + rotatedVector);
    return rotatedVector;
  }

  private static Vector3d createPitchRollPlane(double pitch, double roll, double yaw) {

    Vector3d xAxis = new Vector3d(1, 0, 0);
    Vector3d yAxis = new Vector3d(0, 1, 0);
    Vector3d zAxis = new Vector3d(0, 0, 1);

    // Step 1: Rotate around Z-axis (Yaw)
    Matrix3d yawMatrix = new Matrix3d();
    yawMatrix.rotZ(yaw);
    xAxis = rotateVector(yawMatrix, xAxis);
    yAxis = rotateVector(yawMatrix, yAxis);
    zAxis = rotateVector(yawMatrix, zAxis);

    // Step 2: Rotate around Y-axis (Pitch)
    Matrix3d pitchMatrix = new Matrix3d();
    pitchMatrix.rotY(pitch);
    xAxis = rotateVector(pitchMatrix, xAxis);
    yAxis = rotateVector(pitchMatrix, yAxis);
    zAxis = rotateVector(pitchMatrix, zAxis);

    // Step 3: Rotate around X-axis (Roll)
    // Rotate around X-axis (Roll)
    Matrix3d rollMatrix = new Matrix3d();
    rollMatrix.rotX(roll);
    xAxis = rotateVector(rollMatrix, xAxis);
    yAxis = rotateVector(rollMatrix, yAxis);
    zAxis = rotateVector(rollMatrix, zAxis);

    Vector3d ortho = calculateOrthogonalVector(xAxis, yAxis);
    if (ortho.getZ() == 0) {
      System.out.println("Plane Equation: " + ortho.getX() + "x + " + ortho.getY() + "y = 0");
    } else {
      System.out.println("Plane Equation: " + -ortho.getX() / ortho.getZ() + "x + "
          + -ortho.getY() / ortho.getZ() + "y = z");
    }
    System.out.println("Orthogonal Vector to the Plane: " + ortho);
    System.out.println("X-Axis: " + xAxis);
    System.out.println("Y-Axis: " + yAxis);
    System.out.println("Z-Axis: " + zAxis);

    return ortho;
  }

  private static Vector3d rotateVector(Matrix3d matrix, Vector3d vector) {
    Vector3d result = new Vector3d();
    matrix.transform(vector, result);
    return result;
  }

  private static Vector3d calculateOrthogonalVector(Vector3d xAxis, Vector3d yAxis) {
    Vector3d result = new Vector3d();
    result.cross(xAxis, yAxis);
    return result;
  }

  /**
   * Calculate Rotation Quaternion
   */
  public static Quat4d calculateRotationQuaternion(Vector3d rotationAxis, Double rotationAngleDegree) {
    /**
     * Convert axis and angle to a rotation quaternion:
     * converting to a quaternion representation allows us to perform trig equations
     * ahead of time, meaning that we will save
     * computer cycles when we perform the actual rotation. We will also not have to
     * worry about gimbal lock
     * This follows the general formula for converting axis-angle to quaternion:
     * q0 = cos(theta/2)
     * q1 = x*sin(theta/2)
     * q2 = y*sin(theta/2)
     * q3 = z*sin(theta/2)
     * Where sin(theta/2) is used to calculate the imaginary components of the
     * quaternion
     */

    double rotationAngleRadians = Math.toRadians(rotationAngleDegree);
    double q0 = Math.cos(rotationAngleRadians / 2);
    double imaginaryComponent = Math.sin(rotationAngleRadians / 2);
    double q1 = rotationAxis.getX() * imaginaryComponent;
    double q2 = rotationAxis.getY() * imaginaryComponent;
    double q3 = rotationAxis.getZ() * imaginaryComponent;
    Quat4d rotationQuaternion = new Quat4d(q1, q2, q3, q0);
    return rotationQuaternion;
  }

  /**
   * Calculate vector using quaternion rotation using Rodrigues' rotation formula
   * V' = q*V*q'
   */
  public static Vector3d calculateRotatedVector(Vector3d vector, Quat4d rotationQuaternion) {
    // convert vector to be rotated into a quaternion
    Quat4d vectorQuaternion = new Quat4d(vector.getX(), vector.getY(), vector.getZ(), 0);
    // Perform the rotation. We will be doing passive rotation where the coordinate
    // system rotates with respect to the vector
    Quat4d conjugateRotationQuat = new Quat4d();
    conjugateRotationQuat.conjugate(rotationQuaternion);
    Quat4d rotatedVectorQuaternion = new Quat4d();
    rotatedVectorQuaternion.mul(rotationQuaternion, vectorQuaternion);
    rotatedVectorQuaternion.mul(rotatedVectorQuaternion, conjugateRotationQuat);
    Vector3d rotatedVector = new Vector3d(rotatedVectorQuaternion.getX(), rotatedVectorQuaternion.getY(),
        rotatedVectorQuaternion.getZ());
    return rotatedVector;
  }

  /**
   * Calculate a vector's azimuth from the X axis via projecting the vector onto
   * the X-Y plane and calculating the angle between the projection and the X axis
   */
  public static double CalculateAzimuth(Vector3d vector) {
    Vector3d projection = new Vector3d(vector.getX(), vector.getY(), 0);
    Vector3d xAxis = new Vector3d(1, 0, 0);
    double angle = CalculateAngleBetweenVectors(projection, xAxis);
    return angle;
  }

  /**
   * Calculate a vector's elevation from the X-Y plane via projecting the vector
   * onto the X-Y plane and calculating the angle between the vector and the
   * projection
   */
  public static double CalculateElevation(Vector3d vector) {
    return Math.toDegrees(vector.getZ());
  }

  /**
   * Calculate angle between two vectors
   */
  private static double CalculateAngleBetweenVectors(Vector3d vector1, Vector3d vector2) {
    System.out.println("\nCalculating angle between vectors---\nVector1:  " + vector1 + " \nVector2: " + vector2);
    double dotProduct = vector1.dot(vector2);
    double magnitude1 = vector1.length();
    double magnitude2 = vector2.length();
    double angleRadian = Math.acos(dotProduct / (magnitude1 * magnitude2));

    // Calculate the sign of the angle using the manual cross product
    double crossProductZ = vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
    double sign = -Math.signum(crossProductZ); // Assuming Up is in the Z direction
    double angleDegree = Math.toDegrees(angleRadian) * sign;
    return angleDegree;
  }

  /*
   * Projecting vector onto the plane using the formula:
   * proj_v(u) = u - ((u . n) / |n|^2) * n
   */
  public static Vector3d calculateProjection(Vector3d vector, Vector3d normalVector) {
    if (normalVector.dot(vector) < 0) {
      normalVector.negate();
    }
    normalVector.normalize();

    Vector3d projection = new Vector3d(vector);
    double dotProduct = vector.dot(normalVector);
    double magnitudeSquared = normalVector.lengthSquared();
    Vector3d temp = new Vector3d(normalVector);
    temp.scale(dotProduct / magnitudeSquared);
    projection.sub(temp);
    System.out.println("Projected Vector: " + projection);
    System.out.println("The projected vector is below the xy plane: " + (projection.getZ() < 0));
    return projection;
  }

  private static Vector3d rotateVector(Vector3d vector1, Vector3d vector2, double angleRadians) {
    System.out.println("Vector1: + " + vector1);
    System.out.println("Vector2: + " + vector2);
    System.out.println("Angle in Radians: " + angleRadians);
    // Normalize vectors
    vector2.normalize();

    Matrix3d rotationMatrix = new Matrix3d();
    rotationMatrix.set(new AxisAngle4d(vector2, angleRadians));

    // Apply the rotation to the vector
    rotationMatrix.transform(vector1);

    System.out.println("Rotated Vector: " + vector1);
    return vector1;
  }
}
