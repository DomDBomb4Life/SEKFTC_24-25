// File: AprilTagDetectionPipeline.java
package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.*;

public class AprilTagDetectionPipeline {

    // Method to create and configure the AprilTagProcessor
    public static AprilTagProcessor createAprilTagProcessor() {
        // Create a custom tag library if needed
        AprilTagLibrary myTagLibrary = new AprilTagLibrary.Builder()
                .setAllowOverwrite(true)
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                // Add custom tags here if necessary
                // .addTag(id, name, size, position, DistanceUnit, orientation)
                .build();

        // Build the AprilTagProcessor with desired settings
        return new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagLibrary(myTagLibrary)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506) // fx, fy, cx, cy
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                .build();
    }
}