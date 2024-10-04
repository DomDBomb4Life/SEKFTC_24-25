package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline{
    OpenCvCamera webcam;
    OpMode opMode;

    public VisionPipeline(OpMode opMode, OpenCvPipeline pipeline){
        //create the camera instance with live view
        WebcamName webname = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webname, cameraViewId);

        //activate the camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                opMode.telemetry.addLine("Webcam Initialized Successfully");
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("Webcam Initialization Failed", pipeline);
            }
        });

        //set the pipeline
        webcam.setPipeline(pipeline);
    }
}