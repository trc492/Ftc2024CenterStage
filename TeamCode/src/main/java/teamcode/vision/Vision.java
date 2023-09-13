/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcEocvColorBlobProcessor;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcVisionAprilTag;
import TrcFtcLib.ftclib.FtcVisionEocvColorBlob;
import TrcFtcLib.ftclib.FtcVisionTensorFlow;
import teamcode.Robot;
import teamcode.RobotParams;

/**
 * This class implements AprilTag/TensorFlow/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    private static final String moduleName = "Vision";
    private static final int colorConversion = Imgproc.COLOR_BGRA2BGR;
    private static final double[] redBlobColorThresholds = {100.0, 255.0, 0.0, 100.0, 0.0, 60.0};
    private static final double[] blueBlobColorThresholds = {0.0, 60.0, 0.0, 100.0, 100, 255.0};
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams redBlobFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(10000.0)
            .setMinPerimeter(200.0)
            .setWidthRange(100.0, 1000.0)
            .setHeightRange(100.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.0, 1000.0);
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams blueBlobFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(10000.0)
            .setMinPerimeter(200.0)
            .setWidthRange(100.0, 1000.0)
            .setHeightRange(100.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.0, 1000.0);
    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    private static final float TFOD_MIN_CONFIDENCE = 0.75f;
    public static final String[] TARGET_LABELS = {};

    private final Robot robot;
    public FtcVisionAprilTag aprilTagVision;
    private AprilTagProcessor aprilTagProcessor;
    public FtcVisionEocvColorBlob redBlobVision;
    private FtcEocvColorBlobProcessor redBlobProcessor;
    public FtcVisionEocvColorBlob blueBlobVision;
    private FtcEocvColorBlobProcessor blueBlobProcessor;
    public FtcVisionTensorFlow tensorFlowVision;
    private TfodProcessor tensorFlowProcessor;
    private final VisionPortal visionPortal;

    /**
     * Constructor: Create an instance of the object. Vision is required by both Vuforia and TensorFlow and must be
     * instantiated if either is used. However, to use either Vuforia or TensorFlow, one must explicitly initialize
     * them by calling the initVuforia or initTensorFlow methods respectively.
     *
     * @param robot specifies the robot object.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public Vision(Robot robot, TrcDbgTrace tracer)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();

        this.robot = robot;
        if (RobotParams.Preferences.useAprilTagVision)
        {
            robot.globalTracer.traceInfo(moduleName, "Starting AprilTagVision...");
            FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                .setDrawTagIdEnabled(true)
                .setDrawTagOutlineEnabled(true)
                .setDrawAxesEnabled(false)
                .setDrawCubeProjectionEnabled(false)
                .setLensIntrinsics(
                    RobotParams.WEBCAM_FX, RobotParams.WEBCAM_FY, RobotParams.WEBCAM_CX, RobotParams.WEBCAM_CY)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
            aprilTagVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11, tracer);
            aprilTagProcessor = aprilTagVision.getVisionProcessor();
        }

        if (RobotParams.Preferences.useColorBlobVision)
        {
            robot.globalTracer.traceInfo(moduleName, "Starting ColorBlobVision...");
            redBlobVision = new FtcVisionEocvColorBlob(
                "RedBlob", colorConversion, redBlobColorThresholds, redBlobFilterContourParams,
                RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
            redBlobProcessor = redBlobVision.getVisionProcessor();
            blueBlobVision = new FtcVisionEocvColorBlob(
                "BlueBlob", colorConversion, blueBlobColorThresholds, blueBlobFilterContourParams,
                RobotParams.cameraRect, RobotParams.worldRect, true, tracer);
            blueBlobProcessor = blueBlobVision.getVisionProcessor();
        }

        if (RobotParams.Preferences.useTensorFlowVision)
        {
            robot.globalTracer.traceInfo(moduleName, "Starting TensorFlowVision...");
            tensorFlowVision = new FtcVisionTensorFlow(
                null, TFOD_MODEL_ASSET, TARGET_LABELS, RobotParams.cameraRect, RobotParams.worldRect, tracer);
            tensorFlowProcessor = tensorFlowVision.getVisionProcessor();
            tensorFlowProcessor.setMinResultConfidence(TFOD_MIN_CONFIDENCE);
        }

        VisionPortal.Builder builder = new VisionPortal.Builder();
        // Set the camera (webcam vs. built-in RC phone camera).
        if (RobotParams.Preferences.useWebCam)
        {
            builder.setCamera(opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM));
        }
        else
        {
            builder.setCamera(
                RobotParams.Preferences.useBuiltinCamBack? BuiltinCameraDirection.BACK: BuiltinCameraDirection.FRONT);
        }
        builder.setCameraResolution(new Size(RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT));

        if (RobotParams.Preferences.showVisionView)
        {

            builder.enableLiveView(true);
            builder.setAutoStopLiveView(true);
            //Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //  builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        }
        else
        {
            builder.enableLiveView(false);
        }

        if (aprilTagProcessor != null)
        {
            builder.addProcessor(aprilTagProcessor);
        }

        if (redBlobProcessor != null)
        {
            builder.addProcessor(redBlobProcessor);
        }

        if (blueBlobProcessor != null)
        {
            builder.addProcessor(blueBlobProcessor);
        }

        if (tensorFlowProcessor != null)
        {
            builder.addProcessor(tensorFlowProcessor);
        }
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Disable all vision processor until they are needed.
        setAprilTagVisionEnabled(false);
        setRedBlobVisionEnabled(false);
        setBlueBlobVisionEnabled(false);
        setTensorFlowVisionEnabled(false);
    }   //Vision

    public void setAprilTagVisionEnabled(boolean enabled)
    {
        if (aprilTagProcessor != null)
        {
            visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
        }
    }   //setAprilTagVisionEnabled

    public void setRedBlobVisionEnabled(boolean enabled)
    {
        if (redBlobProcessor != null)
        {
            visionPortal.setProcessorEnabled(redBlobProcessor, enabled);
        }
    }   //setRedBlobVisionEnabled

    public void setBlueBlobVisionEnabled(boolean enabled)
    {
        if (blueBlobProcessor != null)
        {
            visionPortal.setProcessorEnabled(blueBlobProcessor, enabled);
        }
    }   //setBlueBlobVisionEnabled

    public void setTensorFlowVisionEnabled(boolean enabled)
    {
        if (tensorFlowProcessor != null)
        {
            visionPortal.setProcessorEnabled(tensorFlowProcessor, enabled);
        }
    }   //setTensorFlowVisionEnabled

    public boolean isAprilTagVisionEabled()
    {
        return aprilTagProcessor != null && visionPortal.getProcessorEnabled(aprilTagProcessor);
    }   //isAprilTagVisionEnabled

    public boolean isRedBlobVisionEabled()
    {
        return redBlobProcessor != null && visionPortal.getProcessorEnabled(redBlobProcessor);
    }   //isRedBlobVisionEnabled

    public boolean isBlueBlobVisionEabled()
    {
        return blueBlobProcessor != null && visionPortal.getProcessorEnabled(blueBlobProcessor);
    }   //isBlueBlobVisionEnabled

    public boolean isTensorFlowVisionEabled()
    {
        return tensorFlowProcessor != null && visionPortal.getProcessorEnabled(tensorFlowProcessor);
    }   //isTensorFlowVisionEnabled

    /**
     * This method updates the LED state to show the vision detected object.
     *
     * @param label specifies the detected object.
     */
    private void updateVisionLEDs(String label)
    {
        if (label != null && robot.blinkin != null)
        {
            robot.blinkin.setPatternState(label, true, 1.0);
        }
    }   //updateVisionLEDs

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
    private int compareConfidence(
        TrcVisionTargetInfo<FtcVisionTensorFlow.DetectedObject> a,
        TrcVisionTargetInfo<FtcVisionTensorFlow.DetectedObject> b)
    {
        return (int)((b.detectedObj.confidence - a.detectedObj.confidence)*100);
    }   //compareConfidence

}   //class Vision
