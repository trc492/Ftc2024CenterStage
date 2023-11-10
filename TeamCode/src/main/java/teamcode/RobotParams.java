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

package teamcode;

import android.os.Environment;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.openftc.easyopencv.OpenCvCameraRotation;

import TrcCommonLib.trclib.TrcDriveBase.DriveOrientation;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcUtil;
import TrcFtcLib.ftclib.FtcGamepad;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It controls enabling/disabling of various robot features.
     */
    public static class Preferences
    {
        // Miscellaneous
        public static boolean useTraceLog = true;
        public static boolean useLoopPerformanceMonitor = true;
        public static boolean useBlinkin = true;
        public static boolean useBatteryMonitor = false;
        public static boolean doStatusUpdate = true;
        // Vision
        public static boolean useWebCam = true;
        public static boolean hasWebCam2 = true;
        public static boolean useBuiltinCamBack = false;
        public static boolean tuneColorBlobVision = false;
        public static boolean useAprilTagVision = true;
        public static boolean useColorBlobVision = true;
        public static boolean useTensorFlowVision = false;
        public static boolean useTfodModelAsset = false;
        public static boolean showVisionView = true;
        // Robot
        public static boolean noRobot = false;
        public static boolean swerveRobot = false;
        public static boolean robotPowerPlay = false;
        public static boolean robotCenterStage = true;
        public static boolean swerveDualServoSteering = true;
        // Drive Base
        public static boolean useExternalOdometry = true;
        public static boolean doSwervePhysicalAlignment = false;
        // Subsystems
        public static boolean useSubsystems = true;
        public static boolean useElevatorArm = true;
        public static boolean useElevator = true;
        public static boolean useArm = true;
        public static boolean useWrist = true;
        public static boolean useIntake = true;
        public static boolean intakeHasSensor = false;
        public static boolean usePixelTray = true;
        public static boolean useLauncher = true;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "CenterStage_2024";
    public static RevHubOrientationOnRobot.LogoFacingDirection hubLogoDirection =
        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection hubUsbDirection =
        RevHubOrientationOnRobot.UsbFacingDirection.UP;
    public static final String TEAM_FOLDER_PATH                 =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
    public static final String LOG_FOLDER_PATH                  = TEAM_FOLDER_PATH + "/tracelogs";
    public static final String STEERING_CALIBRATION_DATA_FILE   = "SteerCalibration.txt";
    //
    // Hardware names.
    //
    // Miscellaneous.
    public static final String HWNAME_IMU                       = "imu";
    public static final String HWNAME_WEBCAM1                   = "Webcam 1";
    public static final String HWNAME_WEBCAM2                   = "Webcam 2";
    public static final String HWNAME_BLINKIN                   = "blinkin";
    // Drive Base.
    public static final String HWNAME_LFDRIVE_MOTOR             = "lfDriveMotor";
    public static final String HWNAME_RFDRIVE_MOTOR             = "rfDriveMotor";
    public static final String HWNAME_LBDRIVE_MOTOR             = "lbDriveMotor";
    public static final String HWNAME_RBDRIVE_MOTOR             = "rbDriveMotor";
    public static final String HWNAME_LFSTEER_SERVO             = "lfSteerServo";
    public static final String HWNAME_RFSTEER_SERVO             = "rfSteerServo";
    public static final String HWNAME_LBSTEER_SERVO             = "lbSteerServo";
    public static final String HWNAME_RBSTEER_SERVO             = "rbSteerServo";
    public static final String HWNAME_LFSTEER_ENCODER           = "lfSteerEncoder";
    public static final String HWNAME_RFSTEER_ENCODER           = "rfSteerEncoder";
    public static final String HWNAME_LBSTEER_ENCODER           = "lbSteerEncoder";
    public static final String HWNAME_RBSTEER_ENCODER           = "rbSteerEncoder";
    // Subsystems.
    public static final String HWNAME_ELEVATOR                  = "elevator";
    public static final String HWNAME_ARM                       = "arm";
    public static final String HWNAME_WRIST                     = "wrist";
    public static final String HWNAME_INTAKE                    = "intake";
    public static final String HWNAME_PIXELTRAY                 = "pixelTray";
    public static final String HWNAME_LAUNCHER                  = "launcher";
    //
    // Field dimensions.
    //
    public static final double FULL_FIELD_INCHES                = 141.24;
    public static final double HALF_FIELD_INCHES                = FULL_FIELD_INCHES/2.0;
    public static final double FULL_TILE_INCHES                 = FULL_FIELD_INCHES/6.0;
    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                     = 18.00;
    public static final double ROBOT_WIDTH                      = 17.25;
    public static final double DRIVE_BASE_LENGTH                = (24.0 * 15)*TrcUtil.INCHES_PER_MM;
    public static final double DRIVE_BASE_WIDTH                 = 390.0 * TrcUtil.INCHES_PER_MM;
    //
    // Game related locations.
    //
    // Robot start locations in inches.
    public static final double STARTPOS_AUDIENCE_X              = -1.5 * FULL_TILE_INCHES;
    public static final double STARTPOS_BACKSTAGE_X             = 0.5 * FULL_TILE_INCHES;
    public static final double STARTPOS_BLUE_Y                  = HALF_FIELD_INCHES - ROBOT_LENGTH / 2.0;
    public static final TrcPose2D STARTPOS_BLUE_AUDIENCE        = new TrcPose2D(
        STARTPOS_AUDIENCE_X, STARTPOS_BLUE_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_BACKSTAGE       = new TrcPose2D(
        STARTPOS_BACKSTAGE_X, STARTPOS_BLUE_Y, 180.0);
    // Robot park locations in tile units.
    public static final double PARKPOS_X                        = 2.7;
    public static final double PARKPOS_BLUE_CORNER_Y            = 2.5;
    public static final double PARKPOS_BLUE_CENTER_Y            = 0.5;
    public static final TrcPose2D PARKPOS_BLUE_CORNER           = new TrcPose2D(
        PARKPOS_X, PARKPOS_BLUE_CORNER_Y, -90.0);
    public static final TrcPose2D PARKPOS_BLUE_CENTER           = new TrcPose2D(
        PARKPOS_X, PARKPOS_BLUE_CENTER_Y, -90.0);
    // Spike Mark locations to place the pixel in tile units.
    public static final double SPIKE_MARK_ANGLE_OFFSET          = 90.0;
    public static final double SPIKE_MARK_Y_OFFSET              = 0.3;
    public static final double AUDIENCE_SPIKES_X                = -1.5;
    public static final double BACKSTAGE_SPIKES_X               = 0.5;
    public static final double BLUE_SPIKES_Y                    = 1.5;
    public static final TrcPose2D[] BLUE_AUDIENCE_SPIKE_MARKS   = new TrcPose2D[] {
        new TrcPose2D(AUDIENCE_SPIKES_X, BLUE_SPIKES_Y - SPIKE_MARK_Y_OFFSET, 180.0 - SPIKE_MARK_ANGLE_OFFSET),
        new TrcPose2D(AUDIENCE_SPIKES_X, BLUE_SPIKES_Y, 180.0),
        new TrcPose2D(AUDIENCE_SPIKES_X, BLUE_SPIKES_Y - SPIKE_MARK_Y_OFFSET, 180.0 + SPIKE_MARK_ANGLE_OFFSET)
    };
    public static final TrcPose2D[] BLUE_BACKSTAGE_SPIKE_MARKS  = new TrcPose2D[] {
        new TrcPose2D(BACKSTAGE_SPIKES_X, BLUE_SPIKES_Y, 180.0 - SPIKE_MARK_ANGLE_OFFSET),
        new TrcPose2D(BACKSTAGE_SPIKES_X, BLUE_SPIKES_Y, 180.0),
        new TrcPose2D(BACKSTAGE_SPIKES_X, BLUE_SPIKES_Y, 180.0 + SPIKE_MARK_ANGLE_OFFSET)
    };
    public static final int[] BLUE_BACKDROP_APRILTAGS           = new int[]{1, 2, 3};
    public static final int[] RED_BACKDROP_APRILTAGS            = new int[]{4, 5, 6};
    // AprilTag locations to place the pixel in inches.
    public static final double APRILTAG_BACKDROP_X              = 60.25;
    public static final double APRILTAG_AUDIENCE_WALL_X         = -70.25;
    // All AprilTags are at the height of 4.0-inch except for AprilTag 7 and 10 which are at the height of 5.5-inch.
    public static final TrcPose2D[] APRILTAG_POSES              = new TrcPose2D[] {
        new TrcPose2D(APRILTAG_BACKDROP_X, 41.41, 90.0),        // TagId 1
        new TrcPose2D(APRILTAG_BACKDROP_X, 35.41, 90.0),        // TagId 2
        new TrcPose2D(APRILTAG_BACKDROP_X, 29.41, 90.0),        // TagId 3
        new TrcPose2D(APRILTAG_BACKDROP_X, -29.41, 90.0),       // TagId 4
        new TrcPose2D(APRILTAG_BACKDROP_X, -35.41, 90.0),       // TagId 5
        new TrcPose2D(APRILTAG_BACKDROP_X, -41.41, 90.0),       // TagId 6
        new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -40.63, -90.0), // TagId 7
        new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, -35.13, -90.0), // TagId 8
        new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, 35.13, -90.0),  // TagId 9
        new TrcPose2D(APRILTAG_AUDIENCE_WALL_X, 40.63, -90.0)   // TagId 10
    };
    //
    // Vision subsystem.
    //
    public static final int CAM_IMAGE_WIDTH                     = 640;
    public static final int CAM_IMAGE_HEIGHT                    = 480;
    public static final OpenCvCameraRotation CAM_ORIENTATION    = OpenCvCameraRotation.UPRIGHT;
    // Camera location on robot.
    public static final double FRONTCAM_X_OFFSET                = 0.0;
    public static final double FRONTCAM_Y_OFFSET                = -(ROBOT_LENGTH/2.0 - 2.5);
    public static final double FRONTCAM_Z_OFFSET                = 9.75;
    public static final TrcPose2D FRONTCAM_POSE                 = new TrcPose2D(
        FRONTCAM_X_OFFSET, FRONTCAM_Y_OFFSET, FRONTCAM_Z_OFFSET);
//    // Camera: Micorosoft Lifecam HD 3000 v1/v2
//    public static final double WEBCAM_FX                        = 678.154;  // in pixels
//    public static final double WEBCAM_FY                        = 678.170;  // in pixels
//    public static final double WEBCAM_CX                        = 318.135;  // in pixels
//    public static final double WEBCAM_CY                        = 228.374;  // in pixels
//    // Camera: Logitech C270
//    public static final double WEBCAM_FX                        = 822.317;  // in pixels
//    public static final double WEBCAM_FY                        = 822.317;  // in pixels
//    public static final double WEBCAM_CX                        = 319.495;  // in pixels
//    public static final double WEBCAM_CY                        = 242.502;  // in pixels
//    // Camera: Logitech C310
//    public static final double WEBCAM_FX                        = 821.993;  // in pixels
//    public static final double WEBCAM_FY                        = 821.993;  // in pixels
//    public static final double WEBCAM_CX                        = 330.489;  // in pixels
//    public static final double WEBCAM_CY                        = 248.997;  // in pixels
//    // Camera: Logitech C920
//    public static final double WEBCAM_FX                        = 622.001;  // in pixels
//    public static final double WEBCAM_FY                        = 622.001;  // in pixels
//    public static final double WEBCAM_CX                        = 319.803;  // in pixels
//    public static final double WEBCAM_CY                        = 241.251;  // in pixels

    // Measurement unit: pixels
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y      = 120.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X     = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y     = 120.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X   = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y   = CAM_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X  = CAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y  = CAM_IMAGE_HEIGHT - 1;
    // Measurement unit: inches
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X       = -12.5625;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y       = 48.0 - ROBOT_LENGTH/2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X      = 11.4375;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y      = 44.75 - ROBOT_LENGTH/2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X    = -2.5625;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y    = 21.0 - ROBOT_LENGTH/2.0 - FRONTCAM_Y_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X   = 2.5626;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y   = 21.0 - ROBOT_LENGTH/2.0 - FRONTCAM_Y_OFFSET;

    public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
    public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
        RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotParams.HOMOGRAPHY_WORLD_TOPLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
        RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotParams.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);
    //
    // Motor Odometries.
    //
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_312_ENCODER_PPR     = (((1.0 + 46.0/17.0)*(1.0 + 46.0/11.0))*28.0);
    public static final double GOBILDA_5203_312_RPM             = 312.0;
    public static final double GOBILDA_5203_312_MAX_VELOCITY_PPS=
        GOBILDA_5203_312_ENCODER_PPR*GOBILDA_5203_312_RPM/60.0; // 2795.9872 pps
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
    public static final double GOBILDA_5203_435_ENCODER_PPR     = (((1.0 + 46.0/17.0)*(1.0 + 46.0/17.0))*28.0);
    public static final double GOBILDA_5203_435_RPM             = 435.0;
    public static final double GOBILDA_5203_435_MAX_VELOCITY_PPS=
        GOBILDA_5203_435_ENCODER_PPR*GOBILDA_5203_435_RPM/60.0; // 2787.9135 pps
    //
    // DriveBase subsystem.
    //
    public static DriveOrientation DEF_DRIVE_ORIENTATION        = DriveOrientation.FIELD;
    public static final double LFSTEER_ZERO_POS                 = 0.474812;
    public static final double RFSTEER_ZERO_POS                 = 0.467663;
    public static final double LBSTEER_ZERO_POS                 = 0.541338;
    public static final double RBSTEER_ZERO_POS                 = 0.545340;

    public static final boolean LFDRIVE_INVERTED                = true;
    public static final boolean RFDRIVE_INVERTED                = false;
    public static final boolean LBDRIVE_INVERTED                = true;
    public static final boolean RBDRIVE_INVERTED                = false;
    public static final boolean LFSTEER_INVERTED                = true;
    public static final boolean RFSTEER_INVERTED                = true;
    public static final boolean LBSTEER_INVERTED                = true;
    public static final boolean RBSTEER_INVERTED                = true;
    public static final double STEER_SERVO_KP                   = 0.01;
    public static final double STEER_SERVO_KI                   = 0.0;
    public static final double STEER_SERVO_KD                   = 0.0;
    public static final double STEER_SERVO_KF                   = 0.0;
    public static final double STEER_SERVO_IZONE                = 0.0;
    public static final double STEER_SERVO_TOLERANCE            = 0.5;

    public static final boolean DRIVE_WHEEL_BRAKE_MODE_ON       = true;
    public static final double TURN_POWER_LIMIT                 = 0.5;
    public static final double DRIVE_POWER_SCALE_SLOW           = 0.25;
    public static final double DRIVE_POWER_SCALE_NORMAL         = 1.0;
    public static final double TURN_POWER_SCALE_SLOW            = 0.5;
    public static final double TURN_POWER_SCALE_NORMAL          = 1.0;
    // Optii Odometry Wheel:
    public static final double ODWHEEL_DIAMETER                 = 35.0 * TrcUtil.INCHES_PER_MM;
    public static final double ODWHEEL_CPR                      = 4096.0;
    public static final double ODWHEEL_INCHES_PER_COUNT         = Math.PI*ODWHEEL_DIAMETER/ODWHEEL_CPR;
    // Scale = 0.00105687652708656383937269814237 inches/count
    public static final double YLEFT_ODWHEEL_X_OFFSET           = -144.0 * TrcUtil.INCHES_PER_MM;
    public static final double YLEFT_ODWHEEL_Y_OFFSET           = -12.0 * TrcUtil.INCHES_PER_MM;
    public static final double YRIGHT_ODWHEEL_X_OFFSET           = 144.0 * TrcUtil.INCHES_PER_MM;
    public static final double YRIGHT_ODWHEEL_Y_OFFSET          = -12.0 * TrcUtil.INCHES_PER_MM;
    public static final double X_ODWHEEL_X_OFFSET               = 0.0;
    public static final double X_ODWHEEL_Y_OFFSET               = -168.0 * TrcUtil.INCHES_PER_MM;
    public static final FtcGamepad.DriveMode ROBOT_DRIVE_MODE   = FtcGamepad.DriveMode.ARCADE_MODE;
    //
    // Velocity controlled constants.
    //
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS     = GOBILDA_5203_435_MAX_VELOCITY_PPS;

    public static final TrcPidController.PidCoefficients xPosPidCoeff =
        new TrcPidController.PidCoefficients(0.095, 0.0, 0.006, 0.0);
    public static final double XPOS_TOLERANCE                   = 1.0;
    public static final double XPOS_INCHES_PER_COUNT            = 0.01924724265461924299065420560748;
    public static final Double X_RAMP_RATE                      = null;//10.0;

    // 0.022, 0.0, 0.0018
    public static final TrcPidController.PidCoefficients yPosPidCoeff =
        new TrcPidController.PidCoefficients(0.035, 0.0, 0.0035, 0.0);
    public static final double YPOS_TOLERANCE                   = 1.0;
    public static final double YPOS_INCHES_PER_COUNT            = 0.02166184604662450653409090909091;
    public static final Double Y_RAMP_RATE                      = null;//10.0;

    public static final TrcPidController.PidCoefficients turnPidCoeff =
        new TrcPidController.PidCoefficients(0.012, 0.0, 0.0012, 0.0, 0.0);
    public static final double TURN_TOLERANCE                   = 2.0;
    public static final double TURN_SETTLING                    = TrcPidController.DEF_SETTLING_TIME;
    public static final double TURN_STEADY_STATE_ERR            = 2.0;
    public static final double TURN_STALL_ERRRATE_THRESHOLD     = 1.0;
    public static final Double TURN_RAMP_RATE                   = null;//10.0;
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    public static final double ROBOT_MAX_VELOCITY               = 72.0;     // measured maximum from drive speed test.
    public static final double ROBOT_MAX_ACCELERATION           = 530.0;    // measured maximum from drive speed test.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final TrcPidController.PidCoefficients velPidCoeff  =
        new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/ROBOT_MAX_VELOCITY);
    public static final double PPD_FOLLOWING_DISTANCE           = 6.0;
    public static final double PPD_POS_TOLERANCE                = 1.0;
    public static final double PPD_TURN_TOLERANCE               = 2.0;
    //
    // Elevator Subsystem.
    //
    // Actuator parameters.
    public static final boolean ELEVATOR_MOTOR_INVERTED         = false;
    public static final boolean ELEVATOR_HAS_LOWER_LIMIT_SWITCH = true;
    public static final boolean ELEVATOR_LOWER_LIMIT_INVERTED   = false;
    public static final boolean ELEVATOR_HAS_UPPER_LIMIT_SWITCH = false;
    public static final boolean ELEVATOR_UPPER_LIMIT_INVERTED   = false;
    public static final boolean ELEVATOR_VOLTAGE_COMP_ENABLED   = true;
    public static final double ELEVATOR_ENCODER_PPR             = GOBILDA_5203_312_ENCODER_PPR;
    public static final double ELEVATOR_PULLEY_DIAMETER         = 1.405;
    public static final double ELEVATOR_PULLEY_CIRCUMFERENCE    = Math.PI*ELEVATOR_PULLEY_DIAMETER;
    public static final double ELEVATOR_INCHES_PER_COUNT        = ELEVATOR_PULLEY_CIRCUMFERENCE/ELEVATOR_ENCODER_PPR;
    public static final double ELEVATOR_POWER_LIMIT             = 1.0;
    public static final double ELEVATOR_OFFSET                  = 11.6;             // in inches
    public static final double ELEVATOR_MIN_POS                 = ELEVATOR_OFFSET;
    public static final double ELEVATOR_MAX_POS                 = 24.0;
    public static final double ELEVATOR_SAFE_POS                = 14.0;
    public static final double ELEVATOR_LOAD_POS                = 11.8;
    public static final double ELEVATOR_LEVEL1_POS              = 14.5;
    public static final double ELEVATOR_LEVEL2_POS              = 21.5;
    public static final double ELEVATOR_LEVEL3_POS              = 21.5;             // Unreachable
    // Power settings.
    public static final double ELEVATOR_CAL_POWER               = -0.15;
    // Preset positions.
    public static final double ELEVATOR_PRESET_TOLERANCE        = 0.4;
    public static final double[] ELEVATOR_PRESETS               = new double[] {
//        14.0, 16.0, 20.0, 22.0
          ELEVATOR_LOAD_POS, ELEVATOR_LEVEL1_POS, ELEVATOR_LEVEL2_POS
//        ELEVATOR_MIN_POS,
//        ELEVATOR_SAFE_POS,
//        ELEVATOR_LEVEL1_POS,
//        ELEVATOR_LEVEL2_POS,
//        ELEVATOR_LEVEL3_POS,
//        ELEVATOR_MAX_POS
    };
    // PID Actuator parameters.
    public static final double ELEVATOR_KP                      = 0.5;
    public static final double ELEVATOR_KI                      = 0.0;
    public static final double ELEVATOR_KD                      = 0.025;
    public static final double ELEVATOR_KF                      = 0.0;
    public static final double ELEVATOR_TOLERANCE               = 2.0;
    public static final double ELEVATOR_IZONE                   = 10.0;
    //
    // Arm subsystem.
    //
    // Actuator parameters.
    public static final boolean ARM_MOTOR_INVERTED              = false;
    public static final boolean ARM_HAS_FOLLOWER_MOTOR          = true;
    public static final boolean ARM_FOLLOWER_MOTOR_INVERTED     = true;
    public static final boolean ARM_HAS_LOWER_LIMIT_SWITCH      = false;
    public static final boolean ARM_LOWER_LIMIT_INVERTED        = false;
    public static final boolean ARM_HAS_UPPER_LIMIT_SWITCH      = false;
    public static final boolean ARM_UPPER_LIMIT_INVERTED        = false;
    public static final boolean ARM_HAS_EXTERNAL_ENCODER        = true;
    public static final boolean ARM_ENCODER_INVERTED            = true;
    public static final boolean ARM_ENCODER_ABSOLUTE            = true;
    public static final boolean ARM_VOLTAGE_COMP_ENABLED        = true;
    public static final double ARM_DEG_SCALE                    = 360.0;
    public static final double ARM_POWER_LIMIT                  = 0.11;
    public static final double ARM_OFFSET                       = 27.0;
    public static final double ARM_ZERO_OFFSET                  = 0.949697;
    public static final double ARM_MIN_POS                      = 25.6;
    public static final double ARM_MAX_POS                      = 300.0;
    public static final double ARM_LOAD_POS                     = 25.6;
    public static final double ARM_SAFE_POS                     = 37.0;
    public static final double ARM_FREE_TO_MOVE_POS             = 50.0;
    public static final double ARM_SCORE_BACKDROP_POS           = 260.0;
    // Preset positions.
    public static final double ARM_PRESET_TOLERANCE             = 5.0;
    public static final double[] ARM_PRESETS                    = new double[] {
//        30.0, 60.0, 90.0, 120, 150.0, 180.0, 210.0, 240.0, 270.0
          ARM_LOAD_POS, ARM_SCORE_BACKDROP_POS
//        ARM_MIN_POS,
//        ARM_SAFE_POS,
//        ARM_FREE_TO_MOVE_POS,
//        ARM_SCORE_BACKDROP_POS,
//        ARM_MAX_POS
    };
    // PID Actuator parameters.
    public static final double ARM_KP                           = 0.011;
    public static final double ARM_KI                           = 0.0;
    public static final double ARM_KD                           = 0.0;
    public static final double ARM_KF                           = 0.0;
    public static final double ARM_IZONE                        = 0.0;
    public static final double ARM_TOLERANCE                    = 2.0;
    public static final double ARM_MAX_GRAVITY_COMP_POWER       = 0.12;
    //
    // Wrist subsystem.
    //
    public static final boolean WRIST_SERVO_INVERTED            = false;
    public static final boolean WRIST_HAS_FOLLOWER_SERVO        = true;
    public static final boolean WRIST_FOLLOWER_SERVO_INVERTED   = true;
    public static final double WRIST_MIN_POS                    = 0.0;
    public static final double WRIST_MAX_POS                    = 0.38;
    public static final double WRIST_DOWN_POS                   = 0.222;
    public static final double WRIST_UP_POS                     = WRIST_MAX_POS;
    //
    // Intake subsystem.
    //
    public static final double INTAKE_PICKUP_POWER              = -1.0;
    public static final double INTAKE_SPITOUT_POWER             = 0.7;
    public static final double INTAKE_SENSOR_THRESHOLD          = 5.0;
    //
    // Pixel Tray subsystem.
    //
    public static final double PIXELTRAY_LOWER_GATE_CLOSE       = 0.0;
    public static final double PIXELTRAY_LOWER_GATE_OPEN        = 0.25;
    public static final double PIXELTRAY_UPPER_GATE_CLOSE       = 0.3;
    public static final double PIXELTRAY_UPPER_GATE_OPEN        = 0.0;
    public static final double PIXELTRAY_OPEN_CLOSE_TIME        = 0.2;  // in sec
    //
    // Launcher subsystem.
    //
    public static final double LAUNCHER_POWER                   = 1.0;

}   //class RobotParams
