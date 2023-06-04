/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.DcMotor;

import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
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
        // Vision
        public static boolean useVuforia = false;
        public static boolean showVuforiaView = false;
        public static boolean useTensorFlow = false;
        public static boolean showTensorFlowView = false;
        public static boolean useEasyOpenCV = true;
        public static boolean showEasyOpenCvView = true;
        // Robot
        public static boolean noRobot = false;
        public static boolean swerveRobot = false;
        // Drive Base
        public static boolean useExternalOdometry = true;
        public static boolean useVelocityControl = false;
        // Subsystems
        public static boolean initSubsystems = true;
        public static boolean useTurret = true;
        public static boolean hasTurretSensor = true;
        public static boolean useElevator = true;
        public static boolean useArm = true;
        public static boolean useGrabber = true;
        public static boolean hasGrabberSensor = true;
    }   //class Preferences

    public static final String ROBOT_NAME                       = "Robot3543";
    public static final String TEAM_FOLDER_PATH                 =
        Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
    public static final String LOG_FOLDER_PATH                  = TEAM_FOLDER_PATH + "/tracelogs";
    public static final String STEERING_CALIBRATION_DATA_FILE   = "SteerCalibration.txt";
    //
    // Hardware names.
    //
    // Miscellaneous.
    public static final String HWNAME_IMU                       = "imu";
    public static final String HWNAME_WEBCAM                    = "Webcam 1";
    public static final String HWNAME_BLINKIN                   = "blinkin";
    // Drive Base.
    public static final String HWNAME_LFDRIVE_MOTOR             = "lfDriveMotor";
    public static final String HWNAME_RFDRIVE_MOTOR             = "rfDriveMotor";
    public static final String HWNAME_LBDRIVE_MOTOR             = "lbDriveMotor";
    public static final String HWNAME_RBDRIVE_MOTOR             = "rbDriveMotor";
    public static final String HWNAME_LFSTEER_SERVO1            = "lfSteerServo1";
    public static final String HWNAME_LFSTEER_SERVO2            = "lfSteerServo2";
    public static final String HWNAME_RFSTEER_SERVO1            = "rfSteerServo1";
    public static final String HWNAME_RFSTEER_SERVO2            = "rfSteerServo2";
    public static final String HWNAME_LBSTEER_SERVO1            = "lbSteerServo1";
    public static final String HWNAME_LBSTEER_SERVO2            = "lbSteerServo2";
    public static final String HWNAME_RBSTEER_SERVO1            = "rbSteerServo1";
    public static final String HWNAME_RBSTEER_SERVO2            = "rbSteerServo2";
    // Subsystems.
    public static final String HWNAME_TURRET                    = "turret";
    public static final String HWNAME_ELEVATOR                  = "elevator";
    public static final String HWNAME_ARM                       = "arm";
    public static final String HWNAME_GRABBER                   = "grabber";
    //
    // Field dimensions.
    //
    public static final double FULL_FIELD_INCHES                = 141.0;
    public static final double HALF_FIELD_INCHES                = FULL_FIELD_INCHES/2.0;
    public static final double FULL_TILE_INCHES                 = 23.75;
    //
    // Robot dimensions.
    //
    public static final double ROBOT_LENGTH                     = 17.375;
    public static final double ROBOT_WIDTH                      = 14.625;
    public static final double DRIVE_BASE_LENGTH                = 13.250;
    public static final double DRIVE_BASE_WIDTH                 = 12.375;
    //
    // Game positions.
    //
    public static final double STARTPOS_FROM_FIELDCENTER_X      = 1.5 * FULL_TILE_INCHES;
    public static final double STARTPOS_FROM_FIELDCENTER_Y      = HALF_FIELD_INCHES - ROBOT_LENGTH/2.0;
    // Robot start positions in inches.
    public static final TrcPose2D STARTPOS_RED_LEFT = new TrcPose2D(
        -STARTPOS_FROM_FIELDCENTER_X, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D STARTPOS_RED_RIGHT = new TrcPose2D(
        STARTPOS_FROM_FIELDCENTER_X, -STARTPOS_FROM_FIELDCENTER_Y, 0.0);
    public static final TrcPose2D STARTPOS_BLUE_LEFT = new TrcPose2D(
        STARTPOS_FROM_FIELDCENTER_X, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    public static final TrcPose2D STARTPOS_BLUE_RIGHT = new TrcPose2D(
        -STARTPOS_FROM_FIELDCENTER_X, STARTPOS_FROM_FIELDCENTER_Y, 180.0);
    // Robot park positions in tile units.
    public static final double PARKPOS_REDLEFT_P1_X             = -2.5;
    public static final double PARKPOS_REDLEFT_P2_X             = -1.5;
    public static final double PARKPOS_REDLEFT_P3_X             = -0.5;
    public static final double PARKPOS_REDFAR_Y                 = -0.5;
    public static final double PARKPOS_REDNEAR_Y                = -1.5;
    public static final TrcPose2D PARKPOS_REDLEFT_P1_FAR = new TrcPose2D(
        PARKPOS_REDLEFT_P1_X, PARKPOS_REDFAR_Y, -90.0);
    public static final TrcPose2D PARKPOS_REDLEFT_P1_NEAR = new TrcPose2D(
        PARKPOS_REDLEFT_P1_X, PARKPOS_REDNEAR_Y, -90.0);
    public static final TrcPose2D PARKPOS_REDLEFT_P2_FAR = new TrcPose2D(
        PARKPOS_REDLEFT_P2_X, PARKPOS_REDFAR_Y, -90.0);
    public static final TrcPose2D PARKPOS_REDLEFT_P2_NEAR = new TrcPose2D(
        PARKPOS_REDLEFT_P2_X, PARKPOS_REDNEAR_Y, -90.0);
    public static final TrcPose2D PARKPOS_REDLEFT_P3_FAR = new TrcPose2D(
        PARKPOS_REDLEFT_P3_X, PARKPOS_REDFAR_Y, -90.0);
    public static final TrcPose2D PARKPOS_REDLEFT_P3_NEAR = new TrcPose2D(
        PARKPOS_REDLEFT_P3_X, PARKPOS_REDNEAR_Y, -90.0);
    // Array of park positions indexed by signal value.
    public static final TrcPose2D[] PARKPOS_REDLEFT_NEAR = {
        PARKPOS_REDLEFT_P1_NEAR,
        PARKPOS_REDLEFT_P2_NEAR,
        PARKPOS_REDLEFT_P3_NEAR
    };
    public static final TrcPose2D[] PARKPOS_REDLEFT_FAR = {
        PARKPOS_REDLEFT_P1_FAR,
        PARKPOS_REDLEFT_P2_FAR,
        PARKPOS_REDLEFT_P3_FAR
    };

    // Auto-Assist navigation locations in tile units.
    public static final TrcPose2D AUTONAV_CONESTACK_RED_LEFT    = new TrcPose2D(-1.5, -0.5, -90.0);
    public static final TrcPose2D AUTONAV_CONESTACK_RED_RIGHT   = new TrcPose2D(1.5, -0.5, 90.0);
    public static final TrcPose2D AUTONAV_CONESTACK_BLUE_LEFT   = new TrcPose2D(1.5, 0.5, 90.0);
    public static final TrcPose2D AUTONAV_CONESTACK_BLUE_RIGHT  = new TrcPose2D(-1.5, 0.5, -90.0);

    public static final TrcPose2D AUTONAV_SUBSTATION_RED_LEFT   = new TrcPose2D(-0.5, -2.2, 180.0);
    public static final TrcPose2D AUTONAV_SUBSTATION_RED_RIGHT  = new TrcPose2D(0.5, -2.2, 180.0);
    public static final TrcPose2D AUTONAV_SUBSTATION_BLUE_LEFT  = new TrcPose2D(0.5, 2.2, 0.0);
    public static final TrcPose2D AUTONAV_SUBSTATION_BLUE_RIGHT = new TrcPose2D(-0.5, 2.2, 0.0);

    public static final TrcPose2D HIGHPOLE_SUBSTATION_RED       = new TrcPose2D (0.0, -1.0, 0.0);
    public static final TrcPose2D HIGHPOLE_SUBSTATION_BLUE      = new TrcPose2D (0.0, 1.0, 0.0);
    public static final TrcPose2D HIGHPOLE_AUDIENCE             = new TrcPose2D (-1.0, 0.0, 0.0);
    public static final TrcPose2D HIGHPOLE_REFEREE              = new TrcPose2D (1.0, 0.0, 0.0);

    public static final TrcPose2D[] AUTONAV_PICKUP_RED = {
        AUTONAV_SUBSTATION_RED_LEFT,
        AUTONAV_SUBSTATION_RED_RIGHT,
        AUTONAV_CONESTACK_RED_LEFT,
        AUTONAV_CONESTACK_RED_RIGHT
    };
    public static final TrcPose2D[] AUTONAV_PICKUP_BLUE = {
        AUTONAV_SUBSTATION_BLUE_LEFT,
        AUTONAV_SUBSTATION_BLUE_RIGHT,
        AUTONAV_CONESTACK_BLUE_LEFT,
        AUTONAV_CONESTACK_BLUE_RIGHT
    };
    public static final int AUTONAV_LEFT_SUBSTATION_INDEX       = 0;
    public static final int AUTONAV_RIGHT_SUBSTATION_INDEX      = 1;
    public static final int AUTONAV_LEFT_CONESTACK_INDEX        = 2;
    public static final int AUTONAV_RIGHT_CONESTACK_INDEX       = 3;

    public static final TrcPose2D[] AUTONAV_HIGHPOLES = {
        HIGHPOLE_SUBSTATION_RED,
        HIGHPOLE_SUBSTATION_BLUE,
        HIGHPOLE_AUDIENCE,
        HIGHPOLE_REFEREE
    };
    // In tile units.
    public static final TrcPose2D CONE_STACK_RED_LEFT           = new TrcPose2D(-2.4, -0.55, -90.0);
    public static final TrcPose2D LOOK_FOR_CONE_POS_REDLEFT     = new TrcPose2D(-2.0, -0.55, -90.0);
    //
    // Vision subsystem.
    //
    public static final int WEBCAM_PERMISSION_TIMEOUT           = 5000;     // in msec
    public static final int FRAME_QUEUE_CAPACITY                = 2;
    public static final double APRILTAG_SIZE                    = 0.043;// in meters
    public static final double APRILTAG_HEIGHT_OFFSET           = 1.5;  // in inches
    // Camera location on robot.
    public static final double WEBCAM_FRONT_OFFSET              = 2.000;//Camera offset from front of robot in inches
    public static final double WEBCAM_LEFT_OFFSET               = 7.125;//Camera offset from left of robot in inches
    public static final double WEBCAM_HEIGHT_OFFSET             = 3.750;//Camera offset from floor in inches
    public static final double WEBCAM_TILT_DOWN                 = 15.00;//Camera tilt down angle from horizontal in deg
    // Camera: Logitech C310
    public static final int WEBCAM_IMAGE_WIDTH                  = 640;
    public static final int WEBCAM_IMAGE_HEIGHT                 = 480;
    public static final double WEBCAM_FX                        = 821.993;  // in pixels
    public static final double WEBCAM_FY                        = 821.993;  // in pixels
    public static final double WEBCAM_CX                        = 330.489;  // in pixels
    public static final double WEBCAM_CY                        = 248.997;  // in pixels

    // Measurement unit: pixels
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X      = 0.0;
    public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y      = 120.0;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X     = WEBCAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y     = 120.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X   = 0.0;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y   = WEBCAM_IMAGE_HEIGHT - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X  = WEBCAM_IMAGE_WIDTH - 1;
    public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y  = WEBCAM_IMAGE_HEIGHT - 1;
    // Measurement unit: inches
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_X       = -12.5625;
    public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y       = 48.0 - ROBOT_LENGTH + WEBCAM_FRONT_OFFSET;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X      = 11.4375;
    public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y      = 44.75 - ROBOT_LENGTH + WEBCAM_FRONT_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X    = -2.5625;
    public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y    = 21.0 - ROBOT_LENGTH + WEBCAM_FRONT_OFFSET;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X   = 2.5626;
    public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y   = 21.0 - ROBOT_LENGTH + WEBCAM_FRONT_OFFSET;

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
    public static final double STEER_LOW_LIMIT                  = -90.0;
    public static final double STEER_HIGH_LIMIT                 = 90.0;
    public static final double LFSTEER_MINUS90                  = 0.20;
    public static final double LFSTEER_PLUS90                   = 0.85;
    public static final double RFSTEER_MINUS90                  = 0.21;
    public static final double RFSTEER_PLUS90                   = 0.87;
    public static final double LBSTEER_MINUS90                  = 0.26;
    public static final double LBSTEER_PLUS90                   = 0.94;
    public static final double RBSTEER_MINUS90                  = 0.18;
    public static final double RBSTEER_PLUS90                   = 0.85;

    public static final boolean LFDRIVE_INVERTED                = true;
    public static final boolean RFDRIVE_INVERTED                = true;
    public static final boolean LBDRIVE_INVERTED                = true;
    public static final boolean RBDRIVE_INVERTED                = true;
    public static final boolean LFSTEER_INVERTED                = false;
    public static final boolean RFSTEER_INVERTED                = false;
    public static final boolean LBSTEER_INVERTED                = false;
    public static final boolean RBSTEER_INVERTED                = false;

    public static final DcMotor.RunMode DRIVE_MOTOR_MODE        = DcMotor.RunMode.RUN_USING_ENCODER;
    public static final TrcPidController.PidCoefficients DRIVE_POSPID_COEFFS =
        new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0);
    public static final TrcPidController.PidCoefficients DRIVE_VELPID_COEFFS =
        new TrcPidController.PidCoefficients(1.0, 0.0, 0.0, 0.0);
    public static final boolean DRIVE_WHEEL_BRAKE_MODE_ON       = true;
    public static final boolean LEFT_WHEEL_INVERTED             = true;
    public static final boolean RIGHT_WHEEL_INVERTED            = false;
    public static final double TURN_POWER_LIMIT                 = 0.5;
    public static final double SLOW_DRIVE_POWER_SCALE           = 0.5;
    public static final double X_ODOMETRY_WHEEL_OFFSET          = ROBOT_LENGTH/2.0 - (3.875 + 9.5); //behind centroid
    public static final double Y_LEFT_ODOMETRY_WHEEL_OFFSET     = -15.25/2.0;
    public static final double Y_RIGHT_ODOMETRY_WHEEL_OFFSET    = 15.25/2.0;
    public static final FtcGamepad.DriveMode ROBOT_DRIVE_MODE   = FtcGamepad.DriveMode.ARCADE_MODE;
    //
    // Velocity controlled constants.
    //
    public static final double DRIVE_MOTOR_MAX_VELOCITY_PPS     = GOBILDA_5203_312_MAX_VELOCITY_PPS;

    public static final TrcPidController.PidCoefficients xPosPidCoeff =
        new TrcPidController.PidCoefficients(0.095, 0.0, 0.001, 0.0);
    public static final double XPOS_TOLERANCE                   = 1.0;
    public static final double XPOS_INCHES_PER_COUNT            = 0.01924724265461924299065420560748;
    public static final Double X_RAMP_RATE                      = null;//10.0;

    public static final TrcPidController.PidCoefficients yPosPidCoeff =
        new TrcPidController.PidCoefficients(0.06, 0.0, 0.002, 0.0);
    public static final double YPOS_TOLERANCE                   = 1.0;
    public static final double YPOS_INCHES_PER_COUNT            = 0.02166184604662450653409090909091;
    public static final Double Y_RAMP_RATE                      = null;//10.0;

    public static final TrcPidController.PidCoefficients turnPidCoeff =
        new TrcPidController.PidCoefficients(0.02, 0.08, 0.003, 0.0, 30.0);
    public static final double TURN_TOLERANCE                   = 1.0;
    public static final double TURN_SETTLING                    = TrcPidController.DEF_SETTLING_TIME;
    public static final double TURN_STEADY_STATE_ERR            = 2.0;
    public static final double TURN_STALL_ERRRATE_THRESHOLD     = 1.0;
    public static final Double TURN_RAMP_RATE                   = null;//10.0;

    public static final double X_ODWHEEL_INCHES_PER_COUNT       = 7.6150160901199168116026724971383e-4;
    public static final double Y_ODWHEEL_INCHES_PER_COUNT       = 7.6301149255006038191364659148717e-4;
    //
    // Pure Pursuit parameters.
    //
    // No-Load max velocity (i.e. theoretical maximum)
    // goBILDA 5203-312 motor, max shaft speed = 312 RPM
    // motor-to-wheel gear ratio = 1:1
    // max wheel speed = pi * wheel diameter * wheel gear ratio * motor RPM / 60.0
    // = 3.1415926535897932384626433832795 * 4 in. * 1.0 * 312.0 / 60.0
    // = 65.345127194667699360022982372214 in./sec.
    public static final double ROBOT_MAX_VELOCITY               = 23.0;     // measured maximum from drive speed test.
    public static final double ROBOT_MAX_ACCELERATION           = 3000.0;   // measured maximum from drive speed test.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    public static final TrcPidController.PidCoefficients velPidCoeff  =
        new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/ROBOT_MAX_VELOCITY);
    public static final double PPD_FOLLOWING_DISTANCE           = 6.0;
    public static final double PPD_POS_TOLERANCE                = 2.0;
    public static final double PPD_TURN_TOLERANCE               = 1.0;
    //
    // Grid Drive parameters
    //
    public static final double GRIDDRIVE_CELL_SIZE              = FULL_TILE_INCHES;
    //
    // Turret subsystem.
    //
    // Motor parameters.
    public static final boolean TURRET_MOTOR_INVERTED           = true;
    public static final boolean TURRET_HAS_LOWER_LIMIT_SWITCH   = true;
    public static final boolean TURRET_LOWER_LIMIT_INVERTED     = true;
    public static final boolean TURRET_HAS_UPPER_LIMIT_SWITCH   = false;
    public static final boolean TURRET_UPPER_LIMIT_INVERTED     = false;
    public static final boolean TURRET_DIR_SWITCH_INVERTED      = false;
    // PID Actuator parameters.
    public static final double TURRET_KP                        = 0.04;
    public static final double TURRET_KI                        = 0.0;
    public static final double TURRET_KD                        = 0.0;
    public static final double TURRET_TOLERANCE                 = 1.0;
    // Rev Core Hex Motor (https://www.revrobotics.com/rev-41-1300/)
    public static final double TURRET_ENCODER_PPR               = 288.0;
    public static final double TURRET_GEAR_RATIO                = 100.0/60.0;
    public static final double TURRET_DEG_PER_COUNT             = 360.0/(TURRET_ENCODER_PPR*TURRET_GEAR_RATIO);
    public static final double TURRET_OFFSET                    = 0.0;
    public static final double TURRET_MIN_POS                   = 0.0;
    public static final double TURRET_MAX_POS                   = 325.0;
    public static final double TURRET_STALL_MIN_POWER           = 0.5;
    public static final double TURRET_STALL_TOLERANCE           = 1.0;
    public static final double TURRET_STALL_TIMEOUT             = 1.0;
    public static final double TURRET_RESET_TIMEOUT             = 0.5;
    // Power settings.
    public static final double TURRET_CAL_POWER                 = -0.3;
    public static final double TURRET_POWER_SCALE_TELEOP        = 0.6;
    // Preset positions.
    public static final double TURRET_BACK                      = 0.0;
    public static final double TURRET_RIGHT                     = 90.0;
    public static final double TURRET_FRONT                     = 180.0;
    public static final double TURRET_LEFT                      = 270.0;
    public static final double TURRET_PRESET_TOLERANCE          = 10.0;
    public static final double[] TURRET_PRESET_LEVELS           = new double[] {
        TURRET_BACK,    // Slot 0
        TURRET_RIGHT,   // Slot 1
        TURRET_FRONT,   // Slot 2
        TURRET_LEFT     // Slot 3
    };
    // Distance sensor parameters.
    public static final double TURRET_SENSOR_LOWER_THRESHOLD    = 4.0;  // in inches
    public static final double TURRET_SENSOR_UPPER_THRESHOLD    = 12.0; // in inches
    public static final double TURRET_SENSOR_SETTLING_PERIOD    = 0.20; // in seconds
    // Auto-Assist Scoring parameters
    public static final double TURRET_SCAN_OFFSET               = 20.0;
    public static final double TURRET_SCAN_POWER                = 0.30;
    public static final double TURRET_TELEOP_SCAN_POWER         = 0.30;
    public static final double TURRET_SCAN_DURATION             = 5.0;
    //
    // Elevator Subsystem
    //
    // Motor parameters.
    public static final boolean ELEVATOR_MOTOR_INVERTED         = false;
    public static final boolean ELEVATOR_HAS_LOWER_LIMIT_SWITCH = true;
    public static final boolean ELEVATOR_LOWER_LIMIT_INVERTED   = false;
    public static final boolean ELEVATOR_HAS_UPPER_LIMIT_SWITCH = false;
    public static final boolean ELEVATOR_UPPER_LIMIT_INVERTED   = false;
    // PID Actuator parameters.
    public static final double ELEVATOR_KP                      = 0.30;
    public static final double ELEVATOR_KI                      = 0;//0.5;
    public static final double ELEVATOR_KD                      = 0.025;
    public static final double ELEVATOR_TOLERANCE               = 2.0;
    public static final double ELEVATOR_IZONE                   = 10.0;
    public static final double ELEVATOR_ENCODER_PPR             = GOBILDA_5203_435_ENCODER_PPR;
    public static final double ELEVATOR_PULLEY_DIAMETER         = 1.5*1.0465;       // in inches
    public static final double ELEVATOR_PULLEY_CIRCUMFERENCE    = Math.PI*ELEVATOR_PULLEY_DIAMETER;
    public static final double ELEVATOR_INCHES_PER_COUNT        =
    ELEVATOR_PULLEY_CIRCUMFERENCE/ELEVATOR_ENCODER_PPR;
    public static final double ELEVATOR_OFFSET                  = 7.8;              // in inches
    public static final double ELEVATOR_MIN_POS                 = ELEVATOR_OFFSET;
    public static final double ELEVATOR_MAX_POS                 = 34.0;
    public static final double ELEVATOR_STALL_MIN_POWER         = 0.75;
    public static final double ELEVATOR_STALL_TOLERANCE         = 0.0;
    public static final double ELEVATOR_STALL_TIMEOUT           = 1.0;
    public static final double ELEVATOR_RESET_TIMEOUT           = 0.5;
    // Power settings.
    public static final double ELEVATOR_CAL_POWER               = -0.1;
    public static final double ELEVATOR_POWER_COMPENSATION      = 0.1;
    public static final double ELEVATOR_DOWN_POWER_SCALE        = 0.3;
    // Preset positions.
    public static final double LOW_JUNCTION_SCORING_HEIGHT      = 10.8;
    public static final double MID_JUNCTION_SCORING_HEIGHT      = 21.0;
    public static final double HIGH_JUNCTION_SCORING_HEIGHT     = 32.5;
    public static final double ELEVATOR_PRESET_TOLERANCE        = 2.0;
    public static final double[] ELEVATOR_PRESET_LEVELS         = new double[] {
        ELEVATOR_MIN_POS,
        LOW_JUNCTION_SCORING_HEIGHT,
        MID_JUNCTION_SCORING_HEIGHT,
        HIGH_JUNCTION_SCORING_HEIGHT
    };
    //
    // Arm subsystem.
    //
    // Motor parameters.
    public static final boolean ARM_MOTOR_INVERTED              = false;
    public static final boolean ARM_HAS_LOWER_LIMIT_SWITCH      = true;
    public static final boolean ARM_LOWER_LIMIT_INVERTED        = false;
    public static final boolean ARM_HAS_UPPER_LIMIT_SWITCH      = false;
    public static final boolean ARM_UPPER_LIMIT_INVERTED        = false;
    // PID Actuator parameters.
    public static final double ARM_KP                           = 0.1;
    public static final double ARM_KI                           = 0.0;
    public static final double ARM_KD                           = 0.0;
    public static final double ARM_TOLERANCE                    = 0.5;
    public static final double ARM_ENCODER_PPR                  = GOBILDA_5203_435_ENCODER_PPR;
    // https://www.gobilda.com/super-duty-worm-drive-pan-kit-28-1-ratio/
    public static final double ARM_GEAR_RATIO                   = 28.0;
    public static final double ARM_DEG_PER_COUNT                = (360.0/(ARM_ENCODER_PPR*ARM_GEAR_RATIO));
    public static final double ARM_OFFSET                       = -8.0;
    // Arm Zero-Calibrated to Up Position. Arm Max Position is Down.
    public static final double ARM_MIN_POS                      = ARM_OFFSET;
    public static final double ARM_MAX_POS                      = 140.0;
    public static final double ARM_STALL_MIN_POWER              = 0.75;
    public static final double ARM_STALL_TOLERANCE              = 0.0;
    public static final double ARM_STALL_TIMEOUT                = 1.0;
    public static final double ARM_RESET_TIMEOUT                = 0.5;
    // Power settings.
    public static final double ARM_CAL_POWER                    = -0.5;
    // Preset positions.
    public static final double ARM_UP_POS                       = 0.0;
    public static final double ARM_SAFE_POS                     = 10.0;
    public static final double ARM_SINGLE_CONE_POS              = 135.0;
    public static final double ARM_STACK_2_POS                  = 120.0;
    public static final double ARM_STACK_3_CONES_POS            = 100.0;
    public static final double ARM_STACK_4_CONES_POS            = 90.0;
    public static final double ARM_STACK_5_CONES_POS            = 85.0;
    public static final double ARM_PRESET_TOLERANCE             = 2.0;
    // Index 0 is a placeholder so index 1 is 1 cone, 2 is 2 cones, etc.
    public static final double[] ARM_PICKUP_PRESETS             = new double[] {
        0.0,
        ARM_SINGLE_CONE_POS,
        ARM_STACK_2_POS,
        ARM_STACK_3_CONES_POS,
        ARM_STACK_4_CONES_POS,
        ARM_STACK_5_CONES_POS
    };
    // Parameters for calculating angle for aligning grabber on top of the pole.
    public static final double ARM_JOINT_LENGTH                 = 7.098;    // in inches
    public static final double ARM_ANGLE_OFFSET                 = -11.94;   // in degrees
    public static final double CLAW_DISTANCE_ADUSTMENT          = -2.7;    // in inches
    //
    // Grabber subsystem.
    //
    // Servo Grabber parameters.
    public static final double GRABBER_MAX_STEPRATE             = 1.0;
    public static final double GRABBER_MIN_POS                  = 0.0;
    public static final double GRABBER_MAX_POS                  = 0.2;
    public static final boolean GRABBER_LSERVO_INVERTED         = false;
    public static final boolean GRABBER_RSERVO_INVERTED         = true;
    public static final boolean GRABBER_TRIGGER_INVERTED        = true;
    public static final double GRABBER_TRIGGER_THRESHOLD        = 1.5;  // in inches
    public static final double GRABBER_HAS_OBJECT_THRESHOLD     = 2.5;  // in inches
    public static final double GRABBER_OPEN_POS                 = GRABBER_MAX_POS;
    public static final double GRABBER_OPEN_TIME                = 0.5;
    public static final double GRABBER_CLOSE_POS                = GRABBER_MIN_POS;
    public static final double GRABBER_CLOSE_TIME               = 0.5;
    public static final double GRABBER_DEF_CONE_DISTANCE        = 8.0;
    //
    // Miscellaneous.
    //
    public static final double CYCLE_TIME                       = 10.0; // in seconds
    public static final double PARK_TIME                        = 1.0;  // in seconds

}   //class RobotParams
