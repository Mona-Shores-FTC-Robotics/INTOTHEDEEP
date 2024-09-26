# SparkFun OTOS Quickstart for Roadrunner 1.0

The SparkFun OTOS or Optical Tracking Odometry Sensor is an optical-based odometry sensor with an integrated IMU.
This repository allows teams to integrate it into Roadrunner as a drop-in replacement.

## Notes and Warnings
Ensure that your sensor is properly mounted 10mm above the ground using the directions on the product page.

The OTOS sensor is designed to ONLY work on official field tiles.
Ensure that all tuning is performed on them. 
(If you have nothing else to test on, it seems to also be able to track a hardwood floor as well.
However, tuning numbers will likely be different between them.)

The custom localization is implemented using the SparkFunOTOSDrive class, which *extends* MecanumDrive.
This means that all of RoadRunner's standard tuning should remain in MecanumDrive, but you should use SparkFunOTOSDrive
in your OpModes.

I eventually plan to PR this in some form once it's been more extensively tested.

## When things go wrong…
This quickstart has not been extensively tested, and you are likely to encounter bugs and issues. 
If this happens, or if there's anything you're confused about or don't understand, the best way to get help is
making a post in roadrunner-help on the FTC Discord with your MecanumDrive and SparkFunOTOSDrive attached and pinging me
(@j5155).
If you're certain that you've found a bug,
or you have a feature request, you may also make an issue in the Issues tab above. 

Do NOT make an issue on the official Roadrunner quickstart while you are using this one.

## Tuning

### Configure Hardware

Configure your drive motors in MecanumDrive
   as explained [here](https://rr.brott.dev/docs/v1-0/tuning/#drive-classes). 
Make sure to properly reverse them using MecanumDirectionDebugger by following the official docs.

Tuning currently also requires a properly configured Control Hub or Expansion Hub IMU in MecanumDrive. 
This will be fixed in the future, but for now make sure your hub orientation is properly defined.

Also, make sure to configure the OTOS in your hardware config. 
By default, SparkFunOTOSDrive will look for a sensor named sensor_otos,
but you can change this in SparkFunOTOSDrive line 70.

Note that, to mitigate an issue with the OTOS driver in SDK version 9.2,
you must currently configure the OTOS as "SparkFunOTOS Corrected" in your hardware config.
### Tune Scalars and Offsets
First, tune the Angular Scalar by running the OTOSAngularScalar OpMode and following the instructions. 
This will allow you to get the maximum accuracy from the OTOS IMU.

After you have tuned the angular scalar, the IMU will be accurate enough to tune the heading offset.
Run OTOSHeadingOffsetTuner and follow the instructions.

Next, tune the Linear Scalar using the LocalizationTest OpMode.
Again, ensure that you perform this tuning on field tiles so that the OTOS gives accurate data.
SparkFun's official instructions to do are as follows:
> To calibrate the linear scalar, move the
robot a known distance and measure the error; do this multiple times at
multiple speeds to get an average, then set the linear scalar to the
inverse of the error.
> For example, if you move the robot 100 inches and
the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971

Check out the [docs](https://rr.brott.dev/docs/v1-0/tuning/).

