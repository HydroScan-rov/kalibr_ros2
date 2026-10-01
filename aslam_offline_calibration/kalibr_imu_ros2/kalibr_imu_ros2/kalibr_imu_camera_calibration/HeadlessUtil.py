"""Numerical reports for the standalone calibration executable."""
import numpy as np
import sys
import json
import sm


def saveMetrics(calibrator, filename):
    """Write finite residual summaries with explicit physical units."""
    def summary(values):
        """Summarize residual magnitudes, rejecting empty or non-finite results."""
        values = np.asarray(values, dtype=float)
        if not values.size or not np.isfinite(values).all():
            raise RuntimeError("Calibration has missing or non-finite residuals")
        return {"count": int(values.size), "mean": float(np.mean(values)),
                "median": float(np.median(values)), "std": float(np.std(values)),
                "rmse": float(np.sqrt(np.mean(values ** 2)))}

    metrics = {"reprojection_px": [], "gyro_rad_s": [], "acceleration_m_s2": []}
    for camera in calibrator.CameraChain.camList:
        metrics["reprojection_px"].append(summary([np.linalg.norm(error.error())
            for observation in camera.allReprojectionErrors for error in observation]))
    for imu in calibrator.ImuList:
        metrics["gyro_rad_s"].append(summary([np.linalg.norm(error.error()) for error in imu.gyroErrors]))
        metrics["acceleration_m_s2"].append(summary([np.linalg.norm(error.error()) for error in imu.accelErrors]))
    with open(filename, "w") as handle:
        json.dump(metrics, handle, indent=2, allow_nan=False)
        handle.write("\n")

def printErrorStatistics(cself, dest=sys.stdout):
    """Write numerical calibration results without plotting."""
    print('Normalized Residuals\n----------------------------', file=dest)
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors) > 0:
            e2 = np.array([np.sqrt(rerr.evaluateError()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print('Reprojection error (cam{0}):     mean {1}, median {2}, std: {3}'.format(cidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        else:
            print('Reprojection error (cam{0}):     no corners'.format(cidx), file=dest)
    for iidx, imu in enumerate(cself.ImuList):
        e2 = np.array([np.sqrt(e.evaluateError()) for e in imu.gyroErrors])
        print('Gyroscope error (imu{0}):        mean {1}, median {2}, std: {3}'.format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        e2 = np.array([np.sqrt(e.evaluateError()) for e in imu.accelErrors])
        print('Accelerometer error (imu{0}):    mean {1}, median {2}, std: {3}'.format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
    print('', file=dest)
    print('Residuals\n----------------------------', file=dest)
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors) > 0:
            e2 = np.array([np.linalg.norm(rerr.error()) for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print('Reprojection error (cam{0}) [px]:     mean {1}, median {2}, std: {3}'.format(cidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        else:
            print('Reprojection error (cam{0}) [px]:     no corners'.format(cidx), file=dest)
    for iidx, imu in enumerate(cself.ImuList):
        e2 = np.array([np.linalg.norm(e.error()) for e in imu.gyroErrors])
        print('Gyroscope error (imu{0}) [rad/s]:     mean {1}, median {2}, std: {3}'.format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)
        e2 = np.array([np.linalg.norm(e.error()) for e in imu.accelErrors])
        print('Accelerometer error (imu{0}) [m/s^2]: mean {1}, median {2}, std: {3}'.format(iidx, np.mean(e2), np.median(e2), np.std(e2)), file=dest)

def printResults(cself, withCov=False):
    """Write numerical calibration results without plotting."""
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0, nCams):
        T_cam_b = cself.CameraChain.getResultTrafoImuToCam(camNr)
        print('')
        print('Transformation T_cam{0}_imu0 (imu0 to cam{0}, T_ci): '.format(camNr))
        if withCov and camNr == 0:
            print('    quaternion: ', T_cam_b.q(), ' +- ', cself.std_trafo_ic[0:3])
            print('    translation: ', T_cam_b.t(), ' +- ', cself.std_trafo_ic[3:])
        print(T_cam_b.T())
        if not cself.noTimeCalibration:
            print('')
            print('cam{0} to imu0 time: [s] (t_imu = t_cam + shift)'.format(camNr))
            print(cself.CameraChain.getResultTimeShift(camNr), end=' ')
            if withCov:
                print(' +- ', cself.std_times[camNr])
            else:
                print('')
    print('')
    for imuNr, imu in enumerate(cself.ImuList):
        print('IMU{0}:\n'.format(imuNr), '----------------------------')
        imu.getImuConfig().printDetails()

def saveResultTxt(cself, filename='cam_imu_result.txt'):
    """Write numerical calibration results without plotting."""
    with open(filename, 'w') as stream:
        printResultTxt(cself, stream=stream)


def printGravity(cself):
    """Write numerical calibration information without plotting."""
    print('')
    print('Gravity vector: (in target coordinates): [m/s^2]')
    print(cself.gravityDv.toEuclidean())


def printBaselines(cself):
    """Write numerical calibration information without plotting."""
    nCams = len(cself.CameraChain.camList)
    if nCams > 1:
        for camNr in range(0, nCams - 1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr + 1)
            if cself.CameraChain.camList[camNr + 1].T_extrinsic_fixed:
                isFixed = '(fixed to external data)'
            else:
                isFixed = ''
            print('')
            print('Baseline (cam{0} to cam{1}): [m] {2}'.format(camNr, camNr + 1, isFixed))
            print(T.T())
            print(baseline, '[m]')


def exportPoses(cself, filename='poses_imu0.csv'):
    """Write numerical calibration information without plotting."""
    f = open(filename, 'w')
    print('#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []', file=f)
    imu = cself.ImuList[0]
    bodyspline = cself.poseDv.spline()
    times = np.array([im.stamp.toSec() + imu.timeOffset for im in imu.imuData if im.stamp.toSec() + imu.timeOffset > bodyspline.t_min() and im.stamp.toSec() + imu.timeOffset < bodyspline.t_max()])
    for time in times:
        position = bodyspline.position(time)
        orientation = sm.r2quat(bodyspline.orientation(time))
        print('{:.0f},'.format(1000000000.0 * time) + ','.join(map('{:.6f}'.format, position)) + ',' + ','.join(map('{:.6f}'.format, orientation)), file=f)


def printResultTxt(cself, stream=sys.stdout):
    """Write numerical calibration information without plotting."""
    print('Calibration results', file=stream)
    print('===================', file=stream)
    printErrorStatistics(cself, stream)
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0, nCams):
        T = cself.CameraChain.getResultTrafoImuToCam(camNr)
        print('', file=stream)
        print('Transformation (cam{0}):'.format(camNr), file=stream)
        print('-----------------------', file=stream)
        print('T_ci:  (imu0 to cam{0}): '.format(camNr), file=stream)
        print(T.T(), file=stream)
        print('', file=stream)
        print('T_ic:  (cam{0} to imu0): '.format(camNr), file=stream)
        print(T.inverse().T(), file=stream)
        print('', file=stream)
        print('timeshift cam{0} to imu0: [s] (t_imu = t_cam + shift)'.format(camNr), file=stream)
        print(cself.CameraChain.getResultTimeShift(camNr), file=stream)
        print('', file=stream)
    if nCams > 1:
        print('Baselines:', file=stream)
        print('----------', file=stream)
        for camNr in range(0, nCams - 1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr + 1)
            print('Baseline (cam{0} to cam{1}): '.format(camNr, camNr + 1), file=stream)
            print(T.T(), file=stream)
            print('baseline norm: ', baseline, '[m]', file=stream)
            print('', file=stream)
    print('', file=stream)
    print('Gravity vector in target coords: [m/s^2]', file=stream)
    print(cself.gravityDv.toEuclidean(), file=stream)
    print('', file=stream)
    print('', file=stream)
    print('Calibration configuration', file=stream)
    print('=========================', file=stream)
    print('', file=stream)
    for camNr, cam in enumerate(cself.CameraChain.camList):
        print('cam{0}'.format(camNr), file=stream)
        print('-----', file=stream)
        cam.camConfig.printDetails(stream)
        cam.targetConfig.printDetails(stream)
        print('', file=stream)
    print('', file=stream)
    print('', file=stream)
    print('IMU configuration', file=stream)
    print('=================', file=stream)
    print('', file=stream)
    for imuNr, imu in enumerate(cself.ImuList):
        print('IMU{0}:\n'.format(imuNr), '----------------------------', file=stream)
        imu.getImuConfig().printDetails(stream)
        print('', file=stream)
