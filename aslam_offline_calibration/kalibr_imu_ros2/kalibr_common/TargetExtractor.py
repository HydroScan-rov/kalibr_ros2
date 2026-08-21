import sm

import numpy as np
import multiprocessing
import pickle
import signal
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import copy
import cv2
import traceback


def _stamp_text(stamp):
    try:
        return "{0:.9f}".format(stamp.toSec())
    except (AttributeError, TypeError, ValueError):
        return str(stamp)


def _worker_exit_text(process):
    if process.exitcode is None:
        return "still running"
    if process.exitcode < 0:
        try:
            return "signal {0} ({1})".format(
                -process.exitcode, signal.Signals(-process.exitcode).name
            )
        except ValueError:
            return "signal {0}".format(-process.exitcode)
    return "exit code {0}".format(process.exitcode)

def multicoreExtractionWrapper(detector, taskq, resultq, clearImages, noTransformation):
    while True:
        task = taskq.get()
        if task is None:
            return
        idx = task[0]
        stamp = task[1]
        image = task[2]

        try:
            if noTransformation:
                success, obs = detector.findTargetNoTransformation(stamp, np.array(image))
            else:
                success, obs = detector.findTarget(stamp, np.array(image))

            if clearImages:
                obs.clearImage()
            payload = pickle.dumps(obs, protocol=pickle.HIGHEST_PROTOCOL) if success else None
            resultq.put(("result", idx, payload))
        except BaseException:
            resultq.put(("error", idx, _stamp_text(stamp), traceback.format_exc()))
            return

def extractCornersFromDataset(dataset, detector, multithreading=False, numProcesses=None, clearImages=True, noTransformation=False):
    print("Extracting calibration target corners")    
    targetObservations = []
    numImages = dataset.numImages()
    
    # prepare progess bar
    iProgress = sm.Progress2(numImages)
    iProgress.sample()
            
    if multithreading and numProcesses != 1:
        if not numProcesses:
            numProcesses = max(1,multiprocessing.cpu_count()-1)
        if numProcesses < 1:
            raise ValueError("numProcesses must be at least 1")
        manager = None
        taskq = None
        plist = []
        try:
            # Observations contain Boost.Python/numpy_eigen objects. Serialize
            # them in the worker and let the manager queue transport only bytes.
            # This avoids double conversion in the manager and prevents a worker
            # crash from corrupting the result channel used by the parent.
            manager = multiprocessing.Manager()
            resultq = manager.Queue()
            taskq = multiprocessing.Queue()

            for idx, (timestamp, image) in enumerate(dataset.readDataset()):
                taskq.put( (idx, timestamp, image) )
            for _ in range(numProcesses):
                taskq.put(None)
                
            collected = []
            for pidx in range(0, numProcesses):
                detector_copy = copy.copy(detector)
                p = multiprocessing.Process(target=multicoreExtractionWrapper, args=(detector_copy, taskq, resultq, clearImages, noTransformation, ))
                p.start()
                plist.append(p)

            completed = 0
            while completed < numImages:
                try:
                    message = resultq.get(timeout=0.5)
                except queue.Empty:
                    failed = [p for p in plist if p.exitcode not in (None, 0)]
                    if failed:
                        details = ", ".join(
                            "pid {0}: {1}".format(p.pid, _worker_exit_text(p))
                            for p in failed
                        )
                        raise RuntimeError("Corner extraction worker failed ({0})".format(details))
                    if all(not p.is_alive() for p in plist):
                        raise RuntimeError(
                            "Corner extraction workers exited after {0} of {1} images"
                            .format(completed, numImages)
                        )
                    continue

                kind = message[0]
                if kind == "error":
                    _, idx, stamp, details = message
                    raise RuntimeError(
                        "Corner extraction failed for image {0} at {1}s:\n{2}"
                        .format(idx, stamp, details.rstrip())
                    )
                if kind != "result":
                    raise RuntimeError("Unknown corner extraction result: {0}".format(kind))

                _, idx, payload = message
                if payload is not None:
                    collected.append((pickle.loads(payload), idx))
                completed += 1
                iProgress.sample()

            for p in plist:
                p.join()
            failed = [p for p in plist if p.exitcode != 0]
            if failed:
                details = ", ".join(
                    "pid {0}: {1}".format(p.pid, _worker_exit_text(p))
                    for p in failed
                )
                raise RuntimeError("Corner extraction worker failed ({0})".format(details))
        except Exception as e:
            raise RuntimeError("Exception during multithreaded extraction: {0}".format(e))
        finally:
            for p in plist:
                if p.is_alive():
                    p.terminate()
            for p in plist:
                p.join(timeout=5.0)
                if p.is_alive():
                    p.kill()
                    p.join()
            if taskq is not None:
                taskq.cancel_join_thread()
                taskq.close()
            if manager is not None:
                manager.shutdown()
        
        #get result sorted by time (=idx)
        if collected:
            sortedObs = sorted(collected, key=lambda tup: tup[1])
            targetObservations = [obs for obs, idx in sortedObs]
        else:
            targetObservations=[]
    
    #single threaded implementation
    else:
        for timestamp, image in dataset.readDataset():
            if noTransformation:
                success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
            else:
                success, observation = detector.findTarget(timestamp, np.array(image))
            if clearImages:
                observation.clearImage()
            if success == 1:
                targetObservations.append(observation)
            iProgress.sample()

    if len(targetObservations) == 0:
        print("\r")
        raise RuntimeError("No corners could be extracted for camera {0}! Check the calibration target configuration and dataset.".format(dataset.topic))
    else:    
        print("\r  Extracted corners for %d images (of %d images)                              " % (len(targetObservations), numImages))

    #close all opencv windows that might be open
    cv2.destroyAllWindows()
    
    return targetObservations
