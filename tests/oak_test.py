#!/usr/bin/env python3
import argparse
import time
import depthai as dai
import cv2 as cv

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--w", type=int, default=800, help="requested RGB output width")
    ap.add_argument("--h", type=int, default=600, help="requested RGB output height")
    ap.add_argument("--mono-w", type=int, default=None, help="mono/stereo width (default: same as --w)")
    ap.add_argument("--mono-h", type=int, default=None, help="mono/stereo height (default: same as --h)")
    ap.add_argument("--nn-w", type=int, default=640, help="NN input width")
    ap.add_argument("--nn-h", type=int, default=640, help="NN input height")
    ap.add_argument("--shaves", type=int, default=None, help="attempt to set NN shave count (best-effort, depends on depthai version)")
    ap.add_argument("--fps", type=float, default=30.0, help="sensorFps to request")
    ap.add_argument("--no-fps-arg", action="store_true", help="don't pass sensorFps to build()")
    ap.add_argument("--undistort", action="store_true", help="enable enableUndistortion")
    ap.add_argument("--dual-output", action="store_true", help="also request a second 640x640 output from the same Camera, like the real pipeline does")
    ap.add_argument("--stereo", action="store_true", help="also build monoLeft/monoRight + StereoDepth like the real pipeline, to test if stereo is the bottleneck")
    ap.add_argument("--stereo-subpixel", action="store_true", help="enable setSubpixel(True) on stereo (only used with --stereo)")
    ap.add_argument("--stereo-distortion", action="store_true", help="enable enableDistortionCorrection(True) on stereo (only used with --stereo)")
    ap.add_argument("--nn", action="store_true", help="also build SpatialDetectionNetwork, properly linked (requires --stereo)")
    ap.add_argument("--blob", type=str, default="/root/ida_ws/vision/models/yolov8n.blob", help="path to NN blob")
    ap.add_argument("--num-classes", type=int, default=80)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--show", action="store_true", help="show frames with cv.imshow")
    ap.add_argument("--seconds", type=float, default=15.0, help="how long to run")
    args = ap.parse_args()

    mono_w = args.mono_w if args.mono_w is not None else args.w
    mono_h = args.mono_h if args.mono_h is not None else args.h

    pipeline = dai.Pipeline()

    if args.no_fps_arg:
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    else:
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, fps=args.fps)

    rgb_out = camRgb.requestOutput(
        (args.w, args.h),
        type=dai.ImgFrame.Type.BGR888p,
        enableUndistortion=args.undistort
    )

    q = rgb_out.createOutputQueue(maxSize=4, blocking=False)

    q2 = None
    nn_out = None
    if args.dual_output or args.nn:
        nn_out = camRgb.requestOutput(
            (args.nn_w, args.nn_h),
            type=dai.ImgFrame.Type.BGR888p,
            enableUndistortion=args.undistort
        )
        q2 = nn_out.createOutputQueue(maxSize=4, blocking=False)

    q3 = None
    stereo = None
    if args.stereo or args.nn:
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        left_out = monoLeft.requestOutput((mono_w, mono_h), type=dai.ImgFrame.Type.GRAY8)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        right_out = monoRight.requestOutput((mono_w, mono_h), type=dai.ImgFrame.Type.GRAY8)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setSubpixel(args.stereo_subpixel)
        stereo.enableDistortionCorrection(args.stereo_distortion)
        left_out.link(stereo.left)
        right_out.link(stereo.right)
        stereo.initialConfig.setConfidenceThreshold(95)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_3x3)

        if not args.nn:
            # only expose raw depth queue when NN isn't consuming it
            q3 = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

    q4 = None
    if args.nn:
        nn = pipeline.create(dai.node.SpatialDetectionNetwork)
        nn.setBlob(args.blob)
        nn.setConfidenceThreshold(args.conf)
        nn.setBoundingBoxScaleFactor(0.5)
        nn.setDepthLowerThreshold(100)
        nn.setDepthUpperThreshold(5000)
        nn.detectionParser.setRunOnHost(False)
        nn.input.setBlocking(False)
        nn.detectionParser.setSubtype("yolov8n")
        nn.detectionParser.setNumClasses(args.num_classes)
        nn.detectionParser.setCoordinateSize(4)
        nn.detectionParser.setIouThreshold(args.iou)
        nn.detectionParser.setConfidenceThreshold(args.conf)

        nn_out.link(nn.input)
        nn.passthrough.link(stereo.inputAlignTo)
        stereo.depth.link(nn.inputDepth)

        if args.shaves is not None:
            applied = False
            for method_name in ("setNumShavesPerInferenceThread", "setNumShaves", "setNumNCEPerInferenceThread"):
                method = getattr(nn, method_name, None)
                if method is not None:
                    try:
                        method(args.shaves)
                        print(f"[TEST] Applied shave count via {method_name}({args.shaves})")
                        applied = True
                        break
                    except Exception as e:
                        print(f"[TEST] {method_name} exists but failed: {e}")
            if not applied:
                print(f"[TEST] WARNING: could not find a working shave-setter method on this depthai version; "
                      f"--shaves {args.shaves} was NOT applied. Inspect dir(nn) to find the right call.")

        q4 = nn.out.createOutputQueue(maxSize=4, blocking=False)

    print(f"[TEST] Requested RGB=({args.w},{args.h}) mono=({mono_w},{mono_h}) nn_input=({args.nn_w},{args.nn_h}) "
          f"fps_arg={'none' if args.no_fps_arg else args.fps} "
          f"undistort={args.undistort} dual_output={args.dual_output} stereo={args.stereo} "
          f"stereo_subpixel={args.stereo_subpixel} stereo_distortion={args.stereo_distortion} nn={args.nn} "
          f"shaves={args.shaves}")

    pipeline.start()

    frame_count = 0
    t_start = None
    last_report = time.time()
    report_count = 0

    try:
        while pipeline.isRunning():
            in_frame = q.tryGet()
            if q2 is not None:
                q2.tryGet()  # drain second output so it doesn't back up, mimicking real pipeline consuming both
            if q3 is not None:
                q3.tryGet()  # drain stereo depth output
            if q4 is not None:
                q4.tryGet()  # drain NN detections output
            if in_frame is None:
                continue

            if t_start is None:
                t_start = time.time()
                print("[TEST] First frame received, starting timer.")

            frame_count += 1
            report_count += 1

            now = time.time()
            # print instantaneous fps every 2 seconds
            if now - last_report >= 2.0:
                inst_fps = report_count / (now - last_report)
                elapsed = now - t_start
                avg_fps = frame_count / elapsed if elapsed > 0 else 0
                print(f"[TEST] instantaneous={inst_fps:.2f} fps | average={avg_fps:.2f} fps | "
                      f"total_frames={frame_count} | elapsed={elapsed:.1f}s")
                last_report = now
                report_count = 0

            if args.show:
                cv.imshow("rgb_fps_test", in_frame.getCvFrame())
                if cv.waitKey(1) == ord('q'):
                    break

            if t_start is not None and (now - t_start) >= args.seconds:
                break

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - t_start if t_start else 0
        avg_fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"\n[TEST] DONE. total_frames={frame_count} elapsed={elapsed:.2f}s avg_fps={avg_fps:.2f}")
        if args.show:
            cv.destroyAllWindows()

if __name__ == "__main__":
    main()