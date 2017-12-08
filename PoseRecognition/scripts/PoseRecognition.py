import PyOpenPose as OP
import time
import cv2
import sys
import numpy as np
import os
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame

try:
    from pylibfreenect2 import OpenCLPacketPipeline
    pipeline = OpenCLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenGLPacketPipeline
        pipeline = OpenGLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

enable_rgb = True
enable_depth = False

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

types = 0
if enable_rgb:
    types |= FrameType.Color
if enable_depth:
    types |= (FrameType.Ir | FrameType.Depth)
listener = SyncMultiFrameListener(types)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

if enable_rgb and enable_depth:
    device.start()
else:
    device.startStreams(rgb=enable_rgb, depth=enable_depth)

# NOTE: must be called after device.start()
if enable_depth:
    registration = Registration(device.getIrCameraParams(),
                                device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

OPENPOSE_ROOT = os.environ["OPENPOSE_ROOT"]


def showHeatmaps(hm):
    for idx, h in enumerate(hm):
        cv2.imshow("HeatMap "+str(idx), h)


def showPAFs(PAFs, startIdx=0, endIdx=16):
    allpafs = []
    for idx in range(startIdx, endIdx):
        X = PAFs[idx*2]
        Y = PAFs[idx*2+1]
        tmp = np.dstack((X, Y, np.zeros_like(X)))
        allpafs.append(tmp)

    pafs = np.mean(allpafs, axis=0)
    cv2.imshow("PAF", pafs)


def showTxt(res, text, x, y):
     cv2.putText(res, text, (x, y), 0, 1, (0, 0, 128))


def fallDetection(res, velocity, Ground, midPointCurr, leftHand, rightHand):
    #print "velocity: ", velocity/1000
    #print "Ground: ", Ground[1]
    #print "midPointCurr", midPointCurr[1]
    if velocity >= 1000:
        time.sleep(2)
        #print "111111"
        while Ground[1] - 200 <= midPointCurr[1] <= Ground[1] + 200 and Ground[1] != 0:
            print "falling detected"
            if confirm(leftHand, rightHand):
                print "falling confirmed and send message to emergency officer"
                break
            elif cancel(leftHand, rightHand):
                print "cancel"
                break


def confirm(leftHand, rightHand):
    conf = 1
    cancel = 0
    LeftFinger12X, LeftFinger12Y = leftHand[12, 0], leftHand[12, 1]
    RightFinger12X, RightFinger12Y = rightHand[12, 0], rightHand[12, 1]
    LeftFinger8X, LeftFinger8Y = leftHand[8, 0], leftHand[8, 1]
    RightFinger8X, RightFinger8Y = rightHand[8, 0], rightHand[8, 1]
    LeftFinger4X, LeftFinger4Y = leftHand[4, 0], leftHand[4, 1]
    RightFinger4X, RightFinger4Y = rightHand[4, 0], rightHand[4, 1]
    LeftFinger9X, LeftFinger9Y = leftHand[9, 0], leftHand[9, 1]
    RightFinger9X, RightFinger9Y = rightHand[9, 0], rightHand[9, 1]
    if RightFinger8X-20 <= RightFinger4X <= RightFinger8X+20 and RightFinger4X != 0:
        #showTxt(res, 'confirm and send message to emergency officer', 80, 80)
        return conf
    #elif RightFinger9X-10 <= RightFinger12X <= RightFinger9X+10:
        #showTxt(res, 'cancel', 80, 80)
        #return cancel
    if LeftFinger8X-20 <= LeftFinger4X <= LeftFinger8X+20 and LeftFinger4X != 0:
        #showTxt(res, 'confirm and send message to emergency officer', 80, 80)
        return conf


def cancel(leftHand, rightHand):
    canc =1
    LeftFinger12X, LeftFinger12Y = leftHand[12, 0], leftHand[12, 1]
    RightFinger12X, RightFinger12Y = rightHand[12, 0], rightHand[12, 1]
    LeftFinger8X, LeftFinger8Y = leftHand[8, 0], leftHand[8, 1]
    RightFinger8X, RightFinger8Y = rightHand[8, 0], rightHand[8, 1]
    LeftFinger4X, LeftFinger4Y = leftHand[4, 0], leftHand[4, 1]
    RightFinger4X, RightFinger4Y = rightHand[4, 0], rightHand[4, 1]
    LeftFinger9X, LeftFinger9Y = leftHand[9, 0], leftHand[9, 1]
    RightFinger9X, RightFinger9Y = rightHand[9, 0], rightHand[9, 1]
    if RightFinger9X-10 <= RightFinger12X <= RightFinger9X+10 and RightFinger12X != 0:
        return canc
    if LeftFinger9X-10 <= LeftFinger12X <= LeftFinger9X+10 and LeftFinger12X != 0:
        return canc

def run():
    m = 1
    t1 = 0.33
    download_heatmaps = False
    # with_face = with_hands = False
    with_face = False
    with_hands = True

    op = OP.OpenPose((656, 368), (368, 368), (1920, 1080), "COCO", OPENPOSE_ROOT + os.sep + "models" + os.sep, 0, download_heatmaps, OP.OpenPose.ScaleMode.ZeroToOne, with_face, with_hands)
    #op = OP.OpenPose((320, 240), (240, 240), (640, 480), "COCO", OPENPOSE_ROOT + os.sep + "models" + os.sep, 0, download_heatmaps)
    #op = OP.OpenPose((320, 240), (240, 240), (1920, 1080), "COCO", OPENPOSE_ROOT + os.sep + "models" + os.sep, 0, download_heatmaps, OP.OpenPose.ScaleMode.ZeroToOne, with_face, with_hands)
    #op = OP.OpenPose((320, 240), (240, 240), (1920, 1080), "COCO", OPENPOSE_ROOT + os.sep + "models" + os.sep, 0, with_hands)
    actual_fps = 0
    paused = False
    delay = {True: 0, False: 1}
    #fgbg = cv2.createBackgroundSubtractorMOG2()
    print "Entering main Loop."
    while True:
        start_time = time.time()
        try:
		frames = listener.waitForNewFrame()
		color = frames["color"]
		tmp = color.asarray()
		imgSize = list(color.asarray().shape)	
		outSize = imgSize[1::-1]
		rgb = tmp[:, :outSize[0], 0:3]
        except Exception as e:
            print "Failed to grab", e
            break
        t = time.time()
        op.detectPose(rgb)
        #op.detectFace(rgb)
        op.detectHands(rgb)
        t = time.time() - t
        op_fps = 1.0 / t
        res = op.render(rgb)
        cv2.putText(res, 'UI FPS = %f, OP FPS = %f' % (actual_fps, op_fps), (20, 20), 0, 0.5, (0, 0, 255))
        persons = op.getKeypoints(op.KeypointType.POSE)[0]
        rightTemp = op.getKeypoints(op.KeypointType.HAND)[0]
        leftTemp = op.getKeypoints(op.KeypointType.HAND)[1]
        if persons is not None and rightTemp is not None and leftTemp is not None:
			if persons.ndim >= 3 and rightTemp.ndim >= 3 and leftTemp.ndim >= 3:
				persons = np.reshape(persons, (-1, 3))
				leftHand = np.reshape(leftTemp, (-1, 3))
				rightHand = np.reshape(rightTemp, (-1, 3))
			#print "test data: ", leftTemp.shape
			#print "image size: ", res.shape
			#print "hand data: ", leftHand[0,:]
			#print "persons data", persons.shape
			#if persons is not None and len(persons) > 0:
			#print "First Person: ", persons[0].shape
		####
			LeftFinger12X, LeftFinger12Y = leftHand[12, 0], leftHand[12, 1]
			RightFinger12X, RightFinger12Y = rightHand[12, 0], rightHand[12, 1]
			LeftFinger0X, LeftFinger0Y = leftHand[0, 0], leftHand[0, 1]
			RightFinger0X, RightFinger0Y = rightHand[0, 0], rightHand[0, 1]
			Ground = [(persons[10, 0] + persons[13, 0])/2, (persons[10, 1] + persons[13, 1])/2]
			# compute velocity of mid torso point movement
			if m == 1:
				midPointPre = [(persons[8,0] + persons[11, 0])/2, (persons[8, 1] + persons[11, 1])/2]
			elif m == 10:
				midPointCurr = [(persons[8,0] + persons[11, 0])/2, (persons[8, 1] + persons[11, 1])/2]
				velocity = np.sqrt((midPointCurr[0] - midPointPre[0])**2 + (midPointCurr[1] - midPointPre[1])**2)/(t1)
				# fall detection
				fallDetection(res, velocity, Ground, midPointCurr, leftHand, rightHand)
				m = 1

			# headache
			LeftHeadX, LeftHeadY = persons[16, 0], persons[16, 1]
			RightHeadX, RightHeadY = persons[17, 0], persons[17, 1]
			#print  LeftHeadX, LeftHeadY
			#print "left", LeftFinger0X, LeftFinger0Y
			#print "right", LeftFinger0X, RightFinger0Y
			if (LeftHeadX-30 <= LeftFinger0X <= LeftHeadX+30 and LeftHeadY-30 <= LeftFinger0Y <= LeftHeadY+30) or (RightHeadX-30 <= RightFinger0X <= RightHeadX+30 and RightHeadY-30<= RightFinger0Y <= RightHeadY+30):
				if LeftFinger0X != 0 and RightFinger0X != 0 and LeftHeadX != 0 or RightHeadX != 0:
					print "headache"
					if confirm(leftHand, rightHand):
						print "headache confirm and send message to emergency officer"
					elif cancel(leftHand, rightHand):
						print "cancel"

			# heart attack
			heart = [(persons[2, 0]+persons[1, 0])/2, (persons[2, 1]+persons[1, 1])/2 + 100]
			#print "heart", heart
			if (heart[0]-30 <= LeftFinger0X <= heart[0]+30 and heart[1]-40 <= LeftFinger0Y <= heart[1]+40) or (heart[0]-30 <= RightFinger0X <= heart[0]+30 and heart[1]-40 <= RightFinger0Y <= heart[1]+40):
				if LeftFinger0X != 0 and RightFinger0X != 0 and heart[0] != 0:
					print "heart attack"
					if confirm(leftHand, rightHand):
						print "heart attack confirmed and send message to emergency officer"
					elif cancel(leftHand, rightHand):
						print "cancel"

			# stomach
			stomach = [(persons[8, 0]+persons[11,0])/2, (persons[11, 1]+persons[8,1])/2 - 200]
			#print "stomach: ", stomach
			#print "lefthand:",leftHand[0,1]
			#print "righthand: ", rightHand[0,1] 
			if (stomach[0]-20 <= LeftFinger0X <= stomach[0]+20 and stomach[1]-20 <= LeftFinger0Y <= stomach[1]+20) or (heart[0]-20 <= RightFinger0X <= stomach[0]+20 and stomach[1]-20 <= RightFinger0Y <= stomach[1]+20):
				if LeftFinger0X != 0 and RightFinger0X != 0 and stomach[0] != 0:				
					print "stomachache"
					if confirm(leftHand, rightHand):
						print "stomachache confirmed and send message to emergency officer"
					elif cancel(leftHand, rightHand):
						print "cancel"

			# left knee pain
			leftKnee = [persons[9, 0], persons[9, 1]]
			#print "leftknee", leftKnee
			if (leftKnee[0]-20 <= LeftFinger0X <= leftKnee[0]+20 and leftKnee[1]-60 <= LeftFinger0Y <= leftKnee[1]+60) or (leftKnee[0]-20 <= RightFinger0X <= leftKnee[0]+20 and leftKnee[1]-60 <= RightFinger0Y <= leftKnee[1]+60):
				if LeftFinger0X != 0 and RightFinger0X != 0 and leftKnee[0] != 0:
					print "left knee pain"
					if confirm(leftHand, rightHand):
						print "left knee pain confirmed and send message to emergency officer"
					elif cancel(leftHand, rightHand):
						print "cancel"

			# right knee pain
			rightKnee = [persons[12, 0], persons[12, 1]]
			#print "Rightknee", rightKnee
			if (rightKnee[0]-20 <= LeftFinger0X <= rightKnee[0]+20 and rightKnee[1]-60 <= LeftFinger0Y <= rightKnee[1]+60) or (rightKnee[0]-20 <= RightFinger0X <= rightKnee[0]+20 and rightKnee[1]-60 <= RightFinger0Y <= rightKnee[1]+60):
				if LeftFinger0X != 0 and RightFinger0X != 0 and rightKnee[0] != 0:
					print "right knee pain"
					if confirm(leftHand, rightHand):
						print "right knee pain confirmed and send message to emergency officer"
					elif cancel(leftHand, rightHand):
						print "cancel"
        cv2.imshow("OpenPose result", res)
        listener.release(frames)
        m = m + 1
        key = cv2.waitKey(delay[paused])
        if key & 255 == ord('p'):
            paused = not paused
        if key & 255 == ord('q'):
            device.stop()
            device.close()
            sys.exit(0)
            break
        actual_fps = 1.0 / (time.time() - start_time)
        #fgmask = fgbg.apply(rgb)
		#print "fgmask", fgmask.shape
  

if __name__ == '__main__':
    run()
