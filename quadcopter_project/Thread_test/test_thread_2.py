class Hello5Program:  
    def __init__(self):
        self._running = True
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.camera.vflip = True
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        self.tracker = cv2.TrackerTLD_create()
        self.initBB = None

    def terminate(self):  
        self._running = False 
    def run(self):
        global cycle
        while self._running:
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                W  = 640   # float
                H = 480  # float
                
                if self.initBB is not None:
                    # grab the new bounding box coordinates of the object
                    (success, box) = self.tracker.update(image)

                    # check to see if the tracking was a success
                    if success:
                        (x, y, w, h) = [int(v) for v in box]
                        cv2.rectangle(image, (x, y), (x + w, y + h),
                            (0, 255, 0), 2)


                # show the output frame
            #     fps = cap.get(cv2.CAP_PROP_FPS)
            #     print('fps:', fps)  # float
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF
#                 print(time.time()-end)
#                 end = time.time()

                # if the 's' key is selected, we are going to "select" a bounding
                # box to track
                if key == ord("s"):
                    # select the bounding box of the object we want to track (make
                    # sure you press ENTER or SPACE after selecting the ROI)
                    self.initBB = cv2.selectROI("Frame", image, fromCenter=False,
                        showCrosshair=True)

                    # start OpenCV object tracker using the supplied bounding box
                    # coordinates, then start the FPS throughput estimator as well
                    self.tracker.init(image, self.initBB)

                # if the `q` key was pressed, break from the loop
                elif key == ord("q"):
                    break
                # show the frame
#                 cv2.imshow("Frame", image)

                # clear the stream in preparation for the next frame
                self.rawCapture.truncate(0)

#Create Class
FiveSecond = Hello5Program()
#Create Thread
FiveSecondThread = Thread(target=FiveSecond.run) 
#Start Thread 
FiveSecondThread.start()