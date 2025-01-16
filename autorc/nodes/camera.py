from io import BytesIO
import time
from autorc.nodes import Node
from skimage.transform import resize as skresize
from picamera2 import Picamera2
from picamera import PiCamera
import cv2
import numpy as np
import base64

'''
    Supported resolution
    160.0 x 120.0
    176.0 x 144.0
    320.0 x 240.0
    352.0 x 288.0
    640.0 x 480.0
    1024.0 x 768.0
    1280.0 x 1024.0
'''

class BaseWebCam(Node):
    '''
        USB webcam interface, get image from camera and update:
            jpeg for client streaming
            capture_size=(width, height); capture size
            jpeg_size=(width, height): size of jpeg frame
            numpy_size: (height, width) image array for deep learning
    '''

    def __init__(self, context,
                 outputs=('cam/image-jpeg', 'cam/image-np'),
                 capture_size=(320, 240),
                 jpeg_size=(160, 120), numpy_size=None,
                 framerate=20, disable_numpy_stream=False, **kwargs):
        super(BaseWebCam, self).__init__(
            context, outputs=outputs,
            process_rate=framerate, **kwargs)
        self.capture_size = capture_size
        self.jpeg_size = jpeg_size
        self.disable_numpy_stream = disable_numpy_stream
        if (isinstance(jpeg_size, (tuple, list)) and
                (capture_size[0] < jpeg_size[0] or
                    capture_size[1] < jpeg_size[1])):
            raise Exception('Capture size must larger than jpeg size')
        if (isinstance(numpy_size, (tuple, list)) and
                (capture_size[0] < numpy_size[1] or
                    capture_size[1] < numpy_size[0])):
            raise Exception('Capture size must larger than numpy size')
        if (isinstance(numpy_size, (tuple, list)) and
                numpy_size[0] > numpy_size[1]):
            self.logger.warn(
                'numpy_size(height, width) or (rows, cols): First index '
                'should smaller or equal second index')
        self.numpy_size = numpy_size

    def get_frame(self):
        raise Exception('Not yet implemented')

    def get_jpeg(self, frame):
        raise Exception('Not yet implemented')

    def get_np_array(self, frame):
        raise Exception('Not yet implemented')

    def process_loop(self):
        frame = self.get_frame()
        if frame is not None:
            jpeg = self.get_jpeg(frame)
            np_array = None
            if not self.disable_numpy_stream:
                np_array = self.get_np_array(frame)
            return jpeg, np_array

class PiCamera2(BaseWebCam):
    '''Raspberry Pi camera interface using Picamera2'''
    
    def __init__(self, context, size=(160, 120), framerate=20,
                 jpeg_quality=90, use_rgb=True, **kwargs):
        super(PiCamera, self).__init__(
            context, capture_size=size, jpeg_size=size,
            framerate=framerate, **kwargs)
        self.use_rgb = use_rgb
        self.jpeg_quality = jpeg_quality
        self.camera = None
        self.running = False

    def start_up(self):
        """Initialize picamera2"""
        self.camera = Picamera2()
        # Configure camera with capture size
        config = self.camera.create_preview_configuration(
            main={"size": self.capture_size})
        self.camera.configure(config)
        # Start camera
        self.camera.start()
        self.running = True
        self.encode_param = (int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality)
        time.sleep(1)  # Camera warm up

    def get_frame(self):
        """Get a frame from camera"""
        if not self.running:
            return None
        return self.camera.capture_array()

    def get_jpeg(self, frame):
        """Convert frame to JPEG"""
        if frame is not None:
            if (self.jpeg_size is not None and
                    self.jpeg_size != self.capture_size):
                frame = cv2.resize(frame, self.jpeg_size,
                                 cv2.INTER_LINEAR)
            ret, jpeg = cv2.imencode('.jpg', frame, self.encode_param)
            return jpeg.tobytes()

    def get_np_array(self, frame):
        """Get numpy array for processing"""
        new_frame = frame
        if self.numpy_size:
            new_frame = cv2.resize(
                frame, (self.numpy_size[1], self.numpy_size[0]),
                cv2.INTER_LINEAR)
        if self.use_rgb:
            new_frame = cv2.cvtColor(new_frame, cv2.COLOR_BGR2RGB)
        return new_frame

    def shutdown(self):
        """Clean up camera resources"""
        if self.camera:
            self.camera.stop()
            self.running = False

class PiCamera1(BaseWebCam):
    '''Raspberry Pi camera interface using picamera'''
    
    def __init__(self, context, size=(160, 120), framerate=20,
                 jpeg_quality=90, use_rgb=True, **kwargs):
        super(PiCamera, self).__init__(
            context, capture_size=size, jpeg_size=size,
            framerate=framerate, **kwargs)
        self.use_rgb = use_rgb
        self.jpeg_quality = jpeg_quality
        self.camera = None
        self.running = False
        self.stream = io.BytesIO()

    def start_up(self):
        """Initialize picamera"""
        self.camera = PiCam()
        # Configure camera
        self.camera.resolution = self.capture_size
        self.camera.framerate = self.framerate
        # Start camera
        self.camera.start_preview()
        self.running = True
        time.sleep(2)  # Camera warm-up

    def get_frame(self):
        """Get a frame from camera"""
        if not self.running:
            return None
        # Capture frame to stream
        self.stream.seek(0)
        self.camera.capture(self.stream, format='jpeg', use_video_port=True)
        # Convert to numpy array
        data = np.frombuffer(self.stream.getvalue(), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def get_jpeg(self, frame):
        """Convert frame to JPEG"""
        if frame is not None:
            if (self.jpeg_size is not None and
                    self.jpeg_size != self.capture_size):
                frame = cv2.resize(frame, self.jpeg_size,
                                 cv2.INTER_LINEAR)
            ret, jpeg = cv2.imencode('.jpg', frame, 
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            return jpeg.tobytes()

    def get_np_array(self, frame):
        """Get numpy array for processing"""
        new_frame = frame
        if self.numpy_size:
            new_frame = cv2.resize(
                frame, (self.numpy_size[1], self.numpy_size[0]),
                cv2.INTER_LINEAR)
        if self.use_rgb:
            new_frame = cv2.cvtColor(new_frame, cv2.COLOR_BGR2RGB)
        return new_frame

    def shutdown(self):
        """Clean up camera resources"""
        if self.camera:
            self.camera.stop_preview()
            self.camera.close()
            self.running = False

class CVWebCam(BaseWebCam):
    ''' USB webcam interface using open CV
    '''
    cam = None      # open CV cam instance

    def __init__(self, context, size=(160, 120), framerate=20,
                 capture_device=0, jpeg_quality=90,
                 use_rgb=True, **kwargs):
        super(CVWebCam, self).__init__(context, size=size, framerate=framerate,
                                       **kwargs)
        self.use_rgb = use_rgb
        self.framerate = framerate
        self.capture_device = capture_device
        self.jpeg_quality = jpeg_quality

    def start_up(self):
        import cv2
        self.cv2 = cv2
        self.cam = cv2.VideoCapture(self.capture_device)
        if self.capture_size:   # Not working or camera is not supported
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_size[0])
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_size[1])
        self.encode_param = (int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality)
        time.sleep(1)   # Camera warm up

    def get_frame(self):
        if self.cam:
            ret, frame = self.cam.read()
            return frame

    def get_jpeg(self, frame):
        if frame is not None:
            if (self.jpeg_size is not None and
                    self.jpeg_size != self.capture_size):
                frame = self.cv2.resize(frame, self.jpeg_size,
                                        self.cv2.INTER_LINEAR)
            ret, jpeg = self.cv2.imencode('.jpg', frame, self.encode_param)
            return jpeg.tostring()

    def get_np_array(self, frame):
        new_frame = frame
        if self.numpy_size:
            # CV resize use col, row (width, height)
            new_frame = self.cv2.resize(
                frame, (self.numpy_size[1], self.numpy_size[0]),
                self.cv2.INTER_LINEAR)
        if self.use_rgb:
            new_frame = self.cv2.cvtColor(new_frame, self.cv2.COLOR_BGR2RGB)
        return new_frame

    def shutdown(self):
        if self.cam:
            self.cam.release()


class PGWebCam(BaseWebCam):
    '''
        USB webcam interface using Pygame
    '''
    def __init__(self, context, size=(160, 120), framerate=20,
                 capture_device=None, jpeg_quality=90,
                 use_bgr=False, **kwargs):
        super(PGWebCam, self).__init__(context, size=size, framerate=framerate,
                                       **kwargs)
        self.use_bgr = use_bgr
        self.framerate = framerate
        self.capture_device = capture_device
        self.jpeg_quality = jpeg_quality

    def start_up(self):
        import pygame
        import pygame.camera
        import pygame.image
        import pygame.surfarray
        from PIL import Image
        self.pygame = pygame
        self.Image = Image
        pygame.init()
        pygame.camera.init()
        try:
            if not self.capture_device:
                cam_list = pygame.camera.list_cameras()
                self.logger.debug(cam_list)
                self.cam = pygame.camera.Camera(cam_list[0], self.capture_size,
                                                "RGB")
            else:
                self.cam = pygame.camera.Camera(
                    self.capture_device, self.capture_size, "RGB")
            self.cam.start()
        except Exception as e:
            raise Exception('Camera init error')
        self.blank_surface = pygame.surface.Surface(self.capture_size)
        self.surface = None    # To store image
        time.sleep(1)   # Camera warm up

    def get_frame(self):
        if self.cam and self.cam.query_image():
            self.surface = self.cam.get_image(self.blank_surface)
            # Since pygame cam switch width and height, we have to flip back
            # self.surface = self.pygame.transform.rotate(
            # self.pygame.transform.flip(self.surface, True, False), 90)
            # need resize?
            # pygame.transform.scale(self.surface, self.size)
            frame = self.pygame.surfarray.pixels3d(self.surface)
            # Since numpy rotate image 90 degree, need to use numpy to
            # image transpose back
            return np.transpose(frame, (1, 0, 2))   # HxWxC

    def get_jpeg(self, frame):
        if frame is not None:
            if self.jpeg_size and self.jpeg_size != self.capture_size:
                frame = skresize(
                    frame, (self[1], self.jpeg_size[0]),
                    mode='reflect', anti_aliasing=False)
            tmpfile = BytesIO()
            img = self.Image.fromarray(frame)
            img.save(tmpfile, format='jpeg')
            return tmpfile.getvalue()

    def get_np_array(self, frame):
        if self.numpy_size and (
            self.numpy_size[0] != self.capture_size[1] or
                self.numpy_size[1] != self.capture_size[0]):
            # HxWxD
            return skresize(frame, self.numpy_size, mode='reflect',
                            anti_aliasing=False)
        return frame

    def shutdown(self):
        if self.cam:
            self.cam.stop()
