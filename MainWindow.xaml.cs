namespace KinectCoordinateMapping
{
    using Microsoft.Kinect;
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;
    using System.Threading.Tasks;
    using System.Threading;
    using System.Windows;
    using System.Drawing;
    using System.Windows.Controls;
    using System.Windows.Data;
    using System.Windows.Documents;
    using System.Windows.Input;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Navigation;
    using System.Windows.Shapes;
    using System.IO.Ports;

	using Robot;
    using Kinect;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        SerialPort port;

        private const int WIDTH = 1920;
        private const int HEIGHT = 1080;
        private const byte DRIVE = 137;
        private const byte CONTROL = 132;
        private const byte START = 128;
        private const int MAX_ROTATIONS = 30;
        private const int STABILIZE_TIME = 2000;

        private const int MAP_DEPTH_TO_BYTE = 8000 / 256;

		private RobotController robot;
        private KinectReader kinect;
        private KinectSensor kinectSensor = null;
        private DepthFrameReader depthFrameReader = null;
        private FrameDescription depthFrameDescription = null;
        private WriteableBitmap depthBitmap = null;
        private WriteableBitmap colorBitmap = null;
        private FrameDescription colorFrameDescription = null;
        private CoordinateMapper cm = null;
        private ushort[] depthPixels = null;
        private byte[] depthBytes = null;
        private byte[] colorBytes = null;
        private bool takeScreenshot = false;
        private bool dumpPpms = false;
        private byte[] colors = null;
        private ushort[] depths = null;
        private DepthSpacePoint[] mappedColor = null;
        private int panoramaNum = 0;
        private int imageNum = 0;
        private bool capturePanorama = false;
        private int numRotations = 0;
        private int moveTime = 400;
        private int rotateTime = 200;

        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

			this.robot = new RobotController("COM9");
            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // allocate space to put the pixels being received and converted
            this.depthPixels = new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray16, null);
            this.colorBitmap = new WriteableBitmap(this.colorFrameDescription.Width, this.colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.kinect = new KinectReader(depthBitmap, colorBitmap);

            // open the sensor
            this.kinectSensor.Open();
            this.cm = this.kinectSensor.CoordinateMapper;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
            this.imageNum = 0;

            Console.Write(kinectSensor.ColorFrameSource.FrameDescription.Width);
            Console.Write("x");
            Console.WriteLine(kinectSensor.ColorFrameSource.FrameDescription.Height);
            Console.Write(kinectSensor.ColorFrameSource.FrameDescription.HorizontalFieldOfView);
            Console.Write("x");
            Console.WriteLine(kinectSensor.ColorFrameSource.FrameDescription.VerticalFieldOfView);

            Console.WriteLine("Depth: " +
                kinectSensor.DepthFrameSource.FrameDescription.Width + " x " +
                kinectSensor.DepthFrameSource.FrameDescription.Height);
            Console.WriteLine("Depth FoV: " +
                kinectSensor.DepthFrameSource.FrameDescription.HorizontalFieldOfView + " x " +
                kinectSensor.DepthFrameSource.FrameDescription.VerticalFieldOfView);

            while (true)
            {
                CameraIntrinsics ci = kinectSensor.CoordinateMapper.GetDepthCameraIntrinsics();
                if (ci.FocalLengthX == 0 && kinectSensor.IsAvailable)
                {
                    Thread.Sleep(1000);
                }
                else
                {
                    Console.Write("Center: ");
                    Console.Write(ci.PrincipalPointX);
                    Console.Write(", ");
                    Console.WriteLine(ci.PrincipalPointY);

                    Console.WriteLine("Focal lengths " + ci.FocalLengthX.ToString() + " x " + ci.FocalLengthY.ToString());
                    Console.WriteLine("Distortion (2nd, 4th, 6th orders)" +
                        ci.RadialDistortionSecondOrder + " " +
                        ci.RadialDistortionFourthOrder + " " +
                        ci.RadialDistortionSixthOrder);

                    FileStream fs = new FileStream("Depth2Color.bin", FileMode.Create);
                    BinaryWriter bw = new BinaryWriter(fs);
                    PointF[] depth2Color = kinectSensor.CoordinateMapper.GetDepthFrameToCameraSpaceTable();
                    for (int i = 0; i < depth2Color.Length; i++)
                    {
                        bw.Write(depth2Color[i].X);
                        bw.Write(depth2Color[i].Y);
                    }
                    bw.Close();
                    fs.Close();

                    break;
                }
            }
        }

        /*
         * Stops moving after a given amount of milliseconds
         */
        private void StopMoving(int after)
        {
            if (after > 0)
            {
                Thread.Sleep(after);
            }
			robot.StopMoving();
        }

        private void ForwardButton_Click(object sender, RoutedEventArgs e)
        {
			robot.MoveForward();
            StopMoving(moveTime);
        }

        private void BackwardButton_Click(object sender, RoutedEventArgs e)
        {
			robot.MoveBackward();
            StopMoving(moveTime);
        }

        private void RotateCW_Click(object sender, RoutedEventArgs e)
        {
			robot.RotateClockwise();
            StopMoving(rotateTime);
        }

        private void CapturePanoramas_Click(object sender, RoutedEventArgs e)
        {
            this.capturePanorama = true;
            this.takeScreenshot = true;
        }

        private void RotateCW()
        {
			robot.RotateCounterclockwise();
        }

        private void RotateCCW_Click(object sender, RoutedEventArgs e)
        {
			robot.RotateCounterclockwise();
            StopMoving(rotateTime);
        }

        private void RotateCCW()
        {
			robot.RotateCounterclockwise();
		}

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
                depthCamera.Source = this.depthBitmap;
                colorCamera.Source = this.colorBitmap;
            }
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.Key)
            {
                case Key.Left:
                    robot.RotateCounterclockwise();
                    break;
                case Key.Right:
					robot.RotateClockwise();
                    break;
                case Key.Up:
				    robot.MoveForward();
                    break;
                case Key.Down:
				    robot.MoveBackward();
                    break;
            }
        }

        private void Window_KeyUp(object sender, KeyEventArgs e)
        {
            StopMoving(0);
        }

        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            // Color
            using (var colorFrame = reference.ColorFrameReference.AcquireFrame())
            {
                // Depth
                using (var depthFrame = reference.DepthFrameReference.AcquireFrame())
                {
                    if (colorFrame != null && depthFrame != null)
                    {
                        var _colorWidth = colorFrame.ColorFrameSource.FrameDescription.Width;
                        var _colorHeight = colorFrame.ColorFrameSource.FrameDescription.Height;
                        var _depthWidth = depthFrame.DepthFrameSource.FrameDescription.Width;
                        var _depthHeight = depthFrame.DepthFrameSource.FrameDescription.Height;

                        using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                        {
                            // verify data and write the color data to the display bitmap
                            if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                                (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                            {
                                // Note: In order to see the full range of depth (including the less reliable far field depth)
                                // we are setting maxDepth to the extreme potential depth threshold
                                ushort maxDepth = ushort.MaxValue;

                                // If you wish to filter by reliable depth distance, uncomment the following line:
                                //// maxDepth = depthFrame.DepthMaxReliableDistance

                                this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                                this.RenderDepthPixels();
                            }
                        }

                        using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                        {
                            this.colorBitmap.Lock();
                            // verify data and write the new color frame data to the display bitmap
                            if ((_colorWidth == this.colorBitmap.PixelWidth) && (_colorHeight == this.colorBitmap.PixelHeight))
                            {
                                colorFrame.CopyConvertedFrameDataToIntPtr(
                                    this.colorBitmap.BackBuffer,
                                    (uint)(_colorWidth * _colorHeight * 4),
                                    ColorImageFormat.Bgra);

                                this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                            }
                            this.colorBitmap.Unlock();
                        }

                        if ((takeScreenshot || dumpPpms) && !robot.IsMoving())
                        {
                            ushort[] depths = new ushort[_depthHeight * _depthWidth];

                            DepthSpacePoint[] mappedColor = new DepthSpacePoint[_colorHeight * _colorWidth];
                            depthFrame.CopyFrameDataToArray(depths);
                            cm.MapColorFrameToDepthSpace(depths, mappedColor);

                            byte[] colors = new byte[_colorHeight * _colorWidth * 4];
                            //this is the byte array that is converted into a ppm in the end, make it rgba form
                            colorFrame.CopyConvertedFrameDataToArray(colors, ColorImageFormat.Rgba);

                            this.mappedColor = mappedColor;
                            this.depths = depths;
                            this.colors = colors;

                            if (takeScreenshot)
                            {
                                ScreenshotSaveFile();
                                takeScreenshot = capturePanorama || false;
                            } else if (dumpPpms)
                            {
                                ScreenshotSaveFile();
                                //DumpPpms();
                                dumpPpms = false;
                            }

                            // Kick off another rotation if capturing a panorama
                            if (capturePanorama)
                            {
                                if (numRotations < MAX_ROTATIONS)
                                {
                                    numRotations++;
                                    RotateCW();
                                    StopMoving(rotateTime);
                                    Thread.Sleep(STABILIZE_TIME);

                                } else
                                {
                                    this.capturePanorama = false;
                                    this.takeScreenshot = false;
                                    this.panoramaNum++;
                                    this.imageNum = 0;
                                    this.numRotations = 0;
                                }
                            }
                        }
                        

                        depthCamera.Source = this.depthBitmap;
                        colorCamera.Source = this.colorBitmap;
                    }
                }
            }      
        }

        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            this.takeScreenshot = true;
        }

        private void DumpPpms_Click(object sender, RoutedEventArgs e)
        {
            this.dumpPpms = true;
        }

        private void ScreenshotSaveFile()
        {
            byte[] mDepth = new byte[mappedColor.Length];
            var minDepth = 0;
            ushort maxDepth = ushort.MaxValue;

            for (int i = 0; i < this.mappedColor.Length; i++)
            {
                var depthPoint = this.mappedColor[i];
                if (depthPoint.X != double.NegativeInfinity && depthPoint.Y != double.NegativeInfinity)
                {
                    var depthPix = (int)(depthPoint.Y * this.depthFrameDescription.Width + depthPoint.X);
                    var depth = this.depths[depthPix];
                    mDepth[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MAP_DEPTH_TO_BYTE) : 0);
                }
            }

            this.depthBytes = mDepth;
            this.colorBytes = this.colors;

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            if (this.depthBitmap != null && this.colorBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));


                string path = System.IO.Path.Combine(myPhotos, String.Format("Depth-{0:d3}-{1:d3}.png", this.panoramaNum, this.imageNum));

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                }
                catch (IOException)
                {

                }
                // create a png bitmap encoder which knows how to save a .png file
                encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.colorBitmap));

                myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                path = System.IO.Path.Combine(myPhotos, String.Format("Color-{0:d3}-{1:d3}.png", this.panoramaNum, this.imageNum));
                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                }
                catch (IOException)
                {

                }
            }

            this.imageNum += 1;
        }

        private void DumpPpms()
        {
            SaveData(GetPpmFilename("Color"), this.colorBytes, 32, Encoding.Default, 7);
            SaveData(GetPpmFilename("Depth"), this.depthBytes, 16, Encoding.Default, 7);
            //SaveData(GetPpmFilename("Combined"), this.mappedColor, Encoding.Default, 7);
        }

        private string GetPpmFilename(String description)
        {
            return System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyPictures), description + "-" + this.imageNum + ".ppm");
        }
        
        private void SaveData(String filename, byte[] data, int numBits,  Encoding encoding, Int32 type)
        {
            const Int32 bufferSize = 2048;

            //if (System.IO.Path.GetExtension(filename).ToLower() != ".ppm")
            //    filename += ".ppm";
            using (var fs = new FileStream(filename, FileMode.Create, FileAccess.ReadWrite, FileShare.None, bufferSize))
            {
                using (var bw = new BinaryWriter(fs))
                {
                    bw.Write(encoding.GetBytes(this.GetHeader(type, numBits, WIDTH, HEIGHT)));
                    bw.Write(data);
                }        
            }
        }

        private String GetHeader(Int32 type, int numBits, Int32 width, Int32 height)
        {
            return String.Format("P{0}\n{1} {2}\n{3}\n", type, width, height, 255);
        }
  
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;
            ushort depthScale = 8;
            minDepth *= depthScale;
            maxDepth *= depthScale;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel (scale up by 8 to increase contrast)
                ushort depth = frameData[i];
                depth *= depthScale;

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (ushort)(depth >= minDepth && depth <= maxDepth ? depth : 0);
            }
        }
        
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                2 * this.depthBitmap.PixelWidth,
                0);
        }
    }

    enum CameraMode
    {
        Color,
        Depth,
        Infrared
    }
}