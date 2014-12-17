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

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        SerialPort port;

        const int WIDTH = 1920;
        const int HEIGHT = 1080;

        const byte DRIVE = 137;
        const byte CONTROL = 132;
        const byte START = 128;

        int MOVE_TIME = 400;
        int ROTATE_TIME = 200;

        private const int MapDepthToByte = 8000 / 256;

        private KinectSensor kinectSensor = null;
        private DepthFrameReader depthFrameReader = null;
        private FrameDescription depthFrameDescription = null;
        private WriteableBitmap depthBitmap = null;
        private WriteableBitmap colorBitmap = null;
        private FrameDescription colorFrameDescription = null;
        private CoordinateMapper cm = null;

        private byte[] depthPixels = null;
        private byte[] depthBytes = null;
        private byte[] colorBytes = null;

        private bool takeScreenshot = false;
        private bool dumpPpms = false;
        private byte[] colors = null;
        private ushort[] depths = null;
        private DepthSpacePoint[] mappedColor = null;
        private int imageNum = 0;

        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            //open the robot com port
            try
            {
                this.port = new SerialPort("COM9", 57600, Parity.None, 8, StopBits.One);
                this.TurnOnRoomba();
            }
            catch (Exception)
            {
                Console.WriteLine("Robot not detected, unable to turn on");
            }
            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
            this.colorBitmap = new WriteableBitmap(this.colorFrameDescription.Width, this.colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // open the sensor
            this.kinectSensor.Open();
            this.cm = this.kinectSensor.CoordinateMapper;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
            this.imageNum = 0;
        }

        private void TurnOnRoomba()
        {
            byte[] send = { START, CONTROL };
            this.port.ReadTimeout = 10;
            this.port.WriteTimeout = 1000;
            this.port.Open();

            this.port.Write(send, 0, send.Length);
            //makes it play a song to verify it's on
            this.port.Write(new byte[] { 139, 25, 0, 128 }, 0, 4);
            this.port.Write(new byte[] { 140, 1, 1, 48, 20 }, 0, 5);
            this.port.Write(new byte[] { 141, 1 }, 0, 2);
        }

        /*
         * Stops moving after a given amount of milliseconds
         */
        private void StopMoving(int after)
        {
            Thread.Sleep(after);
            byte[] send = { DRIVE, 0x00, 0x00, 0x00, 0x00 };
            this.port.Write(send, 0, send.Length);
        }

        private void ForwardButton_Click(object sender, RoutedEventArgs e)
        {
            byte[] send = { DRIVE, 0x01, 0xF4, 0x03, 0xE8};
            this.port.Write(send, 0, send.Length);
            
            StopMoving(MOVE_TIME);
        }

        private void BackwardButton_Click(object sender, RoutedEventArgs e)
        {
            byte[] send = { DRIVE, 0xFE, 0x0C, 0x03, 0xE8};
            this.port.Write(send, 0, send.Length);

            StopMoving(MOVE_TIME);
        }

        private void RotateCW_Click(object sender, RoutedEventArgs e)
        {
            byte[] send = { DRIVE, 0xF1, 0xF1, 0x00, 0x00};
            for (int i = 0; i < 5; i++)
            {
                this.port.Write(send, 0, send.Length);

                StopMoving(ROTATE_TIME);
                Thread.Sleep(300);
            }
        }

        private void RotateCCW_Click(object sender, RoutedEventArgs e)
        {
            byte[] send = { DRIVE, 0x01, 0xF0, 0x00, 0x00};
            this.port.Write(send, 0, send.Length);

            StopMoving(ROTATE_TIME);
        }

        private void SubmitMove_Click(object sender, RoutedEventArgs e)
        {
            MOVE_TIME = Convert.ToInt32(this.SleepTime_Move.Text);
        }

        private void SubmitRotate_Click(object sender, RoutedEventArgs e)
        {
            ROTATE_TIME = Convert.ToInt32(this.SleepTime_Rotate.Text);
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.GetDefault();

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
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

                        if (takeScreenshot || dumpPpms)
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
                                this.ScreenshotSaveFile();
                                takeScreenshot = false;
                            } else if (dumpPpms)
                            {
                                this.DumpPpms();
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
                    mDepth[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                }
            }

            this.depthBytes = mDepth;
            this.colorBytes = this.colors;

            string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            if (this.depthBitmap != null && this.colorBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));


                string path = System.IO.Path.Combine(myPhotos, "Depth-"+this.imageNum+"-"+ time + ".png");

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

                time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                path = System.IO.Path.Combine(myPhotos, "Color-"+this.imageNum+"-" + time + ".png");
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

            if (this.depthBytes != null && this.colorBytes != null)
            {
                StringBuilder byteString = new StringBuilder();
                for(int i = 0; i < depthBytes.Length; i++){
                    if ((i+1) % WIDTH == 0)
                    {
                        byteString.Append("\n");
                    }
                    byteString.Append(depthBytes[i].ToString() + " ");
                }
                SaveData(System.IO.Path.Combine(myPhotos, "Depth-"+this.imageNum+"-" + time), depthBytes, 8, Encoding.GetEncoding(1251), 5);
                byteString.Clear();

                var realColors = new byte[(colorBytes.Length * 3) /4];
                var counter = 0;
                for (int i = 0; i < colorBytes.Length; i++)
                {
                    if ((i+1) % WIDTH == 0)
                    {
                        byteString.Append("\n");
                    }
                    if (i % 4 != 3)
                    {
                        byteString.Append(colorBytes[i].ToString() + " ");
                        realColors[counter] = colorBytes[i];
                        counter += 1;
                    }
                }
                SaveData(System.IO.Path.Combine(myPhotos, "Color-" + this.imageNum + "-" + time), realColors, 8, Encoding.GetEncoding(1251), 6);
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

            if (System.IO.Path.GetExtension(filename).ToLower() != ".ppm")
                filename += ".ppm";
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
            return String.Format("P{0}\n{1} {2}\n{3}\n", type, width, height, 1 << numBits);
        }
  
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
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