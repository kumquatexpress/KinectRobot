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

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor _sensor;
        MultiSourceFrameReader _reader;

        private const int MapDepthToByte = 8000 / 256;

        private KinectSensor kinectSensor = null;

        private DepthFrameReader depthFrameReader = null;

        private FrameDescription depthFrameDescription = null;

        private WriteableBitmap depthBitmap = null;

        private WriteableBitmap colorBitmap = null;

        private CoordinateMapper cm = null;

        private byte[] depthPixels = null;

        private string statusText = null;

        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            // open the sensor
            this.kinectSensor.Open();
            this.cm = this.kinectSensor.CoordinateMapper;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
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
                    var _colorWidth = colorFrame.FrameDescription.Width;
                    var _colorHeight = colorFrame.FrameDescription.Height;
                    var _depthWidth = depthFrame.FrameDescription.Width;
                    var _depthHeight = depthFrame.FrameDescription.Height;

                    if (depthFrame != null && colorFrame != null)
                    {
                        ushort[] depths = new ushort[_depthHeight*_depthWidth];
                        
                        DepthSpacePoint[] mappedColor = new DepthSpacePoint[_colorHeight*_colorWidth];
                        depthFrame.CopyFrameDataToArray(depths);

                        var cBitmap = (WriteableBitmap)colorFrame.ToBitmap();
                        byte[] colors = new byte[_colorHeight * cBitmap.BackBufferStride];
                        colorFrame.ToBitmap().CopyPixels(colors, cBitmap.BackBufferStride, 0);

                        this.depthBitmap = (WriteableBitmap)depthFrame.ToBitmap();
                        this.colorBitmap = cBitmap;

                        cm.MapColorFrameToDepthSpace(depths, mappedColor);
                        byte[] mColor = new byte[_depthHeight * _depthWidth * Constants.BYTES_PER_PIXEL];
                        for (int i = 0; i < mappedColor.Length; i++)
                        {
                            var colorPoint =  mappedColor[i];
                            if (colorPoint.X != double.NegativeInfinity && colorPoint.Y != double.NegativeInfinity)
                            {
                                var starting = i * 4;
                                var depthPix = (int)(colorPoint.X * colorPoint.Y);
                                mColor[depthPix++] = colors[starting++];
                                mColor[depthPix++] = colors[starting++];
                                mColor[depthPix++] = colors[starting++];
                                mColor[depthPix++] = colors[starting++];
                            }
                            
                        }
                        WriteableBitmap bitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
                        bitmap.WritePixels(new Int32Rect(0, 0, bitmap.PixelWidth, bitmap.PixelHeight), mColor, this.depthBitmap.BackBufferStride, 0);

                        depthCamera.Source = depthFrame.ToBitmap();

                        colorCamera.Source = bitmap;

                    }
                }
            }


        
        }

        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.depthBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));

                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = System.IO.Path.Combine(myPhotos, "KinectScreenshot-Depth-" + time + ".png");

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

                path = System.IO.Path.Combine(myPhotos, "KinectScreenshot-Color-" + time + ".png");
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
        }

    }

    enum CameraMode
    {
        Color,
        Depth,
        Infrared
    }
}