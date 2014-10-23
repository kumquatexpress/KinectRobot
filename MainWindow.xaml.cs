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

        const int WIDTH = 1920;
        const int HEIGHT = 1080;

        private const int MapDepthToByte = 8000 / 256;

        private KinectSensor kinectSensor = null;

        private DepthFrameReader depthFrameReader = null;

        private FrameDescription depthFrameDescription = null;

        private WriteableBitmap depthBitmap = null;

        private WriteableBitmap colorBitmap = null;

        private CoordinateMapper cm = null;

        private byte[] depthPixels = null;

        private byte[] depthBytes = null;

        private byte[] colorBytes = null;

        private string statusText = null;

        private byte[] colors = null;
        private ushort[] depths = null;
        private DepthSpacePoint[] mappedColor = null;
        private int imageNum = 0;

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
            this.imageNum = 0;
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
                        var _colorWidth = colorFrame.FrameDescription.Width;
                        var _colorHeight = colorFrame.FrameDescription.Height;
                        var _depthWidth = depthFrame.FrameDescription.Width;
                        var _depthHeight = depthFrame.FrameDescription.Height;

                        ushort[] depths = new ushort[_depthHeight * _depthWidth];

                        DepthSpacePoint[] mappedColor = new DepthSpacePoint[_colorHeight * _colorWidth];
                        depthFrame.CopyFrameDataToArray(depths);
                        cm.MapColorFrameToDepthSpace(depths, mappedColor);

                        var cBitmap = (WriteableBitmap)colorFrame.ToBitmap();
                        byte[] colors = new byte[_colorHeight * cBitmap.BackBufferStride];
                        colorFrame.ToBitmap().CopyPixels(colors, cBitmap.BackBufferStride, 0);

                        this.mappedColor = mappedColor;
                        this.depths = depths;
                        this.colors = colors;
                        this.depthBitmap = (WriteableBitmap)depthFrame.ToBitmap();
                        this.colorBitmap = cBitmap;

                        depthCamera.Source = depthFrame.ToBitmap();
                        colorCamera.Source = colorFrame.ToBitmap();
                    }
                }
            }      
        }

        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            byte[] mColor = new byte[this.depths.Length * Constants.BYTES_PER_PIXEL];
            for (int i = 0; i < this.mappedColor.Length; i++)
            {
                var colorPoint = this.mappedColor[i];
                if (colorPoint.X != double.NegativeInfinity && colorPoint.Y != double.NegativeInfinity)
                {
                    var starting = i * 4;
                    var depthPix = (int)(colorPoint.X * colorPoint.Y);
                    mColor[depthPix++] = this.colors[starting++];
                    mColor[depthPix++] = this.colors[starting++];
                    mColor[depthPix++] = this.colors[starting++];
                    mColor[depthPix++] = this.colors[starting++];
                }
            }

            byte[] mDepth = new byte[this.depths.Length];
            for (int i = 0; i < this.depths.Length; i++)
            {
                mDepth[i] = (byte)this.depths[i];
            }
            this.depthBytes = mDepth;
            this.colorBytes = mColor;

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
                    byteString.Append(" " + depthBytes[i].ToString());
                }
                SaveData(System.IO.Path.Combine(myPhotos, "Depth-"+this.imageNum+"-" + time), byteString.ToString(), Encoding.GetEncoding(1251));
                byteString.Clear();
                for (int i = 0; i < colorBytes.Length; i++)
                {
                    byteString.Append(" " + colorBytes[i].ToString());
                }
                SaveData(System.IO.Path.Combine(myPhotos, "Color-" + this.imageNum + "-" + time), byteString.ToString(), Encoding.GetEncoding(1251));
            }

            this.imageNum += 1;
        }

        private void SaveData(String filename, String data, Encoding encoding)
        {
            const Int32 bufferSize = 2048;

            if (System.IO.Path.GetExtension(filename).ToLower() != ".ppm")
                filename += ".ppm";
            using (var fs = new FileStream(filename, FileMode.Create, FileAccess.ReadWrite, FileShare.None, bufferSize))
            {
                using (var bw = new BinaryWriter(fs, encoding))
                {
                    var buffer = encoding.GetBytes(this.GetHeader(WIDTH, HEIGHT));
                    bw.Write(buffer);

                    buffer = encoding.GetBytes(data);
                    bw.Write(buffer);
                }
            }
        }

        private String GetHeader(Int32 width, Int32 height)
        {
            return String.Format("P6 {0} {1} 255 ", width, height);
        }
    }

    enum CameraMode
    {
        Color,
        Depth,
        Infrared
    }
}