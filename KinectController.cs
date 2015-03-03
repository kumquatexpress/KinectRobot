using Microsoft.Kinect;
using System.Windows;
using System.Windows.Media.Imaging;
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
using System;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;

using System.Windows.Navigation;


namespace Kinect {
    public class KinectReader
    {
        private const ushort DEPTH_SCALE = 8;
        private const int WIDTH = 1920;
        private const int HEIGHT = 1080;

        private const int MAP_DEPTH_TO_BYTE = 8000 / 256;

        public  WriteableBitmap depthBitmap;
        public WriteableBitmap colorBitmap;
        private byte[] depthBytes = null;
        private byte[] colorBytes = null;

        private KinectSensor sensor;
        private MultiSourceFrameReader reader;
        private CoordinateMapper cm;

        private FrameDescription depthFrameDescription = null;
        private FrameDescription colorFrameDescription = null;

        private byte[] colors = null;
        private ushort[] depths = null;
        private DepthSpacePoint[] mappedColor = null;

        private bool takeScreenShot;
        private String filename;
        private ushort[] depthPixels;

        public KinectReader(WriteableBitmap depthBitmap, WriteableBitmap colorBitmap)
        {

            this.sensor = KinectSensor.GetDefault();
            sensor.Open();

            this.depthBitmap = depthBitmap;
            this.colorBitmap = colorBitmap;
            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.sensor.DepthFrameSource.FrameDescription;
            this.colorFrameDescription = this.sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            this.cm = this.sensor.CoordinateMapper;

            this.reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color);
            this.reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
        }

        public void close()
        {
            sensor.Close();
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();
            using (var colorFrame = reference.ColorFrameReference.AcquireFrame())
            {
                using (var depthFrame = reference.DepthFrameReference.AcquireFrame())
                {
                    var description = sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

                    if (colorFrame != null && IsSameSize(description, colorBitmap) && depthFrame != null &&
                        IsSameSize(sensor.DepthFrameSource.FrameDescription, depthBitmap))
                    {
                        var _colorWidth = colorFrame.ColorFrameSource.FrameDescription.Width;
                        var _colorHeight = colorFrame.ColorFrameSource.FrameDescription.Height;
                        var _depthWidth = depthFrame.DepthFrameSource.FrameDescription.Width;
                        var _depthHeight = depthFrame.DepthFrameSource.FrameDescription.Height;

                        using (var colorBuffer = colorFrame.LockRawImageBuffer())
                        {
                            colorBitmap.Lock();
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                colorBitmap.BackBuffer,
                                (uint)(description.Width * description.Height * 4),
                                ColorImageFormat.Bgra);
                            colorBitmap.AddDirtyRect(new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight));
                            colorBitmap.Unlock();

                        }


                        description = sensor.DepthFrameSource.FrameDescription;

                        using (var depthBuffer = depthFrame.LockImageBuffer())
                        {
                            if (depthPixels == null)
                            {
                                this.depthPixels = new ushort[description.Height * description.Width];
                            }
                            ProcessDepthFrameData(depthBuffer, depthFrame);
                            depthBitmap.WritePixels(
                                new Int32Rect(0, 0, depthBitmap.PixelWidth, depthBitmap.PixelHeight),
                                depthPixels,
                                2 * depthBitmap.PixelWidth,
                                0);
                        }

                        if (this.takeScreenShot)
                        {
                            ushort[] depths = new ushort[_depthHeight * _depthWidth];

                            DepthSpacePoint[] mappedColor = new DepthSpacePoint[_colorHeight * _colorWidth];
                            depthFrame.CopyFrameDataToArray(depths);
                            //cm.MapColorFrameToDepthSpace(depths, mappedColor);

                            byte[] colors = new byte[_colorHeight * _colorWidth * 4];
                            //this is the byte array that is converted into a ppm in the end, make it rgba form
                            colorFrame.CopyConvertedFrameDataToArray(colors, ColorImageFormat.Rgba);

                            //this.mappedColor = mappedColor;
                            this.depths = depths;
                            this.colors = colors;

                            this.takeScreenShot = false;
                            SaveFile();
                        }
                    }
                    
                }
            }
        }

        public void TakeScreenShot(String filename)
        {
            this.takeScreenShot = true;
            this.filename = filename;
        }

        public bool Available()
        {
            return this.sensor.IsAvailable;
        }

        private void SaveFile()
        {
            this.colorBytes = this.colors;

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            if (this.depthBitmap != null && this.colorBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.depthBitmap));


                string path = System.IO.Path.Combine(myPhotos, String.Format("Depth-{0}.png", this.filename));

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

                path = System.IO.Path.Combine(myPhotos, String.Format("Color-{0}.png", this.filename));
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
                    Console.Error.Write(String.Format("Couldn't write file {0} from kinect", this.filename));
                }
            }
        }

        private static bool IsSameSize(FrameDescription description, WriteableBitmap bitmap)
        {
            return description.Width == bitmap.PixelWidth && description.Height == bitmap.PixelHeight;
        }

        private unsafe void ProcessDepthFrameData(KinectBuffer buffer, DepthFrame depthFrame)
        {
            ushort* frameData = (ushort*)buffer.UnderlyingBuffer;
            ushort minDepth = (ushort)(depthFrame.DepthMinReliableDistance * DEPTH_SCALE);
            ushort maxDepth = (ushort)(depthFrame.DepthMaxReliableDistance * DEPTH_SCALE);

            int maxCounter = (int)(buffer.Size / depthFrame.FrameDescription.BytesPerPixel);
            for (int i = 0; i < maxCounter; ++i)
            {
                ushort depth = frameData[i];
                depth *= DEPTH_SCALE;
                depthPixels[i] = (ushort)(depth >= minDepth && depth <= maxDepth ? depth : 0);
            }
        }
    }
}
