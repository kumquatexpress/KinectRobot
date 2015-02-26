using Microsoft.Kinect;
using System.Windows;
using System.Windows.Media.Imaging;

namespace Kinect {
    public class KinectReader
    {
        private const ushort DEPTH_SCALE = 8;

        private WriteableBitmap depthBitmap;
        private WriteableBitmap colorBitmap;
        private KinectSensor sensor;
        private MultiSourceFrameReader reader;
        private ushort[] depthPixels;

        public KinectReader(WriteableBitmap depthBitmap, WriteableBitmap colorBitmap)
        {
            this.depthBitmap = depthBitmap;
            this.colorBitmap = colorBitmap;
            this.sensor = KinectSensor.GetDefault();
            sensor.Open();
            this.reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color);
            reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
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
                using (var colorBuffer = colorFrame.LockRawImageBuffer())
                {
                    var description = sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
                    if (IsSameSize(description, colorBitmap))
                    {
                        colorBitmap.Lock();
                        colorFrame.CopyConvertedFrameDataToIntPtr(
                            colorBitmap.BackBuffer,
                            (uint)(description.Width * description.Height * 4),
                            ColorImageFormat.Bgra);
                        colorBitmap.AddDirtyRect(new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight));
                        colorBitmap.Unlock();
                    }
                }
            }

            using (var depthFrame = reference.DepthFrameReference.AcquireFrame())
            {
                var description = sensor.DepthFrameSource.FrameDescription;
                if (IsSameSize(description, depthBitmap))
                {
                    using (var depthBuffer = depthFrame.LockImageBuffer())
                    {
                        if (depthPixels == null)
                        {
                            this.depthPixels = new ushort[description.Height * description.Width];
                        }
                        ProcessDepthFrameData(depthBuffer, depthFrame, depthPixels);
                        depthBitmap.WritePixels(
                            new Int32Rect(0, 0, depthBitmap.PixelWidth, depthBitmap.PixelHeight),
                            depthPixels,
                            2 * depthBitmap.PixelWidth,
                            0);
                    }
                }
            }
        }

        private static bool IsSameSize(FrameDescription description, WriteableBitmap bitmap)
        {
            return description.Width == bitmap.PixelWidth && description.Height == bitmap.PixelHeight;
        }

        private static unsafe void ProcessDepthFrameData(KinectBuffer buffer, DepthFrame depthFrame, ushort[] pixels)
        {
            ushort* frameData = (ushort*)buffer.UnderlyingBuffer;
            ushort minDepth = (ushort)(depthFrame.DepthMinReliableDistance * DEPTH_SCALE);
            ushort maxDepth = (ushort)(depthFrame.DepthMaxReliableDistance * DEPTH_SCALE);

            for (int i = 0; i < (int)(buffer.Size / depthFrame.FrameDescription.BytesPerPixel); i++)
            {
                ushort depth = (ushort)(frameData[i] * DEPTH_SCALE);
                pixels[i] = depth >= minDepth && depth <= maxDepth ? depth : (ushort)0;
            }
        }
    }
}
