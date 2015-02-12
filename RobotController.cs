namespace Robot {
	using System;
	using System.IO.Ports;

	public class RobotController
	{
		const byte DRIVE_COMMAND = 137;
		const byte START_COMMAND = 128;
		const byte CONTROL_PACKET = 132;
		const int BAUD_RATE = 57600;
		const int READ_TIMEOUT = 10;
		const int WRITE_TIMEOUT = 1000;

		private SerialPort serialPort;
		private Boolean isMoving;

		public RobotController(string comPort)
		{	
			this.isMoving = false;
			this.serialPort = new SerialPort(comPort, BAUD_RATE, Parity.None, 8, StopBits.One);
			initializeSerialPort();
		}

		public void MoveForward()
		{
			// TODO(rafekettler): extract the speed into a parameter
			sendBytes(DRIVE_COMMAND, 0x01, 0xF4, 0x03, 0xE8);
			this.isMoving = true;
		}

		public void MoveBackward()
		{
			sendBytes(DRIVE_COMMAND, 0xFE, 0x0C, 0x03, 0xE8);
			this.isMoving = true;
		}

		public void RotateClockwise()
		{
			sendBytes(DRIVE_COMMAND, 0xF1, 0xF1, 0x00, 0x00);
			this.isMoving = true;
		}

		public void RotateCounterclockwise()
		{
			sendBytes(DRIVE_COMMAND, 0x01, 0xF4, 0x00, 0x00);
			this.isMoving = true;
		}

		public void StopMoving()
		{
			sendBytes(DRIVE_COMMAND, 0x00, 0x00, 0x00, 0x00);
			this.isMoving = false;
		}
			
		private void initializeSerialPort()
		{
			serialPort.ReadTimeout = READ_TIMEOUT;
			serialPort.WriteTimeout = WRITE_TIMEOUT;
			serialPort.Open();		
		}

		private void initializeRobot()
		{
			// TODO(rafekettler): extract these hardcoded commands
			sendBytes(START_COMMAND, CONTROL_PACKET);
			// makes it play a song to verify it's on
			sendBytes(139, 25, 0, 128);
			sendBytes(140, 1, 1, 48, 20 );
			sendBytes(141, 1);
		}

		private void sendBytes(params byte[] bytes) {
			serialPort.Write(bytes, 0, bytes.Length);
		}
	}
}
