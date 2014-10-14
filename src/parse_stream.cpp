/*******************************************************************
 * NOT FOR USE!  FIREFIGHTING ONLY!
 * NOT FOR USE!  FIREFIGHTING ONLY!
 */
















#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <zmq.hpp>

struct termios options;

float swap_bytes(unsigned char* buffer)
{
	unsigned char reversed[4];
	reversed[0] = buffer[3];
	reversed[1] = buffer[2];
	reversed[2] = buffer[1];
	reversed[3] = buffer[0];

	float tmp;
	memcpy(&tmp, &reversed[0], sizeof(float));
	return tmp;
}

int main()
{
	int desc = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
	zmq::context_t context(1);
	zmq::socket_t socket(context, ZMQ_PUB);
	socket.bind("tcp://127.0.0.1:55004");

	std::cout << "open succeeded\n";
	if(desc == -1)
	{
		return -1;
	}
	else {
		tcgetattr(desc, &options);
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		options.c_cflag |= (CLOCAL | CREAD);
		tcsetattr(desc, TCSANOW, &options);
	}

	unsigned char buffer[128];
	int num_bytes_read;
	float roll, pitch, yaw;
	float q0, q1, q2, q3;
	char out[128];

	std::cout << "finished init\n";

	while(1) {
		// read the first byte, see if it's start of packet
		num_bytes_read = read(desc, &buffer, 1);
		if(num_bytes_read > 0 && buffer[0] == (char)0x75)
		{
			// read the 2nd byte, see if it's 2nd byte in header
			num_bytes_read = read(desc, &buffer, 1);
			if(num_bytes_read > 0 && buffer[0] == (char)0x65)
			{
				// read descriptor set, payload length, field length, field descriptor
				num_bytes_read = read(desc, &buffer, 4);
				char descriptor_set = buffer[0];
				char payload_length = buffer[1];
				char field_length = buffer[2];
				char field_descriptor = buffer[3];

				// FIXME we just handle single replies in packets for now
				if(payload_length == field_length)
				{
					num_bytes_read = read(desc, &buffer, payload_length);

					// FIXME we assume checksum is good

					// base command set
					if(descriptor_set == (char)0x01)
					{
						if(field_descriptor == (char)0xF1)
						{
							// ACK/NACK
							if(buffer[1] == (char)0x00)
							{
								std::cout << buffer[0] << ": ACK" << std::endl;
							}
							else
							{
								std::cout << buffer[0] << ": NACK" << std::endl;
							}
						}
					}
					// AHRS data set
					else if(descriptor_set == (char)0x80)
					{
						// euler angles
						if(field_descriptor == (char)0x0C)
						{
							roll = (double)buffer[0];
							pitch = (double)buffer[4];
							yaw = (double)buffer[8];
						}
						// quaternion
						else if(field_descriptor == (char)0x0A)
						{
							q0 = swap_bytes(&buffer[0]);
							q1 = swap_bytes(&buffer[4]);
							q2 = swap_bytes(&buffer[8]);
							q3 = swap_bytes(&buffer[12]);
							snprintf(out, 128, "%f %f %f %f", q0, q1, q2, q3);
							std::cout << out << std::endl;
							zmq::message_t msg(strlen(out));
							memcpy((void*)msg.data(), out, strlen(out));
							socket.send(msg);
							//			    std::cout << out << std::endl;
						}
					}
					// GPS data set
					else if(descriptor_set == (char)0x81)
					{
						if(field_descriptor == (char)0x03)
						{
							char valid = buffer[40];
							char lat_lng_valid = valid & 0x01;
							char ellipsoid_height_valid = valid & 0x02;
							char msl_height_valid = valid & 0x04;
							char horizontal_accuracy_valid = valid & 0x08;
							char vertical_accuracy_valid = valid & 0x10;

							if(lat_lng_valid)
							{
							}
							if(ellipsoid_height_valid)
							{
							}
							if(msl_height_valid)
							{
							}
							if(horizontal_accuracy_valid)
							{
							}
							if(vertical_accuracy_valid)
							{
							}
						}
					}
				}
			}
		}
	}
}
