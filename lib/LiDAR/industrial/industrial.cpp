#include "industrial.h"

industrialLiDAR::industrialLiDAR(/* args */)
{
	g_checked_CH0 = false;
}

industrialLiDAR::~industrialLiDAR()
{
}

/**
 * @brief to process Carnavicom Industrial LiDAR sensor
 * 
 * @param input 	datagram of LiDAR raw data
 * @param output 	protocol structure of Carnavicom LiDAR sensor
 */
void industrialLiDAR::process(const std::vector<u_char> &input, carnaviDatagram &output)
{
	//sort length
	if(input.size() > 0)
	{
		switch(input[CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::PRODUCT_LINE])
		{
		case CARNAVICOM::MODEL::LiDAR::VL_R002IF01:
			R2(input, output);
			break;
		case CARNAVICOM::MODEL::LiDAR::VL_R001IK02:
			R300(input, output);
			break;
		}
	}
}

/**
 * @brief to parse raw data to length value
 * 
 * @param input 	datagram of LiDAR raw data
 * @param output 	protocol structure of Carnavicom LiDAR sensor
 * @param ch 		now length data's channel
 */
void industrialLiDAR::parseLength(const std::vector<u_char> &input, carnaviDatagram &output, int ch)
{
	int start = CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::RAWDATA_START;
	int end = input.size() - CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_SIZE::CHECKSUM;

	float up = 0;
	float low = 0;
	float len = 0;

	int cnt=0;
	for(int i=start; i<end; i+=2)
	{
		up = input[i];
		low = input[i+1];
		len = up + low/100;		//convert 2 byte to length[m]
		// if(ch == 0)
			// printf("[%d] index : %d, %f\n", ch, cnt, len);
		output.industrial_Length[ch][cnt] = len;
		cnt++;
	}
}

/**
 * @brief to process Carnavicom industrial LiDAR sensor VL-R002IK01(R2)
 * 
 * @param input 	datagram of LiDAR raw data
 * @param output 	protocol structure of Carnavicom LiDAR sensor
 */
void industrialLiDAR::R2(const std::vector<u_char> &input, carnaviDatagram &output)
{
	u_char mode = input[static_cast<int>(CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if(mode == CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		// check ch 0 data input
		if(ch == CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			// printf("[industrial][input CH0]\n");
			output.clear();
			//set specification
			{
				output.LiDAR_Model = CARNAVICOM::MODEL::LiDAR::VL_R002IF01;
				output.PARA_Vertical_Resolution 
					= CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::VERTICAL_RESOLUTION;
				output.PARA_Horizontal_Resolution
					= CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_RESOLUTION;
				output.PARA_Start_Angle = 0;
				output.PARA_End_Angle 
					= CARNAVICOM::INDUSTRIAL::SPECIFICATION::R2::HORIZONTAL_DATA_CNT;
			}
				
			parseLength(input, output, static_cast<int>(ch & 0x0F));

			g_checked_CH0 = true;
		}
		else if(ch == CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_1)	//check ch 1 data input
		{
			// printf("[industrial][input CH1]\n");
			if(g_checked_CH0)
			{
				parseLength(input, output, static_cast<int>(ch & 0x0F));	// convert byte to length
				g_checked_CH0 = false;
				output.PARA_Input_END = true;		//data input complete
			}
		}
	}
}

/**
 * @brief to process Carnavicom industrial LiDAR sensor VL-R001IK02(R300)
 * 
 * @param input		datagram of LiDAR raw data
 * @param output	protocol structure of Carnavicom LiDAR sensor
 */
void industrialLiDAR::R300(const std::vector<u_char> &input, carnaviDatagram &output)
{
	u_char mode = input[static_cast<int>(CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::MODE)];
	u_char ch = input[static_cast<int>(CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];

	if(mode == CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::COMMAND::MODE::DISTANCE_DATA)
	{
		//check ch 0 data input
		if(ch == CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::CHANNEL::CHANNEL_0)
		{
			// output.clear();
			//set specification
			{
				output.LiDAR_Model = CARNAVICOM::MODEL::LiDAR::VL_R001IK02;
				output.PARA_Vertical_Resolution 
					= CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::VERTICAL_RESOLUTION;
				output.PARA_Horizontal_Resolution
					= CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_RESOLUTION;
				output.PARA_Start_Angle = 0;
				output.PARA_End_Angle 
					= CARNAVICOM::INDUSTRIAL::SPECIFICATION::R300::HORIZONTAL_DATA_CNT;
			}
				
			parseLength(input, output, static_cast<int>(ch & 0x0F));	// convert byte to length
			output.PARA_Input_END = true;		//data input complete
		}
	}
}
