#include "lidarParser.h"
#define OUTPUT_FULL_DATA_CNT	5

lidarParser::lidarParser()
{
    // protocolDatagram = new carnaviDatagram();
    g_LiDARModel = CARNAVICOM::MODEL::VL_AS16;

    m_detect_Start = false;
    m_detect_End = false;

    g_checkModel = true;

    m_vlas16_Processor = new VL_AS16();
	m_industrial = new industrialLiDAR();


	g_checked_preDatagramSet = false;
	g_cntForAllData = 0;
	g_bufSize = 0;

	g_lidarBuffer.clear();
}

/**
 * @brief to set datagram of LiDAR raw data
 * 
 * @param data 		datagram of LiDAR raw data
 * @return true 	to classify the sensor and parse LiDAR raw data
 * @return false 	can not classify kind of LiDAR sensor
 */
bool lidarParser::setData(const std::vector<u_char> &data)
{
    if(g_checkModel)
    {
        protocolDatagram.LiDAR_Model = classificationModel(data);  //라이다 종류 확인
		if(protocolDatagram.LiDAR_Model != -1)	// not lidar data check
		{
			g_checkModel = false;
		}
		// printf("model ? %d\n", protocolDatagram.LiDAR_Model);
    }

    switch (protocolDatagram.LiDAR_Model)
    {
    case static_cast<int>(CARNAVICOM::MODEL::LiDAR::VL_AS16) :
        /* code */
		// printf("[lidarParser][setData] classification AS16!\n");
        return accumulateData_VLAS16(data);
	case CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R002IF01:
		// printf("[lidarParser][setData] classification R2!\n");
		return accumulateData_industrial(data, protocolDatagram.LiDAR_Model);
	case CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R001IK02:
		// break;
		// printf("[lidarParser][setData] classification R300!\n");
		return accumulateData_industrial(data, protocolDatagram.LiDAR_Model);
    default:
		return false;
    }

	return true;
}

/**
 * @brief return carnavicom LiDAR model
 * 
 * @return int ref. CARNAVICOM::MODEL::LiDAR
 */
int lidarParser::getLidarModel()
{
    return protocolDatagram.LiDAR_Model;
}


/**
 * @brief return the parsing result protocol structure
 * 
 * @param datagram ref. carnaviDatagram structure
 */
void lidarParser::getLiDARdatagram(carnaviDatagram &datagram)
{
	if(protocolDatagram.PARA_Input_END)	//check lidar data input end
	{
    	datagram = protocolDatagram;
	}
}


/**
 * @brief to classify Carnavicom LiDAR model
 * 
 * @param data datagram of LiDAR raw data
 * @return int  ref. CARNAVICOM::MODEL::LiDAR
 */
int lidarParser::classificationModel(const std::vector<u_char> &data)
{
    // printf("[parser][classification MODEL]\n");

    if((static_cast<int>(data[0]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::HEAD::LOWER))
        && (static_cast<int>(data[1]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::HEAD::UPPER))
		&& (static_cast<int>(data[data.size() - 2]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::TAIL::LOWER))
        && (static_cast<int>(data[data.size() - 1]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::TAIL::UPPER)))		// VL-AS16 header & tail detected
    {
        return static_cast<int>(CARNAVICOM::MODEL::LiDAR::VL_AS16);
    }
	else if((static_cast<int>(data[0]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::HEAD::LOWER))
        && (static_cast<int>(data[1]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::HEAD::UPPER)))					// VL-AS16 only header detected
	{
		return static_cast<int>(CARNAVICOM::MODEL::LiDAR::VL_AS16);
	}

	if((data[CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] & 0xFF) 
		== CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)																// industrial header detected
	{
		u_char indus_M = data[CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::PRODUCT_LINE];
		switch(indus_M)
		{
		case CARNAVICOM::MODEL::LiDAR::VL_R002IF01:
			return CARNAVICOM::MODEL::LiDAR::VL_R002IF01;
		case CARNAVICOM::MODEL::LiDAR::VL_R001IK02:
			return CARNAVICOM::MODEL::LiDAR::VL_R001IK02;
		}
	}

	return -1;
}

/**
 * @brief to parse Carnavicom LiDAR raw data(VL-AS16)
 * 
 * @param data datagram of LiDAR raw data
 */
void lidarParser::parsing_VLAS16(const std::vector<u_char> &data)
{
//    g_lengthBuffer.clear();
    parsingRawData(data, protocolDatagram);
}

/**
 * @brief to check all Data input(VL-AS16)
 * 
 * @param data 		datagram of LiDAR raw data
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
bool lidarParser::accumulateData_VLAS16(const std::vector<u_char> &data)
{
	// header detected
    if((static_cast<int>(data[0]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::HEAD::LOWER))
        && (static_cast<int>(data[1]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::HEAD::UPPER)))
    {
		protocolDatagram.clear();
		g_lidarBuffer.clear();
		g_lidarBuffer = data;
        m_detect_Start  = true;
    }

	// tail detected
    if(static_cast<int>(data[data.size() - 2]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::TAIL::LOWER)
        && static_cast<int>(data[data.size() - 1]) == static_cast<int>(CARNAVICOM::VL_AS16::PROTOCOL_VALUE::TAIL::UPPER)	
        && m_detect_Start)
    {
        for(int i=0; i<data.size(); i++)
        {
            g_lidarBuffer.push_back(data[i]);
        }
        m_detect_End = true;
    }

    if(m_detect_Start && m_detect_End )
    {
        //data output Sque.
        m_detect_Start = false;
        m_detect_End = false;
        
        //parsing
        parsing_VLAS16(g_lidarBuffer);

        previous_lidarBuffer.clear();
        previous_lidarBuffer = g_lidarBuffer;
        g_lidarBuffer.clear();

		m_detect_Start = false;
		m_detect_End = false;

        return true;
    }
    else
    {
        return false;
    }

}

/**
 * @brief  to check all Data input(industrial)
 * 
 * @param data 		datagram of LiDAR raw data
 * @param model 	industrial LiDAR model
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
bool lidarParser::accumulateData_industrial(const std::vector<u_char> &data, int model)
{
	//check data size 
	// u_char model = data[CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::PRODUCT_LINE];
	switch(model)
	{
		case CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R002IF01:	//R2
			return process_R2(data);
		case CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::MODEL::VL_R001IK02:	//R300
			return process_R300(data);
		default:
			return false;
	}
}

/**
 * @brief to check all Data input(industrial - VL_R002IF01(R2))
 * 
 * @param data 		datagram of LiDAR raw data
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
bool lidarParser::process_R2(const std::vector<u_char> &data) 
{
	if(data.size() < CARNAVICOM::INDUSTRIAL::R2::PROTOCOL_SIZE::TOTAL)		// check data size
	{
		// return false;
		if(data[CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] 		// check header value
			== CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)
		{
			m_detect_Start = true;											
			protocolDatagram.clear();										// datagram clear
			g_lidarBuffer.clear();											// buf clear
			g_lidarBuffer = data;
		}
		else
		{
			if(m_detect_Start)
			{
				for(size_t i=0; i<data.size(); i++)
				{
					g_lidarBuffer.push_back(data[i]);
				}
				m_detect_End = true;
			}
		}
	}
	else if(data.size() == CARNAVICOM::INDUSTRIAL::R2::PROTOCOL_SIZE::TOTAL)
	{
		u_char ch = data[static_cast<int>(CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::COMMAND::PARAMETER)];
		// ERROR update 2022-09-22
		// check datagram clear phase, when R2's 0 channel data input.
		if(ch == 0)
		{
			protocolDatagram.clear();
		}
		m_detect_Start = true;
		m_detect_End = true;
		g_lidarBuffer = data;
	}

	if(m_detect_Start && m_detect_End)
	{
		parsingRawData_industrial(g_lidarBuffer, protocolDatagram);

		m_detect_Start = false;
		m_detect_End = false;
	}
	printf("check Return value : %d\n", protocolDatagram.PARA_Input_END);
	return protocolDatagram.PARA_Input_END;
}

/**
 * @brief to check all Data input(industrial - VL_R002IF01(R2))
 * 
 * @param data 		datagram of LiDAR raw data
 * @return true 	if datagram has header and tail
 * @return false 	if datagram has only header or tail
 */
bool lidarParser::process_R300(const std::vector<u_char> &data) 
{
	if(data.size() < CARNAVICOM::INDUSTRIAL::R300::PROTOCOL_SIZE::TOTAL)
	{
		// return false;
		if(data[CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_POS::HEADER] 
			== CARNAVICOM::INDUSTRIAL::COMMON::PROTOCOL_VALUE::HEADER)
		{
			m_detect_Start = true;
			protocolDatagram.clear();
			g_lidarBuffer.clear();
			g_lidarBuffer = data;
		}
		else
		{
			if(m_detect_Start)
			{
				for(size_t i=0; i<data.size(); i++)
				{
					g_lidarBuffer.push_back(data[i]);
				}
				m_detect_End = true;
			}
		}
	}
	else if(data.size() == CARNAVICOM::INDUSTRIAL::R300::PROTOCOL_SIZE::TOTAL)
	{
		protocolDatagram.clear();
		m_detect_Start = true;
		m_detect_End = true;
		g_lidarBuffer = data;
	}

	if(m_detect_Start
		&& m_detect_End
		&& g_lidarBuffer.size() ==  CARNAVICOM::INDUSTRIAL::R300::PROTOCOL_SIZE::TOTAL)
	{
		parsingRawData_industrial(g_lidarBuffer, protocolDatagram);
		m_detect_Start = false;
		m_detect_End = false;
	}
	return protocolDatagram.PARA_Input_END;
}

/**
 * @brief return LiDAR raw data size
 * 
 * @return size_t 
 */
size_t lidarParser::getRawDataSize()
{
    if(!m_detect_Start && !m_detect_End && previous_lidarBuffer.size() != 0)
    {
        return previous_lidarBuffer.size();
    }
    else
    {
        return 0;
    }
}

/**
 * @brief return LiDAR raw data
 * 
 * @return std::vector<u_char> 
 */
std::vector<u_char> lidarParser::getRawData()
{
    if(!m_detect_Start && !m_detect_End && previous_lidarBuffer.size() != 0)
    {
        return previous_lidarBuffer;
    }
}

/**
 * @brief to parsing LiDAR raw data to Protocol structure(VL-AS16)
 * 
 * @param data 		datagram of LiDAR raw data
 * @param datagram 	ref. carnaviDatagram structure
 */
void lidarParser::parsingRawData(const std::vector<u_char> &data, carnaviDatagram &datagram)
{
    m_vlas16_Processor->processor(data, datagram);
}

/**
 * @brief to parsing LiDAR raw data to Protocol structure(industrial)
 * 
 * @param data 		datagram of LiDAR raw data
 * @param datagram 	ref. carnaviDatagram structure
 */
void lidarParser::parsingRawData_industrial(const std::vector<u_char> &data, carnaviDatagram &datagram)
{
	m_industrial->process(data, datagram);
}