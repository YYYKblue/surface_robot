#include "../include/LdrAlgorithm.h"

namespace pavo2 {


LdrAlgorithm::LdrAlgorithm()
{
    InitTables();
}


void LdrAlgorithm::FarHighRelectivityPreProcess(pavo_response_scan_t* points, int size)
{
    int index_left = 0;
    int index_right = 0;
    int distance_max_index = 0;
    int distance_min_index = 0;
    int intensity_max_points_num = 0;

    bool inCalc = false;

    std::vector<pavo_response_scan_t*> dat_ptr;
    for(int i = 0; i < size; i++)
    {
        dat_ptr.push_back(&points[i]);
    }

    for(int i = 0; i < size; i++) //first san preprocess
    {
        int threshold_index = points[i].distance / 500; //unit:2 mm
        if(threshold_index > 20)
            threshold_index = 20;

        if(points[i].intensity >= m_intensity_threshold_table[threshold_index])
        {
            if(!inCalc)
            {
                inCalc = true;
                index_left = i;
                index_right = i;
                distance_max_index = i;
                distance_min_index = i;
                intensity_max_points_num = 1;
            }
            else
            {
                index_right = i;
                if(points[i].distance > points[distance_max_index].distance)
                    distance_max_index = i;
                if(points[i].distance < points[distance_min_index].distance)
                    distance_min_index = i;
                if(points[i].intensity == INTENSITY_MAX)
                    intensity_max_points_num += 1;
            }
        }
        else //go out of intensity-special area
        {
            if(!inCalc)
                continue;

            inCalc = false;
            if((index_right - index_left + 1 < 2) //points num < 3
               || (intensity_max_points_num >= 5) //points number with I=2000 >= 5
               || (points[distance_max_index].distance - points[distance_min_index].distance > 300)) //difference > 30cm
            {
                continue; //not satisfy the conditions
            }

            for(int j = index_left; j <= index_right; j++)
            {
                points[j].intensity = INTENSITY_MAX;
            }

        }

    }
}

void LdrAlgorithm::FarHighRelectivityFilter(pavo_response_scan_t* points, int size)
{
    FarHighRelectivityPreProcess(points, size);

#if 1
    int index_left = 0;
    int index_right = 0;
    int distance_max = 0;
    int distance_min = 0;
    int intensity_max = 0;
    int intensity_min = 0;
    int window_num = 0;

    bool inCalc = false;
    for(int i = 0; i < size; i++)
    {
        if(points[i].intensity == INTENSITY_MAX)
        {
            if(!inCalc)
            {
                inCalc = true;
                index_left = i;
            }

            index_right = i;
        }
        else
        {
            if(!inCalc)
                continue;

            inCalc = false;

            if(index_right - index_left + 1 < 3)//at least 3 points with I==2000
                continue;

            //process left side
            if(index_left >= 5)
            {
                window_num = 1;
                while(window_num < 50) //
                {
                    for(int w = 0; w <= FILTER_WINDOW_SIZE; w++)
                    {
                        int index = index_left - window_num - w;
                        if(points[index].distance > distance_max)
                            distance_max = points[index].distance;
                        if(points[index].distance < distance_min)
                            distance_min = points[index].distance;
                        if(points[index].intensity > intensity_max)
                            intensity_max = points[index].intensity;
                        if(points[index].intensity < intensity_min)
                            intensity_min = points[index].intensity;
                    }

                    if((distance_max - distance_min < 300) && (intensity_max - intensity_min < 20)) //found 30cm
                        break;

                    window_num++;
                }

                float distance_left = abs((points[index_left - window_num].distance - points[index_left].distance) *
                        (points[index_left - window_num].angle - points[index_left].angle) * 3.1415926f / 180.0f);

                if((window_num < 50) && (distance_left > 300)) //30cm
                {
                    for(int j = index_left - window_num; j < index_left; j++)
                    {
                        points[j].distance = 0;
                        points[j].intensity = 0;
                    }
                }
            }

            // process right side
            if(index_right + 5 < size)
            {
                window_num = 1;
                while(window_num < 50) //
                {
                    for(int w = 0; w <= FILTER_WINDOW_SIZE; w++)
                    {
                        int index = index_right + window_num + w;
                        if(points[index].distance > distance_max)
                            distance_max = points[index].distance;
                        if(points[index].distance < distance_min)
                            distance_min = points[index].distance;
                        if(points[index].intensity > intensity_max)
                            intensity_max = points[index].intensity;
                        if(points[index].intensity < intensity_min)
                            intensity_min = points[index].intensity;
                    }

                    if((distance_max - distance_min < 300) && (intensity_max - intensity_min < 20)) //found
                        break;

                    window_num++;
                }

                float distance_right = abs((points[index_right + window_num].distance - points[index_right].distance) *
                        (points[index_right + window_num].angle - points[index_right].angle) * 3.1415926f / 180.0f);

                if((window_num < 50) && (distance_right > 300)) //30cm
                {
                    for(int j = index_right ; j < index_right + window_num; j++)
                    {
                        points[j].distance = 0;
                        points[j].intensity = 0;
                    }
                }
            }

        }
    }
#endif
}


#define HDL_Grabber_toRadians(x) ((x) * 3.1415926 / 180.0)
const int NUM_ROT_ANGLES = 36001;
void LdrAlgorithm::InitTables()
{
    if(m_intensity_threshold_table.size() == 0)
    {
        m_intensity_threshold_table.resize(INTENSITY_THRESHOLD_TABLE_LENGTH + 1);
        m_intensity_threshold_table[0] = 600;
        m_intensity_threshold_table[1] = 590;
        m_intensity_threshold_table[2] = 590;
        m_intensity_threshold_table[3] = 560;
        m_intensity_threshold_table[4] = 560;
        m_intensity_threshold_table[5] = 550;
        m_intensity_threshold_table[6] = 550;
        m_intensity_threshold_table[7] = 520;
        m_intensity_threshold_table[8] = 520;
        m_intensity_threshold_table[9] = 510;
        m_intensity_threshold_table[10] = 510;
        m_intensity_threshold_table[11] = 510;
        m_intensity_threshold_table[12] = 510;
        m_intensity_threshold_table[13] = 500;
        m_intensity_threshold_table[14] = 500;
        m_intensity_threshold_table[15] = 490;
        m_intensity_threshold_table[16] = 490;
        m_intensity_threshold_table[17] = 490;
        m_intensity_threshold_table[18] = 490;
        m_intensity_threshold_table[19] = 470;
        m_intensity_threshold_table[20] = 470;
        m_intensity_threshold_table[21] = 450;
        m_intensity_threshold_table[22] = 450;
        m_intensity_threshold_table[23] = 440;
        m_intensity_threshold_table[24] = 440;
    }

    if((m_cos_lookup_table.size() == 0) || (m_sin_lookup_table.size() == 0))
    {
      m_cos_lookup_table.resize(NUM_ROT_ANGLES);
      m_sin_lookup_table.resize(NUM_ROT_ANGLES);


      for(unsigned int i = 0; i < NUM_ROT_ANGLES; i++)
      {
        double rad = HDL_Grabber_toRadians(i / 100.0);
        m_cos_lookup_table[i] = std::cos(rad);
        m_sin_lookup_table[i] = std::sin(rad);
      }
    }
}

float LdrAlgorithm::AngleCos(float A1, float B1, float A2, float B2)
{

    float nominator = A1 * A2 + B1 * B2;

    float denominator = static_cast<float>(sqrt(pow(A1, 2) + pow(B1, 2))) *
            static_cast<float>(sqrt(pow(A2, 2) + pow(B2, 2)));

    if(denominator < 0.0001) //means denominator == 0
        return 1.0f;

    return nominator / denominator;
}

} //namespace pavo2
