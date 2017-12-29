#ifndef VIGNETTINGTABLE_H
#define VIGNETTINGTABLE_H

#include "vignetting.h"

namespace camera_model
{
class VignettingTable
{
    public:
    VignettingTable( std::string _vignetting_calib );

    VignettingTable( cv::Size image_size,
                     cv::Size boardSize,
                     std::vector< std::vector< double > > params,
                     bool _is_color = false );

    void buildTable( );
    cv::Mat removeLUT( cv::Mat& src );

    std::string toString( );

    private:
    vignetting m_vignetting;
    cv::Mat m_table;
};
}

#endif // VIGNETTINGTABLE_H
