#ifndef VIGNETTINGTABLE_H
#define VIGNETTINGTABLE_H

#include "vignetting.h"

namespace camera_model
{
class VignettingTable : public vignetting
{
    public:
    VignettingTable( cv::Size image_size,
                     std::string camera_model_file,
                     cv::Size boardSize,
                     std::vector< std::vector< double > > params,
                     bool _is_color = false );

    void buildTable( );

    cv::Mat removeLUT( cv::Mat& src );

    cv::Mat m_table;
};
}

#endif // VIGNETTINGTABLE_H
