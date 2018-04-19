#ifndef VIGNETTINGTABLE_H
#define VIGNETTINGTABLE_H

#include "vignetting.h"

namespace camera_model
{
class VignettingTable
{
    public:
    VignettingTable( ) {}
    VignettingTable( std::string _vignetting_calib );
    VignettingTable( std::string _vignetting_calib, std::string mask_file );
    VignettingTable( cv::Size image_size, std::vector< std::vector< double > > params, bool _is_color = false );

    void buildTable( );
    void buildTable( const std::string file_mask );
    cv::Mat removeLUT( cv::Mat& src );
    cv::Mat getTable( ) const;

    std::string toString( );

    private:
    vignetting m_vignetting;
    cv::Mat m_table;
};
}

#endif // VIGNETTINGTABLE_H
