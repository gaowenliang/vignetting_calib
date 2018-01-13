#ifndef VIGNETTING_H
#define VIGNETTING_H

#include <camera_model/camera_models/CameraFactory.h>
#include <ceres/ceres.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#define ORDER_POLY 4
namespace camera_model
{

class vignetting
{
    public:
    vignetting( ) {}
    vignetting( std::string _vignetting_calib );
    vignetting( cv::Size image_size, cv::Size boardSize, bool _is_color = false );
    vignetting( std::string camera_model_file, cv::Size boardSize, bool _is_color = false );

    void init( );

    public:
    double get( int xx, int yy, int index );
    cv::Mat remove( cv::Mat image_in );
    void showResualt( );
    bool readFromYamlFile( const std::string vignetting_model_file );
    void writeToYamlFile( std::string vignetting_model_file );

    std::vector< std::vector< double > > getParams( ) const;
    void setParams( const std::vector< std::vector< double > >& params );
    cv::Size getImageSize( ) const;
    Eigen::Vector2d getCenter( ) const;
    bool getIs_color( ) const;
    cv::Size getChessbordSize( ) const;
    friend std::ostream& operator<<( std::ostream& out, const vignetting& params );

    template< typename T >
    inline T distance( T x1, T y1, T x2, T y2 )
    {
        return sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );
    }

    private:
    camera_model::CameraPtr cam;
    std::vector< std::vector< double > > m_params;

    Eigen::Vector2d center;
    cv::Size image_size;
    cv::Size chessbordSize;

    bool m_is_color;
};
}

#endif // VIGNETTING_H
