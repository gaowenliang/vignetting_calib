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
    vignetting( cv::Size image_size, std::string camera_model_file, cv::Size boardSize, bool _is_color = false );

    public:
    double get( int xx, int yy, int index );

    cv::Mat remove( cv::Mat image_in );

    template< typename T >
    T distance( T x1, T y1, T x2, T y2 )
    {
        return sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );
    }

    cv::Mat draw( )
    {
        cv::Mat image_show( image_size, CV_8UC3 );

        for ( int row_index = 0; row_index < image_show.rows; ++row_index )
            for ( int col_index = 0; col_index < image_show.cols; ++col_index )
            {
                double r = distance( double( col_index ), double( row_index ), center( 0 ), center( 1 ) );
            }
    }

    cv::Mat showPoly( ) { cv::Mat poly_image( ); }
    std::vector< std::vector< double > > getParams( ) const;

    public:
    camera_model::CameraPtr cam;
    std::vector< std::vector< double > > m_params;

    Eigen::Vector2d center;
    cv::Size image_size;
    cv::Size chessbordSize;

    bool m_is_color;
    int points_num;
    std::vector< std::vector< double > > intensituValues;
    std::vector< double > rs;
};
}

#endif // VIGNETTING_H
