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

class VignettingTable : public vignetting
{
    public:
    VignettingTable( cv::Size image_size,
                     std::string camera_model_file,
                     cv::Size boardSize,
                     std::vector< std::vector< double > > params,
                     bool _is_color = false )
    : vignetting( image_size, camera_model_file, boardSize, _is_color )
    , m_table( cv::Mat( image_size, CV_64FC1 ) )
    {
        m_params = params;
        buildTable( );
    }

    void buildTable( )
    {
        for ( int raw_index = 0; raw_index < image_size.height; ++raw_index )
        {
            for ( int col_index = 0; col_index < image_size.width; ++col_index )
            {
                double feed = m_params[0][0] / get( col_index, raw_index, 0 );
                //                std::cout << " feed " << feed << std::endl;
                m_table.at< double >( raw_index, col_index ) = feed;
            }
        }
    }

    cv::Mat removeLUT( cv::Mat& src )
    {
        cv::Mat dst( src.rows, src.cols, CV_8UC1 );

        uchar* p_src;
        uchar* p_dst;
        double* p_table;
        int velue;
        int raw_index, col_index;
        for ( raw_index = 0; raw_index < image_size.height; ++raw_index )
        {
            p_src   = src.ptr< uchar >( raw_index );
            p_dst   = dst.ptr< uchar >( raw_index );
            p_table = m_table.ptr< double >( raw_index );
            for ( col_index = 0; col_index < image_size.width; ++col_index )
            {
                velue = p_table[col_index] * p_src[col_index];
                if ( velue > 255 )
                    velue = 255;

                p_dst[col_index] = velue;
            }
        }
        return dst;
    }

    cv::Mat m_table;
};
}

#endif // VIGNETTING_H
