#include "vignettingtable.h"
#include <opencv2/opencv.hpp>

camera_model::VignettingTable::VignettingTable( cv::Size image_size,
                                                std::string camera_model_file,
                                                cv::Size boardSize,
                                                std::vector< std::vector< double > > params,
                                                bool _is_color )
: vignetting( image_size, camera_model_file, boardSize, _is_color )
{
    m_params = params;

    if ( m_is_color )
        m_table = cv::Mat( image_size, CV_64FC3 );
    else
        m_table = cv::Mat( image_size, CV_64FC1 );
    buildTable( );
}

void
camera_model::VignettingTable::buildTable( )
{
    int channels = m_table.channels( );

    for ( int row_index = 0; row_index < m_table.rows; ++row_index )
    {
        for ( int col_index = 0; col_index < m_table.cols; ++col_index )
        {
            for ( int channel_index = 0; channel_index < channels; ++channel_index )
            {
                double feed = m_params[channel_index][0] / get( col_index, row_index, channel_index );
                if ( m_is_color )
                    m_table.at< cv::Vec3d >( row_index, col_index )[channel_index] = feed;
                else
                    m_table.at< double >( row_index, col_index ) = feed;
            }
        }
    }
}

cv::Mat
camera_model::VignettingTable::removeLUT( cv::Mat& src )
{
    int channels = src.channels( );
    int nRows    = src.rows;
    int nCols    = src.cols * channels;

    if ( m_is_color )
    {
        cv::Mat dst( src.rows, src.cols, CV_8UC3 );

        uchar* p_src;
        uchar* p_dst;
        double* p_table;
        int value;
        int row_index, col_index;
        for ( row_index = 0; row_index < nRows; ++row_index )
        {
            p_src   = src.ptr< uchar >( row_index );
            p_dst   = dst.ptr< uchar >( row_index );
            p_table = m_table.ptr< double >( row_index );
            for ( col_index = 0; col_index < nCols; ++col_index )
            {
                value = p_table[col_index] * p_src[col_index];
                if ( value > 255 )
                    value = 255;

                p_dst[col_index] = value;
            }
        }
        return dst;
    }
    else
    {
        cv::Mat dst( src.rows, src.cols, CV_8UC1 );

        uchar* p_src;
        uchar* p_dst;
        double* p_table;
        int value;
        int row_index, col_index;

        for ( row_index = 0; row_index < nRows; ++row_index )
        {
            p_src   = src.ptr< uchar >( row_index );
            p_dst   = dst.ptr< uchar >( row_index );
            p_table = m_table.ptr< double >( row_index );
            for ( col_index = 0; col_index < nCols; ++col_index )
            {
                value = p_table[col_index] * p_src[col_index];
                if ( value > 255 )
                    value = 255;

                p_dst[col_index] = value;
            }
        }
        return dst;
    }
}
