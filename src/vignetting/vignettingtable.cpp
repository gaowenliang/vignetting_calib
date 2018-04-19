#include <opencv2/opencv.hpp>
#include <vignetting_model/vignetting/vignettingtable.h>

camera_model::VignettingTable::VignettingTable( std::string _vignetting_calib )
: m_vignetting( _vignetting_calib )
{
    buildTable( );
    std::cout << this->toString( ) << std::endl;
}

camera_model::VignettingTable::VignettingTable( std::string _vignetting_calib, std::string mask_file )
: m_vignetting( _vignetting_calib )
{
    buildTable( mask_file );
    std::cout << this->toString( ) << std::endl;
}

camera_model::VignettingTable::VignettingTable( cv::Size image_size,
                                                std::vector< std::vector< double > > params,
                                                bool _is_color )
: m_vignetting( image_size, _is_color )
{
    m_vignetting.setParams( params );
    buildTable( );
    std::cout << this->toString( ) << std::endl;
}

void
camera_model::VignettingTable::buildTable( )
{
    if ( m_vignetting.getIs_color( ) )
        m_table = cv::Mat( m_vignetting.getImageSize( ), CV_64FC3 );
    else
        m_table = cv::Mat( m_vignetting.getImageSize( ), CV_64FC1 );

    int channels = m_table.channels( );

    for ( int row_index = 0; row_index < m_table.rows; ++row_index )
    {
        for ( int col_index = 0; col_index < m_table.cols; ++col_index )
        {
            for ( int channel_index = 0; channel_index < channels; ++channel_index )
            {
                double feed = m_vignetting.getParams( )[channel_index][0]
                              / m_vignetting.get( col_index, row_index, channel_index );
                if ( m_vignetting.getIs_color( ) )
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

    if ( m_vignetting.getIs_color( ) )
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

std::string
camera_model::VignettingTable::toString( )
{
    std::ostringstream oss;
    oss << m_vignetting;
    return oss.str( );
}

cv::Mat
camera_model::VignettingTable::getTable( ) const
{
    return m_table;
}

void
camera_model::VignettingTable::buildTable( const std::string file_mask )
{
    std::cout << "[#Vignet] Loading Mask...." << std::endl;
    cv::Mat m_mask;
    m_mask = cv::imread( file_mask, cv::IMREAD_GRAYSCALE );

    if ( m_vignetting.getIs_color( ) )
        m_table = cv::Mat( m_vignetting.getImageSize( ), CV_64FC3 );
    else
        m_table = cv::Mat( m_vignetting.getImageSize( ), CV_64FC1 );

    int channels = m_table.channels( );

    for ( int row_index = 0; row_index < m_table.rows; ++row_index )
    {
        for ( int col_index = 0; col_index < m_table.cols; ++col_index )
        {
            int mask_v = m_mask.at< uchar >( row_index, col_index );

            for ( int channel_index = 0; channel_index < channels; ++channel_index )
            {
                double feed;
                if ( mask_v > 20 )
                    feed = m_vignetting.getParams( )[channel_index][0]
                           / m_vignetting.get( col_index, row_index, channel_index );
                else
                    feed = 1.0;

                if ( m_vignetting.getIs_color( ) )
                {
                    m_table.at< cv::Vec3d >( row_index, col_index )[channel_index] = feed;
                }
                else
                {
                    m_table.at< double >( row_index, col_index ) = feed;
                }
            }
        }
    }

    std::cout << "[#Vignet] Mask Loaded." << std::endl;
}
