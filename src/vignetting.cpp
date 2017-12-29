#include "vignetting.h"

camera_model::vignetting::vignetting( cv::Size _image_size, std::string camera_model_file, cv::Size boardSize, bool _is_color )
: image_size( _image_size )
, chessbordSize( boardSize )
, m_is_color( _is_color )
{
    cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( camera_model_file );

    if ( m_is_color )
    {
        m_params.resize( 3 );
        m_params.at( 0 ).resize( ORDER_POLY );
        m_params.at( 1 ).resize( ORDER_POLY );
        m_params.at( 2 ).resize( ORDER_POLY );
        intensituValues.resize( 3 );
    }
    else
    {
        m_params.resize( 1 );
        m_params.at( 0 ).resize( ORDER_POLY );
        intensituValues.resize( 1 );
    }

    //        image_size = cv::Size( cam->imageWidth( ), cam->imageHeight( ) );
    center( 0 ) = image_size.width / 2;
    center( 1 ) = image_size.height / 2;
    //        center( 0 ) = 659.363; // image_size.width / 2;
    //        center( 1 ) = 516.62;  // image_size.height / 2;
}

double
camera_model::vignetting::get( int xx, int yy, int index )
{
    double dis = distance( double( xx ), double( yy ), center( 0 ), center( 1 ) );
    //        std::cout << " dis " << dis << std::endl;
    double r = m_params.at( index ).at( 0 ) + m_params.at( index ).at( 1 ) * dis * dis
               + m_params.at( index ).at( 2 ) * dis * dis * dis * dis
               + m_params.at( index ).at( 3 ) * dis * dis * dis * dis * dis * dis;

    return r;
}

cv::Mat
camera_model::vignetting::remove( cv::Mat image_in )
{
    if ( m_is_color )
    {
        cv::Mat image_tmp( image_in.rows, image_in.cols, CV_8UC3 );
        for ( int row_index = 0; row_index < image_size.height; ++row_index )
            for ( int col_index = 0; col_index < image_size.width; ++col_index )
            {
                cv::Vec3b velue = image_in.at< cv::Vec3b >( row_index, col_index );
                cv::Vec3b valuw_feeded;
                double feed[3];
                for ( int index = 0; index < 3; ++index )
                {
                    feed[index] = m_params.at( index ).at( 0 ) / get( col_index, row_index, index );
                    int value_feeded_tmp = velue[index] * feed[index];
                    if ( value_feeded_tmp > 255 )
                        value_feeded_tmp = 255;
                    if ( value_feeded_tmp < 0 )
                        value_feeded_tmp = 0;
                    valuw_feeded[index]  = value_feeded_tmp;
                    //  std::cout << " valuw_feeded[index] " <<  valuw_feeded[index] <<
                    //  std::endl;
                }
                image_tmp.at< cv::Vec3b >( row_index, col_index ) = valuw_feeded;
            }
        return image_tmp;
    }
    else
    {
        cv::Mat image_tmp( image_in.rows, image_in.cols, CV_8UC1 );
        for ( int row_index = 0; row_index < image_size.height; ++row_index )
        {
            for ( int col_index = 0; col_index < image_size.width; ++col_index )
            {
                double feed      = m_params[0][0] / get( col_index, row_index, 0 );
                int velue        = image_in.at< uchar >( row_index, col_index );
                int valuw_feeded = velue * feed;
                if ( valuw_feeded > 255 )
                    valuw_feeded = 255;
                if ( valuw_feeded < 0 )
                    valuw_feeded = 0;

                image_tmp.at< uchar >( row_index, col_index ) = valuw_feeded;
            }
        }
        return image_tmp;
    }
}

std::vector< std::vector< double > >
camera_model::vignetting::getParams( ) const
{
    return m_params;
}
