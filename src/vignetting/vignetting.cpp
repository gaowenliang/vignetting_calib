#include <vignetting_model/vignetting/vignetting.h>

camera_model::vignetting::vignetting( std::string _vignetting_calib )
{
    readFromYamlFile( _vignetting_calib );
}

camera_model::vignetting::vignetting( cv::Size _image_size, bool _is_color )
: image_size( _image_size )
, m_is_color( _is_color )
{
    center( 0 ) = image_size.width / 2;
    center( 1 ) = image_size.height / 2;
    init( );
}

camera_model::vignetting::vignetting( std::string camera_model_file, bool _is_color )
: m_is_color( _is_color )
{
    cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( camera_model_file );

    image_size  = cv::Size( cam->imageWidth( ), cam->imageHeight( ) );
    center( 0 ) = cam->getPrinciple( ).x;
    center( 1 ) = cam->getPrinciple( ).y;
    init( );
}

void
camera_model::vignetting::init( )
{
    if ( m_is_color )
    {
        m_params.resize( 3 );
        m_params.at( 0 ).resize( ORDER_POLY );
        m_params.at( 1 ).resize( ORDER_POLY );
        m_params.at( 2 ).resize( ORDER_POLY );
    }
    else
    {
        m_params.resize( 1 );
        m_params.at( 0 ).resize( ORDER_POLY );
    }
}

double
camera_model::vignetting::get( int xx, int yy, int index )
{
    double dis = distance( double( xx ), double( yy ), center( 0 ), center( 1 ) );
    //        std::cout << " dis " << dis << std::endl;
    // clang-format off
  double r = m_params.at( index ).at( 0 )
      + m_params.at( index ).at( 1 ) * dis * dis
      + m_params.at( index ).at( 2 ) * dis * dis * dis * dis
      + m_params.at( index ).at( 3 ) * dis * dis * dis * dis * dis * dis;
    // clang-format on

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

bool
camera_model::vignetting::readFromYamlFile( const std::string vignetting_model_file )
{
    cv::FileStorage fs( vignetting_model_file, cv::FileStorage::READ );

    if ( !fs.isOpened( ) )
    {
        return false;
    }

    if ( !fs["model_type"].isNone( ) )
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if ( sModelType.compare( "VIGNETTING" ) != 0 )
        {
            return false;
        }
    }

    image_size.width  = static_cast< int >( fs["image_width"] );
    image_size.height = static_cast< int >( fs["image_height"] );

    center( 0 ) = static_cast< double >( fs["center_x"] );
    center( 1 ) = static_cast< double >( fs["center_y"] );

    int num_channel = static_cast< int >( fs["channel"] );
    m_is_color      = static_cast< int >( fs["is_color_cam"] );
    init( );

    cv::FileNode n = fs["vignetting_parameters"];
    for ( int channel_index = 0; channel_index < num_channel; ++channel_index )
    {
        std::ostringstream os;
        os << channel_index;
        cv::FileNode nn = n[std::string( "channel" ) + os.str( )];

        m_params.at( channel_index ).at( 0 ) = static_cast< double >( nn["k0"] );
        m_params.at( channel_index ).at( 1 ) = static_cast< double >( nn["k1"] );
        m_params.at( channel_index ).at( 2 ) = static_cast< double >( nn["k2"] );
        m_params.at( channel_index ).at( 3 ) = static_cast< double >( nn["k3"] );
    }
}

void
camera_model::vignetting::writeToYamlFile( std::string vignetting_model_file )
{
    cv::FileStorage fs( vignetting_model_file, cv::FileStorage::WRITE );

    fs << "model_type"
       << "VIGNETTING";
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "center_x" << center( 0 );
    fs << "center_y" << center( 1 );
    fs << "is_color_cam" << m_is_color;

    int num_channel;
    if ( m_is_color )
        num_channel = 3;
    else
        num_channel = 1;

    fs << "channel" << num_channel;

    fs << "vignetting_parameters";
    fs << "{";
    for ( int channel_index = 0; channel_index < num_channel; ++channel_index )
    {
        std::ostringstream os;
        os << channel_index;
        fs << std::string( "channel" ) + os.str( );
        fs << "{";
        fs << std::string( "k0" ) << m_params.at( channel_index ).at( 0 );
        fs << std::string( "k1" ) << m_params.at( channel_index ).at( 1 );
        fs << std::string( "k2" ) << m_params.at( channel_index ).at( 2 );
        fs << std::string( "k3" ) << m_params.at( channel_index ).at( 3 );
        fs << "}";
    }
    fs << "}";

    fs.release( );
}

bool
camera_model::vignetting::getIs_color( ) const
{
    return m_is_color;
}

namespace camera_model
{
std::ostream&
operator<<( std::ostream& out, const camera_model::vignetting& params )
{
    out << "Camera vignetting Parameters" << std::endl;
    out << "| model_type    | "
        << "VIGNETTING" << std::endl;
    out << "| image_width   | " << params.getImageSize( ).width << std::endl;
    out << "| image_height  | " << params.getImageSize( ).height << std::endl;
    out << "| center_x      | " << params.getCenter( )( 0 ) << std::endl;
    out << "| center_y      | " << params.getCenter( )( 1 ) << std::endl;
    out << "| is_color_cam  | " << params.getIs_color( ) << std::endl;

    int num_channel;
    if ( params.getIs_color( ) )
        num_channel = 3;
    else
        num_channel = 1;

    out << "| channel       | " << num_channel << std::endl;

    out << "| vignetting_parameters" << std::endl;
    for ( int channel_index = 0; channel_index < num_channel; ++channel_index )
    {
        std::ostringstream os;
        os << channel_index;
        out << std::string( "|    channel | " ) + os.str( ) << std::endl;
        out << std::string( "|        k0  | " ) << params.getParams( ).at( channel_index ).at( 0 )
            << std::endl;
        out << std::string( "|        k1  | " ) << params.getParams( ).at( channel_index ).at( 1 )
            << std::endl;
        out << std::string( "|        k2  | " ) << params.getParams( ).at( channel_index ).at( 2 )
            << std::endl;
        out << std::string( "|        k3  | " ) << params.getParams( ).at( channel_index ).at( 3 )
            << std::endl;
    }

    return out;
}
}

void
camera_model::vignetting::showResualt( )
{

    if ( getIs_color( ) )
    {
        std::vector< cv::Mat > images;
        for ( int index = 0; index < 3; ++index )
        {
            cv::Mat img_tmp( getImageSize( ), CV_8UC1, cv::Scalar( 0 ) );
            for ( int row_index = 0; row_index < getImageSize( ).height; ++row_index )
                for ( int col_index = 0; col_index < getImageSize( ).width; ++col_index )
                {
                    double value = get( col_index, row_index, index );
                    if ( value > 255 )
                        value = 255;

                    img_tmp.at< uchar >( row_index, col_index ) = value;
                }
            images.push_back( img_tmp );
        }
        cv::Mat image( getImageSize( ), CV_8UC3, cv::Scalar( 0, 0, 0 ) );

        cv::merge( images, image );

        cv::namedWindow( "resualt_b", cv::WINDOW_NORMAL );
        cv::namedWindow( "resualt_g", cv::WINDOW_NORMAL );
        cv::namedWindow( "resualt_r", cv::WINDOW_NORMAL );
        cv::namedWindow( "resualt", cv::WINDOW_NORMAL );
        cv::imshow( "resualt_b", images.at( 0 ) );
        cv::imshow( "resualt_g", images.at( 1 ) );
        cv::imshow( "resualt_r", images.at( 2 ) );
        cv::imshow( "resualt", image );
    }
    else
    {
        cv::Mat image( getImageSize( ), CV_8UC1, cv::Scalar( 0 ) );

        for ( int row_index = 0; row_index < getImageSize( ).height; ++row_index )
            for ( int col_index = 0; col_index < getImageSize( ).width; ++col_index )
            {
                double value = get( col_index, row_index, 0 );
                if ( value > 255 )
                    value = 255;

                image.at< uchar >( row_index, col_index ) = value;
            }

        cv::namedWindow( "resualt", cv::WINDOW_NORMAL );
        cv::imshow( "resualt", image );
    }

    cv::waitKey( 0 );
}

Eigen::Vector2d
camera_model::vignetting::getCenter( ) const
{
    return center;
}

cv::Size
camera_model::vignetting::getImageSize( ) const
{
    return image_size;
}

void
camera_model::vignetting::setParams( const std::vector< std::vector< double > >& params )
{
    m_params = params;
}
