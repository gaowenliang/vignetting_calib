#include <opencv2/opencv.hpp>
#include <vignetting_model/vignetting/vignettingcalib.h>

camera_model::VignettingCalib::VignettingCalib( cv::Size image_size, bool _is_color )
: vignetting( image_size, _is_color )
{
    initCalib( );
}

camera_model::VignettingCalib::VignettingCalib( std::string camera_model_file, bool _is_color )
: vignetting( camera_model_file, _is_color )
{
    initCalib( );
}

camera_model::VignettingCalib::VignettingCalib( cv::Size image_size, cv::Size boardSize, bool _is_color )
: vignetting( image_size, _is_color )
, mBoardSize( boardSize )
{
    initCalib( );
}

camera_model::VignettingCalib::VignettingCalib( std::string camera_model_file, cv::Size boardSize, bool _is_color )
: vignetting( camera_model_file, _is_color )
, mBoardSize( boardSize )
{
    initCalib( );
}

void
camera_model::VignettingCalib::initCalib( )
{
    if ( getIs_color( ) )
    {
        intensituValues.resize( 3 );
    }
    else
    {
        intensituValues.resize( 1 );
    }
}

void
camera_model::VignettingCalib::readin_points( const std::vector< std::pair< cv::Point2d, std::vector< double > > > points )
{
    points_num = points.size( );
    if ( getIs_color( ) )
    {
        intensituValues.at( 0 ).clear( );
        intensituValues.at( 1 ).clear( );
        intensituValues.at( 2 ).clear( );
    }
    else
    {
        intensituValues.at( 0 ).clear( );
    }
    rs.clear( );

    for ( int point_index = 0; point_index < points_num; ++point_index )
    {
        double r = sqrt( ( points[point_index].first.x - getCenter( )( 0 ) )
                         * ( points[point_index].first.x - getCenter( )( 0 ) )
                         + ( points[point_index].first.y - getCenter( )( 1 ) )
                           * ( points[point_index].first.y - getCenter( )( 1 ) ) );

        rs.push_back( r );
        if ( getIs_color( ) )
        {
            intensituValues.at( 0 ).push_back( points[point_index].second.at( 0 ) );
            intensituValues.at( 1 ).push_back( points[point_index].second.at( 1 ) );
            intensituValues.at( 2 ).push_back( points[point_index].second.at( 2 ) );
        }
        else
        {
            intensituValues.at( 0 ).push_back( points[point_index].second.at( 0 ) );
        }
    }
}

void
camera_model::VignettingCalib::solve( )
{

    if ( rs.size( ) == intensituValues.at( 0 ).size( ) )
        points_num = rs.size( );

    int index_max = 1;
    if ( getIs_color( ) )
        index_max = 3;

    std::vector< std::vector< double > > param_tmp;
    for ( int index = 0; index < index_max; ++index )
    {
        double poly_k[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        ceres::Problem problem;
        for ( int i = 0; i < points_num; ++i )
        {
            ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction< luminanceError, 1, 4 >(
            new luminanceError( intensituValues.at( index )[i], rs[i] ) );

            problem.AddResidualBlock( costFunction, NULL /* squared loss */, poly_k );
        }
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type   = ceres::DOGLEG;
        ceres::Solver::Summary summary;
        ceres::Solve( options, &problem, &summary );

        std::vector< double > param_channel_tmp;
        for ( int param_index = 0; param_index < ORDER_POLY; ++param_index )
        {
            param_channel_tmp.push_back( poly_k[param_index] );
        }
        param_tmp.push_back( param_channel_tmp );
        std::cout << param_channel_tmp[0] << " " << param_channel_tmp[1] << " "
                  << param_channel_tmp[2] << " " << param_channel_tmp[3] << " " << std::endl;
    }
    setParams( param_tmp );
}

void
camera_model::VignettingCalib::getValue9( std::vector< double >& value, const cv::Mat image, int x_index, int y_index )
{
    value.clear( );

    if ( getIs_color( ) )
    {
        for ( int index = 0; index < 3; ++index )
        {
            double value_tmp_cv = ( image.at< cv::Vec3b >( y_index - 1, x_index - 1 )[index]
                                    + image.at< cv::Vec3b >( y_index - 1, x_index )[index]
                                    + image.at< cv::Vec3b >( y_index - 1, x_index + 1 )[index]
                                    + image.at< cv::Vec3b >( y_index, x_index - 1 )[index]
                                    + image.at< cv::Vec3b >( y_index, x_index )[index]
                                    + image.at< cv::Vec3b >( y_index, x_index + 1 )[index]
                                    + image.at< cv::Vec3b >( y_index + 1, x_index - 1 )[index]
                                    + image.at< cv::Vec3b >( y_index + 1, x_index )[index]
                                    + image.at< cv::Vec3b >( y_index + 1, x_index + 1 )[index] )
                                  / 9.0;
            value.push_back( value_tmp_cv );
            //            std::cout << "value_tmp " << index << " " << value_tmp_cv <<
            //            std::endl;
        }
    }
    else
    {
        value.push_back( ( image.at< uchar >( y_index - 1, x_index - 1 ) + image.at< uchar >( y_index - 1, x_index )
                           + image.at< uchar >( y_index - 1, x_index + 1 )
                           + image.at< uchar >( y_index, x_index - 1 )
                           + image.at< uchar >( y_index, x_index )
                           + image.at< uchar >( y_index, x_index + 1 )
                           + image.at< uchar >( y_index + 1, x_index - 1 )
                           + image.at< uchar >( y_index + 1, x_index )
                           + image.at< uchar >( y_index + 1, x_index + 1 ) )
                         / 9.0 );
    }
}

void
camera_model::VignettingCalib::getValue1( std::vector< double >& value, const cv::Mat image, int x_index, int y_index )
{
    value.clear( );

    if ( getIs_color( ) )
    {
        for ( int index = 0; index < 3; ++index )
        {
            double value_tmp_cv = image.at< cv::Vec3b >( y_index, x_index )[index];
            value.push_back( value_tmp_cv );
            //            std::cout << "value_tmp " << index << " " << value_tmp_cv <<
            //            std::endl;
        }
    }
    else
    {
        //        std::cout << "v " << int( image.at< uchar >( y_index, x_index ) ) << "\n";
        value.push_back( image.at< uchar >( y_index, x_index ) );
    }
}

cv::Mat
camera_model::VignettingCalib::getChessboardPoints( cv::Mat image_in, std::vector< cv::Point2f > points, int threshold )
{
    cv::Mat image = image_in;
    cv::Mat image_color;
    if ( image.channels( ) == 1 )
    {
        cv::cvtColor( image, image_color, CV_GRAY2RGB );
    }
    else
    {
        image.copyTo( image_color );
    }

    int cnt = 0;
    for ( int row_index = 0; row_index < getChessbordSize( ).height - 1; ++row_index )
        for ( int col_index = 0; col_index < getChessbordSize( ).width - 1; ++col_index )
        {
            int row_index_add = getChessbordSize( ).width * row_index;
            ++cnt;

            double dx_raw = ( points[row_index_add + col_index].x
                              - points[row_index_add + col_index + getChessbordSize( ).width].x );
            double dy_raw = ( points[row_index_add + col_index].y
                              - points[row_index_add + col_index + getChessbordSize( ).width].y );
            double dx_col
            = ( points[row_index_add + col_index].x - points[row_index_add + col_index + 1].x );
            double dy_col
            = ( points[row_index_add + col_index].y - points[row_index_add + col_index + 1].y );

            double avg_x
            = ( points[row_index_add + col_index].x + points[row_index_add + col_index + 1].x
                + points[row_index_add + col_index + getChessbordSize( ).width].x
                + points[row_index_add + col_index + getChessbordSize( ).width + 1].x )
              / 4;
            double avg_y
            = ( points[row_index_add + col_index].y + points[row_index_add + col_index + 1].y
                + points[row_index_add + col_index + getChessbordSize( ).width].y
                + points[row_index_add + col_index + getChessbordSize( ).width + 1].y )
              / 4;

            double index_x;
            double index_y;

            if ( row_index == 0 )
            {
                index_x = avg_x + dx_raw;
                index_y = avg_y + dy_raw;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
            if ( col_index == 0 )
            {
                index_x = avg_x + dx_col;
                index_y = avg_y + dy_col;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
            if ( row_index == getChessbordSize( ).height - 2 )
            {
                index_x = avg_x - dx_raw;
                index_y = avg_y - dy_raw;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
            if ( col_index == getChessbordSize( ).width - 2 )
            {
                index_x = avg_x - dx_col;
                index_y = avg_y - dy_col;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
            if ( row_index == 0 && col_index == 0 )
            {
                index_x = avg_x + dx_raw + dx_col;
                index_y = avg_y + dy_raw + dy_col;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
            else if ( row_index == 0 && col_index == getChessbordSize( ).width - 2 )
            {
                index_x = avg_x + dx_raw - dx_col;
                index_y = avg_y + dy_raw - dy_col;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
            else if ( row_index == getChessbordSize( ).height - 2 && col_index == 0 )
            {
                index_x = avg_x - dx_raw + dx_col;
                index_y = avg_y - dy_raw + dy_col;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
            else if ( row_index == getChessbordSize( ).height - 2
                      && col_index == getChessbordSize( ).width - 2 )
            {
                index_x = avg_x - dx_raw - dx_col;
                index_y = avg_y - dy_raw - dy_col;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }

            {
                index_x = avg_x;
                index_y = avg_y;
                inBoard( index_x, index_y );
                addValue9( image, image_color, index_x, index_y, threshold );
            }
        }

    return image_color;
}

void
camera_model::VignettingCalib::getOnePoints( cv::Mat image_in, cv::Mat& image_color, cv::Point2f points, int threshold )
{
    cv::Mat image = image_in;

    double avg_x = points.x;
    double avg_y = points.y;

    double index_x;
    double index_y;

    {
        index_x = avg_x;
        index_y = avg_y;
        inBoard( index_x, index_y );
        addValue1( image, image_color, index_x, index_y, threshold );
    }
}

void
camera_model::VignettingCalib::addValue9( cv::Mat& image, cv::Mat& image_color, double& index_x, double& index_y, int threshold )
{
    double value_tmp;

    if ( getIs_color( ) )
    {
        cv::Vec3b value_tmp_cv = image.at< cv::Vec3b >( index_y, index_x );

        value_tmp = avgInThree( value_tmp_cv[0], value_tmp_cv[1], value_tmp_cv[2] );
    }
    else
        value_tmp = image.at< uchar >( index_y, index_x );

    if ( value_tmp < threshold )
        drawRedPoint( image_color, index_x, index_y );
    else
    {
        std::vector< double > value;
        getValue9( value, image, index_x, index_y );

        if ( getIs_color( ) )
        {
            intensituValues.at( 0 ).push_back( value.at( 0 ) );
            intensituValues.at( 1 ).push_back( value.at( 1 ) );
            intensituValues.at( 2 ).push_back( value.at( 2 ) );
        }
        else
        {
            intensituValues.at( 0 ).push_back( value.at( 0 ) );
        }
        double new_r = distance( index_x, index_y, getCenter( )( 0 ), getCenter( )( 1 ) );
        rs.push_back( new_r );
        drawGreenPoint( image_color, index_x, index_y );
    }
}

void
camera_model::VignettingCalib::addValue1( cv::Mat& image, cv::Mat& image_color, double& index_x, double& index_y, int threshold )
{
    double value_tmp;

    if ( getIs_color( ) )
    {
        cv::Vec3b value_tmp_cv = image.at< cv::Vec3b >( index_y, index_x );

        value_tmp = avgInThree( value_tmp_cv[0], value_tmp_cv[1], value_tmp_cv[2] );
    }
    else
        value_tmp = image.at< uchar >( index_y, index_x );

    if ( value_tmp < threshold )
        drawRedPoint( image_color, index_x, index_y );
    else
    {
        std::vector< double > value;
        getValue1( value, image, index_x, index_y );

        if ( getIs_color( ) )
        {
            intensituValues.at( 0 ).push_back( value.at( 0 ) );
            intensituValues.at( 1 ).push_back( value.at( 1 ) );
            intensituValues.at( 2 ).push_back( value.at( 2 ) );
        }
        else
        {
            intensituValues.at( 0 ).push_back( value.at( 0 ) );
        }
        double new_r = distance( index_x, index_y, getCenter( )( 0 ), getCenter( )( 1 ) );
        rs.push_back( new_r );
        drawGreenPoint( image_color, index_x, index_y );
    }
}

void
camera_model::VignettingCalib::inBoard( double& x_index, double& y_index )
{
    x_index = x_index < 0.0 ? 0.0 : x_index;
    x_index = x_index > ( double )getImageSize( ).width ? ( double )getImageSize( ).width : x_index;

    y_index = y_index < 0.0 ? 0.0 : y_index;
    y_index = y_index > ( double )getImageSize( ).height ? ( double )getImageSize( ).height : y_index;
}

void
camera_model::VignettingCalib::drawRedPoint( cv::Mat& image_color, int x_index, int y_index )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;
    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );
    cv::Scalar red( 0, 0, 255 );
    cv::circle( image_color,
                cv::Point( cvRound( x_index * drawMultiplier ), cvRound( y_index * drawMultiplier ) ),
                5,
                red,
                2,
                CV_AA,
                drawShiftBits );
}

void
camera_model::VignettingCalib::drawYellowPoint( cv::Mat& image_color, int x_index, int y_index )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;
    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );
    cv::Scalar red( 0, 0, 255 );
    cv::circle( image_color,
                cv::Point( cvRound( x_index * drawMultiplier ), cvRound( y_index * drawMultiplier ) ),
                5,
                yellow,
                2,
                CV_AA,
                drawShiftBits );
}

void
camera_model::VignettingCalib::drawGreenPoint( cv::Mat& image_color, int x_index, int y_index )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;
    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );
    cv::Scalar red( 0, 0, 255 );
    cv::circle( image_color,
                cv::Point( cvRound( x_index * drawMultiplier ), cvRound( y_index * drawMultiplier ) ),
                5,
                green,
                2,
                CV_AA,
                drawShiftBits );
}

void
camera_model::VignettingCalib::setBoardSize( const cv::Size& boardSize )
{
    mBoardSize = boardSize;
}
